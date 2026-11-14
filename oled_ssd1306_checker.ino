/**
 * @file oled_ssd1306_checker.ino
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief OLED SSD1306の画面検査ファームウェア(マイコン: RP2040 デュアルコア構成)
 *        - Core 0 (ドライバCPU): I2Cバス専有、OLED接続/抜去監視、GDDRAM転送、レジスタ制御
 *        - Core 1 (アプリCPU)  : 検査シーケンス制御、フレームバッファ描画、Serialログ出力
 * @version 0.2
 * @date 2026-11-14
 * @copyright Copyright (c) 2026 Chimipupu All Rights Reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------
const uint8_t OLED_CONTRAST_LEVELS[] = { 0x00U, 0x20U, 0x7FU, 0xFFU };
const uint8_t OLED_CONTRAST_LEVEL_CNT = (uint8_t)(sizeof(OLED_CONTRAST_LEVELS) / sizeof(OLED_CONTRAST_LEVELS[0]));

const uint8_t OLED_MUX_RATIOS[] = { 15U, 23U, 31U };
const uint8_t OLED_MUX_RATIO_CNT = (uint8_t)(sizeof(OLED_MUX_RATIOS) / sizeof(OLED_MUX_RATIOS[0]));

// -----------------------------------------------------------
#if 0
#define I2C_SCL_DEF           (400000) // I2C 400KHz
#else
#define I2C_SCL_DEF          (1000000) // I2C 1MHz
#endif

#define SCREEN_WIDTH            (128)
#define SCREEN_HEIGHT           (32)
#define SCREEN_BUF_SIZE         ((SCREEN_WIDTH * SCREEN_HEIGHT) / 8)
#define OLED_PAGE_CNT           (SCREEN_HEIGHT / 8)
#define OLED_I2C_CHUNK_SIZE     (16)
#define OLED_RESET              (-1)
#define OLED_I2C_ADDR           (0x3C)

#define OLED_CONTRAST_DEFAULT   (0x8F)
#define OLED_STATUS_OFF_MASK    (0x40)

#define WAIT_ALL_ON_MS          (1500)
#define WAIT_ENTIRE_ON_MS       (1000)
#define WAIT_ON_OFF_MS          (1000)
#define WAIT_BRIGHTNESS_MS      (700)
#define WAIT_PAGE_EACH_MS       (500)
#define WAIT_CHECKER_MS         (1000)
#define WAIT_INVERT_MS          (1000)
#define WAIT_HW_FLIP_MS         (700)
#define WAIT_OFFSET_EACH_MS     (500)
#define WAIT_MUX_EACH_MS        (700)
#define WAIT_TEXT_MS            (1500)
#define WAIT_ROT_EACH_MS        (600)
#define WAIT_SCROLL_EACH_MS     (1000)
#define POLL_INTERVAL_MS        (50)
#define FIFO_TIMEOUT_MS         (500)

// コア間FIFO コマンドID
#define DRV_CMD_FLUSH_FRAME     (0x01U)
#define DRV_CMD_SEND_CMD1       (0x02U)
#define DRV_CMD_SEND_CMD2       (0x03U)
#define DRV_CMD_PAGE_FRAME      (0x04U)
#define DRV_CMD_SCROLL_CTRL     (0x05U)
#define DRV_CMD_READ_STATUS     (0x06U)

// スクロール制御サブコマンド
#define DRV_SCROLL_STOP         (0x00U)
#define DRV_SCROLL_RIGHT        (0x01U)
#define DRV_SCROLL_LEFT         (0x02U)
#define DRV_SCROLL_DIAG_RIGHT   (0x03U)
#define DRV_SCROLL_DIAG_LEFT    (0x04U)

// 応答マーカー
#define DRV_RESP_MARKER         (0x80U)

// 32bit FIFOメッセージ パック/アンパックマクロ
#define DRV_MSG_PACK(cmd, a1, a2, a3) \
    (((uint32_t)(cmd) << 24U) | ((uint32_t)(a1) << 16U) | ((uint32_t)(a2) << 8U) | (uint32_t)(a3))

#define DRV_MSG_CMD(msg)        ((uint8_t)(((msg) >> 24U) & 0xFFU))
#define DRV_MSG_ARG1(msg)       ((uint8_t)(((msg) >> 16U) & 0xFFU))
#define DRV_MSG_ARG2(msg)       ((uint8_t)(((msg) >> 8U)  & 0xFFU))
#define DRV_MSG_ARG3(msg)       ((uint8_t)((msg) & 0xFFU))

// -----------------------------------------------------------
// [CPUコア間共有変数 (コア間状態通知)]
volatile bool g_is_oled_connected = false;
volatile bool g_is_core0_ready = false;

// -----------------------------------------------------------
// [Static変数]
static Adafruit_SSD1306 s_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static uint8_t s_tx_buf[SCREEN_BUF_SIZE];
static uint32_t s_last_poll_ms = 0;

// -----------------------------------------------------------
// [プロトタイプ宣言: Core 0 ドライバCPU]
static bool _drv_is_oled_ack(void);
static bool _drv_read_oled_status(uint8_t *p_status);
static bool _drv_flush_tx_buf(void);
static bool _drv_write_page_buffer(uint8_t page, const uint8_t *p_page_data, bool is_invert);
static bool _drv_exec_page_frame(uint8_t invert_mask);
static void _drv_exec_scroll(uint8_t mode, uint8_t start_page, uint8_t end_page);
static void _drv_handle_fifo_msg(uint32_t msg);

// -----------------------------------------------------------
// [プロトタイプ宣言: Core 1 アプリCPU RPCラッパー & ユーティリティ]
static bool _app_req_driver(uint32_t msg, uint8_t *p_ret_val);
static bool _app_flush_display(void);
static bool _app_send_cmd1(uint8_t cmd);
static bool _app_send_cmd2(uint8_t cmd, uint8_t param);
static bool _app_page_frame(uint8_t invert_mask);
static bool _app_scroll_ctrl(uint8_t mode, uint8_t start_page, uint8_t end_page);
static bool _app_read_status(uint8_t *p_status);
static bool _wait_ms_with_detect(uint32_t wait_ms);

// 検査関数ポインタ型
typedef bool (*oled_test_func_t)(void);

// 検査関数プロトタイプ
static bool _test_all_pixels(void);
static bool _test_entire_display_on(void);
static bool _test_display_on_off(void);
static bool _test_brightness(void);
static bool _test_page_switch(void);
static bool _test_checkerboard(void);
static bool _test_invert_display(void);
static bool _test_hw_flip(void);
static bool _test_display_offset(void);
static bool _test_multiplex_ratio(void);
static bool _test_full_text(void);
static bool _test_rotation(void);
static bool _test_scroll(void);

// 検査関数ポインタテーブル
static const oled_test_func_t s_oled_test_func_tbl[] = {
    // 全画面のピクセル点灯検査（GDDRAM白埋めによるドット抜け・輝度ムラ確認）
    _test_all_pixels,

    // ハードウェア全画素強制点灯検査（0xA5: Entire Display ON / 0xA4: Resume）
    _test_entire_display_on,

    // 画面のON/OFF・スリープ復帰 & I2Cステータス読み出し検査（0xAE / 0xAF）
    _test_display_on_off,

    // 画面の輝度（コントラスト）変更検査（0x81）
    _test_brightness,

    // GDDRAM ページ切り替え・ページアドレッシング検査（Page 0 -> Page 3）
    _test_page_switch,

    // 市松模様テスト（隣接ピクセル干渉・全画素ON/OFF網羅確認）
    _test_checkerboard,

    // ハードウェア白黒反転検査（0xA7: Inverse / 0xA6: Normal）
    _test_invert_display,

    // ハードウェア左右・上下ミラー反転検査（0xA0/0xA1, 0xC0/0xC8）
    _test_hw_flip,

    // ハードウェア垂直オフセットシフト検査（0xD3: Display Offset）
    _test_display_offset,

    // マルチプレクス比（スキャンライン数制限）検査（0xA8: Multiplex Ratio）
    _test_multiplex_ratio,

    // 標準フォント(6x8px)を全画面128x32分表示（21文字 x 4行 = 84文字）
    _test_full_text,

    // ソフトウェア画面回転テスト（0° -> 90° -> 180° -> 270°）
    _test_rotation,

    // ハードウェアスクロールテスト（水平右/左 + 斜め右/左）
    _test_scroll,
};

const uint8_t OLED_TEST_FUNC_TBL_CNT = (uint8_t)(sizeof(s_oled_test_func_tbl) / sizeof(s_oled_test_func_tbl[0]));

// ===========================================================
// [Core 0 : ドライバCPU 実装領域 (I2Cバス完全専有)]
// ===========================================================

/**
 * @brief I2Cバス上でOLEDの応答(ACK)があるか確認
 */
static bool _drv_is_oled_ack(void)
{
    uint8_t err = 0;

    Wire.beginTransmission(OLED_I2C_ADDR);
    err = Wire.endTransmission();

    return (err == 0);
}

/**
 * @brief SSD1306のステータスレジスタをI2C経由で1バイト読み出す
 */
static bool _drv_read_oled_status(uint8_t *p_status)
{
    uint8_t read_cnt = 0;

    if (p_status == NULL) {
        return false;
    }

    read_cnt = Wire.requestFrom((uint8_t)OLED_I2C_ADDR, (uint8_t)1);
    if ((read_cnt == 1) && (Wire.available() > 0)) {
        *p_status = (uint8_t)Wire.read();
        return true;
    }

    return false;
}

/**
 * @brief ドライバ側ダブルバッファ(s_tx_buf)をGDDRAMへ全画面転送する
 */
static bool _drv_flush_tx_buf(void)
{
    uint16_t offset = 0;
    uint8_t i = 0;
    uint8_t err = 0;

    s_display.ssd1306_command(SSD1306_COLUMNADDR);
    s_display.ssd1306_command(0x00U);
    s_display.ssd1306_command((uint8_t)(SCREEN_WIDTH - 1));
    s_display.ssd1306_command(SSD1306_PAGEADDR);
    s_display.ssd1306_command(0x00U);
    s_display.ssd1306_command((uint8_t)(OLED_PAGE_CNT - 1));

    for (offset = 0; offset < SCREEN_BUF_SIZE; offset += OLED_I2C_CHUNK_SIZE) {
        Wire.beginTransmission(OLED_I2C_ADDR);
        Wire.write((uint8_t)0x40U); // Co=0, D/C#=1 (GDDRAMデータ書き込み)
        for (i = 0; i < OLED_I2C_CHUNK_SIZE; i++) {
            Wire.write(s_tx_buf[offset + i]);
        }
        err = Wire.endTransmission();
        if (err != 0) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Page Addressing Modeで指定ページ(128バイト)に任意バッファを書き込む
 */
static bool _drv_write_page_buffer(uint8_t page, const uint8_t *p_page_data, bool is_invert)
{
    uint8_t offset = 0;
    uint8_t i = 0;
    uint8_t err = 0;
    uint8_t data_byte = 0;

    if (p_page_data == NULL) {
        return false;
    }

    s_display.ssd1306_command((uint8_t)(0xB0U | (page & 0x07U)));
    s_display.ssd1306_command(SSD1306_SETLOWCOLUMN | 0x00U);
    s_display.ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x00U);

    for (offset = 0; offset < SCREEN_WIDTH; offset += OLED_I2C_CHUNK_SIZE) {
        Wire.beginTransmission(OLED_I2C_ADDR);
        Wire.write((uint8_t)0x40U);
        for (i = 0; i < OLED_I2C_CHUNK_SIZE; i++) {
            data_byte = p_page_data[offset + i];
            if (is_invert) {
                data_byte = (uint8_t)(~data_byte);
            }
            Wire.write(data_byte);
        }
        err = Wire.endTransmission();
        if (err != 0) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Page Addressing Modeで全ページを転送後、Horizontal Modeへ復帰する
 */
static bool _drv_exec_page_frame(uint8_t invert_mask)
{
    uint8_t page = 0;
    bool is_ok = true;
    bool is_inv = false;

    s_display.ssd1306_command(SSD1306_MEMORYMODE);
    s_display.ssd1306_command(0x02U);

    for (page = 0; page < OLED_PAGE_CNT; page++) {
        is_inv = (((invert_mask >> page) & 0x01U) != 0);
        if (!_drv_write_page_buffer(page, &s_tx_buf[(uint16_t)page * SCREEN_WIDTH], is_inv)) {
            is_ok = false;
            break;
        }
    }

    // Horizontal Addressing Mode (0x00) へ必ず復帰
    s_display.ssd1306_command(SSD1306_MEMORYMODE);
    s_display.ssd1306_command(0x00U);
    s_display.ssd1306_command(SSD1306_COLUMNADDR);
    s_display.ssd1306_command(0x00U);
    s_display.ssd1306_command((uint8_t)(SCREEN_WIDTH - 1));
    s_display.ssd1306_command(SSD1306_PAGEADDR);
    s_display.ssd1306_command(0x00U);
    s_display.ssd1306_command((uint8_t)(OLED_PAGE_CNT - 1));

    return is_ok;
}

/**
 * @brief ハードウェアスクロール制御を実行する
 */
static void _drv_exec_scroll(uint8_t mode, uint8_t start_page, uint8_t end_page)
{
    switch (mode) {
        case DRV_SCROLL_RIGHT:
            s_display.startscrollright(start_page, end_page);
            break;
        case DRV_SCROLL_LEFT:
            s_display.startscrollleft(start_page, end_page);
            break;
        case DRV_SCROLL_DIAG_RIGHT:
            s_display.startscrolldiagright(start_page, end_page);
            break;
        case DRV_SCROLL_DIAG_LEFT:
            s_display.startscrolldiagleft(start_page, end_page);
            break;
        case DRV_SCROLL_STOP:
        default:
            s_display.stopscroll();
            break;
    }
}

/**
 * @brief Core 1から受信したFIFOコマンドを解析・実行する
 */
static void _drv_handle_fifo_msg(uint32_t msg)
{
    uint8_t cmd = DRV_MSG_CMD(msg);
    uint8_t arg1 = DRV_MSG_ARG1(msg);
    uint8_t arg2 = DRV_MSG_ARG2(msg);
    uint8_t arg3 = DRV_MSG_ARG3(msg);
    uint8_t *p_src_buf = NULL;
    uint8_t status_byte = 0;
    bool is_ok = false;

    if (!g_is_oled_connected) {
        rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, 0U, 0U, 0U));
        return;
    }

    switch (cmd) {
        case DRV_CMD_FLUSH_FRAME:
            p_src_buf = s_display.getBuffer();
            if (p_src_buf != NULL) {
                // 512バイトを即座にドライバ側バッファへ退避し、Core 1へACKを先行返却
                memcpy(s_tx_buf, p_src_buf, SCREEN_BUF_SIZE);
                rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, 1U, 0U, 0U));
                // I2C転送はCore 1の次フレーム描画と並列で実行
                if (!_drv_flush_tx_buf()) {
                    g_is_oled_connected = false;
                }
            } else {
                rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, 0U, 0U, 0U));
            }
            return;

        case DRV_CMD_SEND_CMD1:
            s_display.ssd1306_command(arg1);
            is_ok = true;
            break;

        case DRV_CMD_SEND_CMD2:
            s_display.ssd1306_command(arg1);
            s_display.ssd1306_command(arg2);
            is_ok = true;
            break;

        case DRV_CMD_PAGE_FRAME:
            p_src_buf = s_display.getBuffer();
            if (p_src_buf != NULL) {
                memcpy(s_tx_buf, p_src_buf, SCREEN_BUF_SIZE);
                rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, 1U, 0U, 0U));
                if (!_drv_exec_page_frame(arg1)) {
                    g_is_oled_connected = false;
                }
            } else {
                rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, 0U, 0U, 0U));
            }
            return;

        case DRV_CMD_SCROLL_CTRL:
            _drv_exec_scroll(arg1, arg2, arg3);
            is_ok = true;
            break;

        case DRV_CMD_READ_STATUS:
            is_ok = _drv_read_oled_status(&status_byte);
            break;

        default:
            is_ok = false;
            break;
    }

    rp2040.fifo.push(DRV_MSG_PACK(DRV_RESP_MARKER, (is_ok ? 1U : 0U), status_byte, 0U));
}

// ===========================================================
// [Core 1 : アプリCPU 実装領域 (RPC要求 & 検査ロジック)]
// ===========================================================

/**
 * @brief Core 0ドライバへFIFO経由でコマンドを発行し応答を待つ
 */
static bool _app_req_driver(uint32_t msg, uint8_t *p_ret_val)
{
    uint32_t resp = 0;
    uint32_t start_ms = millis();
    bool is_got_resp = false;

    rp2040.fifo.push(msg);

    while ((millis() - start_ms) < FIFO_TIMEOUT_MS) {
        if (rp2040.fifo.pop_nb(&resp)) {
            is_got_resp = true;
            break;
        }
        tight_loop_contents();
    }

    if (!is_got_resp) {
        return false;
    }

    if (p_ret_val != NULL) {
        *p_ret_val = DRV_MSG_ARG2(resp);
    }

    return (DRV_MSG_ARG1(resp) != 0);
}

/**
 * @brief Core 1が描画したフレームバッファの転送をCore 0へ要求
 */
static bool _app_flush_display(void)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_FLUSH_FRAME, 0U, 0U, 0U), NULL);
}

/**
 * @brief SSD1306の1バイトコマンド送信をCore 0へ要求
 */
static bool _app_send_cmd1(uint8_t cmd)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_SEND_CMD1, cmd, 0U, 0U), NULL);
}

/**
 * @brief SSD1306の2バイトコマンド送信をCore 0へ要求
 */
static bool _app_send_cmd2(uint8_t cmd, uint8_t param)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_SEND_CMD2, cmd, param, 0U), NULL);
}

/**
 * @brief Page Addressing Modeでのページ単位転送をCore 0へ要求
 */
static bool _app_page_frame(uint8_t invert_mask)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_PAGE_FRAME, invert_mask, 0U, 0U), NULL);
}

/**
 * @brief ハードウェアスクロール制御をCore 0へ要求
 */
static bool _app_scroll_ctrl(uint8_t mode, uint8_t start_page, uint8_t end_page)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_SCROLL_CTRL, mode, start_page, end_page), NULL);
}

/**
 * @brief SSD1306ステータス読み出しをCore 0へ要求
 */
static bool _app_read_status(uint8_t *p_status)
{
    return _app_req_driver(DRV_MSG_PACK(DRV_CMD_READ_STATUS, 0U, 0U, 0U), p_status);
}

/**
 * @brief Core 0が監視する接続フラグを見ながら指定時間待機する
 */
static bool _wait_ms_with_detect(uint32_t wait_ms)
{
    uint32_t elapsed = 0;
    uint32_t step = 0;

    while (elapsed < wait_ms) {
        if (!g_is_oled_connected) {
            return false;
        }

        step = ((wait_ms - elapsed) > POLL_INTERVAL_MS) ? POLL_INTERVAL_MS : (wait_ms - elapsed);
        delay(step);
        elapsed += step;
    }

    return g_is_oled_connected;
}

/**
 * @brief 全画面のピクセル点灯検査（ドット抜け・輝度ムラ確認）
 */
static bool _test_all_pixels(void)
{
    Serial.println(F("All Pixels ON Test"));

    if (!_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U)) {
        return false;
    }
    s_display.setRotation(0);
    s_display.fillScreen(SSD1306_WHITE);
    if (!_app_flush_display()) {
        return false;
    }

    return _wait_ms_with_detect(WAIT_ALL_ON_MS);
}

/**
 * @brief ハードウェア全画素強制点灯検査（0xA5: Entire Display ON / 0xA4: Resume）
 */
static bool _test_entire_display_on(void)
{
    Serial.println(F("Entire Display ON (0xA5/0xA4) Test"));

    s_display.clearDisplay();
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(16, 8);
    s_display.print(F("RAM DATA"));
    if (!_app_flush_display()) {
        return false;
    }

    if (!_wait_ms_with_detect(WAIT_ENTIRE_ON_MS)) {
        return false;
    }

    // GDDRAMを書き換えずに全画素強制点灯
    if (!_app_send_cmd1(SSD1306_DISPLAYALLON)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_ENTIRE_ON_MS)) {
        (void)_app_send_cmd1(SSD1306_DISPLAYALLON_RESUME);
        return false;
    }

    // GDDRAM表示へ復帰
    if (!_app_send_cmd1(SSD1306_DISPLAYALLON_RESUME)) {
        return false;
    }
    return _wait_ms_with_detect(WAIT_ENTIRE_ON_MS);
}

/**
 * @brief 画面のON/OFF・スリープ復帰 & I2Cステータス読み出し検査
 */
static bool _test_display_on_off(void)
{
    uint8_t status_byte = 0;
    bool is_read_ok = false;

    Serial.println(F("Display ON/OFF & Status Read Test"));

    s_display.clearDisplay();
    s_display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(22, 12);
    s_display.print(F("ON / OFF TEST"));
    if (!_app_flush_display()) {
        return false;
    }

    if (!_wait_ms_with_detect(WAIT_ON_OFF_MS)) {
        return false;
    }

    // 画面OFF（スリープ移行）
    if (!_app_send_cmd1(SSD1306_DISPLAYOFF)) {
        return false;
    }
    is_read_ok = _app_read_status(&status_byte);
    if (is_read_ok) {
        Serial.print(F("  Status(OFF): 0x"));
        Serial.print(status_byte, HEX);
        Serial.println(((status_byte & OLED_STATUS_OFF_MASK) != 0) ? F(" [OK: D6=1]") : F(" [WARN: D6!=1]"));
    } else {
        Serial.println(F("  Status(OFF): I2C Read NACK (Write-only module)"));
    }

    if (!_wait_ms_with_detect(WAIT_ON_OFF_MS)) {
        (void)_app_send_cmd1(SSD1306_DISPLAYON);
        return false;
    }

    // 画面ON（スリープ復帰・RAM内容維持確認）
    if (!_app_send_cmd1(SSD1306_DISPLAYON)) {
        return false;
    }
    is_read_ok = _app_read_status(&status_byte);
    if (is_read_ok) {
        Serial.print(F("  Status(ON) : 0x"));
        Serial.print(status_byte, HEX);
        Serial.println(((status_byte & OLED_STATUS_OFF_MASK) == 0) ? F(" [OK: D6=0]") : F(" [WARN: D6!=0]"));
    }

    return _wait_ms_with_detect(WAIT_ON_OFF_MS);
}

/**
 * @brief 画面の輝度（コントラスト）変更検査（0x81）
 */
static bool _test_brightness(void)
{
    uint8_t i = 0;

    Serial.println(F("Brightness (Contrast) Test"));

    for (i = 0; i < OLED_CONTRAST_LEVEL_CNT; i++) {
        s_display.clearDisplay();
        s_display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
        s_display.setTextSize(1);
        s_display.setTextColor(SSD1306_WHITE);
        s_display.setCursor(4, 18);
        s_display.print(F("CONTRAST: 0x"));
        if (OLED_CONTRAST_LEVELS[i] < 0x10U) {
            s_display.print(F("0"));
        }
        s_display.print(OLED_CONTRAST_LEVELS[i], HEX);
        if (!_app_flush_display()) {
            return false;
        }

        if (!_app_send_cmd2(SSD1306_SETCONTRAST, OLED_CONTRAST_LEVELS[i])) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_BRIGHTNESS_MS)) {
            (void)_app_send_cmd2(SSD1306_SETCONTRAST, OLED_CONTRAST_DEFAULT);
            return false;
        }
    }

    // デフォルト輝度へ復帰
    return _app_send_cmd2(SSD1306_SETCONTRAST, OLED_CONTRAST_DEFAULT);
}

/**
 * @brief GDDRAM ページ切り替え検査（Page 0〜3のラベル表示＋対象ページ反転）
 */
static bool _test_page_switch(void)
{
    uint8_t active_page = 0;
    uint8_t target_page = 0;

    Serial.println(F("Page Switch (Page Addressing Mode) Test"));

    // フレームバッファ上に各ページ(8px高)のラベルを用意
    s_display.clearDisplay();
    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);
    for (target_page = 0; target_page < OLED_PAGE_CNT; target_page++) {
        s_display.setCursor(4, (int16_t)(target_page * 8));
        s_display.print(F("PAGE "));
        s_display.print(target_page);
    }

    for (active_page = 0; active_page < OLED_PAGE_CNT; active_page++) {
        if (!_app_page_frame((uint8_t)(1U << active_page))) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_PAGE_EACH_MS)) {
            return false;
        }
    }

    s_display.clearDisplay();
    return _app_flush_display();
}

/**
 * @brief 市松模様テスト（隣接ピクセル干渉・全画素ON/OFF網羅確認）
 */
static bool _test_checkerboard(void)
{
    uint8_t *p_buf = NULL;
    uint16_t i = 0;
    uint8_t phase = 0;
    uint8_t even_pattern = 0;
    uint8_t odd_pattern = 0;

    Serial.println(F("Checkerboard Test"));

    if (!_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U)) {
        return false;
    }
    s_display.setRotation(0);
    p_buf = s_display.getBuffer();

    if (p_buf == NULL) {
        return false;
    }

    for (phase = 0; phase < 2; phase++) {
        even_pattern = (phase == 0) ? 0x55U : 0xAAU;
        odd_pattern  = (phase == 0) ? 0xAAU : 0x55U;

        for (i = 0; i < SCREEN_BUF_SIZE; i++) {
            p_buf[i] = ((i & 0x01U) == 0) ? even_pattern : odd_pattern;
        }
        if (!_app_flush_display()) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_CHECKER_MS)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief ハードウェア白黒反転検査（0xA7: Inverse / 0xA6: Normal）
 */
static bool _test_invert_display(void)
{
    Serial.println(F("Hardware Invert Display Test"));

    s_display.clearDisplay();
    s_display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(16, 8);
    s_display.print(F("INVERTED"));
    if (!_app_flush_display()) {
        return false;
    }

    if (!_app_send_cmd1(SSD1306_INVERTDISPLAY)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_INVERT_MS)) {
        (void)_app_send_cmd1(SSD1306_NORMALDISPLAY);
        return false;
    }

    if (!_app_send_cmd1(SSD1306_NORMALDISPLAY)) {
        return false;
    }
    return _wait_ms_with_detect(WAIT_INVERT_MS);
}

/**
 * @brief ハードウェア左右・上下ミラー反転検査（Segment Re-map & COM Scan Dir）
 */
static bool _test_hw_flip(void)
{
    uint8_t mode = 0;
    uint8_t seg_cmd = 0;
    uint8_t com_cmd = 0;

    Serial.println(F("Hardware Mirror Flip Test"));

    s_display.clearDisplay();
    s_display.drawTriangle(4, 26, 14, 6, 24, 26, SSD1306_WHITE);
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(32, 8);
    s_display.print(F("HW FLIP"));
    if (!_app_flush_display()) {
        return false;
    }

    for (mode = 0; mode < 4; mode++) {
        seg_cmd = ((mode & 0x01U) != 0) ? SSD1306_SEGREMAP : (uint8_t)(SSD1306_SEGREMAP | 0x01U);
        com_cmd = ((mode & 0x02U) != 0) ? SSD1306_COMSCANINC : SSD1306_COMSCANDEC;

        if ((!_app_send_cmd1(seg_cmd)) || (!_app_send_cmd1(com_cmd))) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_HW_FLIP_MS)) {
            (void)_app_send_cmd1(SSD1306_SEGREMAP | 0x01U);
            (void)_app_send_cmd1(SSD1306_COMSCANDEC);
            return false;
        }
    }

    // 通常向きへ復帰
    if ((!_app_send_cmd1(SSD1306_SEGREMAP | 0x01U)) || (!_app_send_cmd1(SSD1306_COMSCANDEC))) {
        return false;
    }
    return true;
}

/**
 * @brief ハードウェア垂直オフセットシフト検査（0xD3: Display Offset）
 */
static bool _test_display_offset(void)
{
    uint8_t step = 0;
    uint8_t offset = 0;

    Serial.println(F("Hardware Display Offset Shift Test"));

    s_display.clearDisplay();
    s_display.drawFastHLine(0, 0, SCREEN_WIDTH, SSD1306_WHITE);
    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(16, 12);
    s_display.print(F("VERTICAL OFFSET"));
    s_display.drawFastHLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SSD1306_WHITE);
    if (!_app_flush_display()) {
        return false;
    }

    for (step = 0; step <= 4; step++) {
        offset = (uint8_t)((step * 8U) % SCREEN_HEIGHT);
        if (!_app_send_cmd2(SSD1306_SETDISPLAYOFFSET, offset)) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_OFFSET_EACH_MS)) {
            (void)_app_send_cmd2(SSD1306_SETDISPLAYOFFSET, 0x00U);
            return false;
        }
    }

    // オフセット0へ復帰
    return _app_send_cmd2(SSD1306_SETDISPLAYOFFSET, 0x00U);
}

/**
 * @brief マルチプレクス比（スキャンライン数制限）検査（0xA8: Multiplex Ratio）
 */
static bool _test_multiplex_ratio(void)
{
    uint8_t i = 0;

    Serial.println(F("Multiplex Ratio Test"));

    s_display.fillScreen(SSD1306_WHITE);
    if (!_app_flush_display()) {
        return false;
    }

    for (i = 0; i < OLED_MUX_RATIO_CNT; i++) {
        if (!_app_send_cmd2(SSD1306_SETMULTIPLEX, OLED_MUX_RATIOS[i])) {
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_MUX_EACH_MS)) {
            (void)_app_send_cmd2(SSD1306_SETMULTIPLEX, (uint8_t)(SCREEN_HEIGHT - 1));
            return false;
        }
    }

    // 全ライン駆動へ復帰
    return _app_send_cmd2(SSD1306_SETMULTIPLEX, (uint8_t)(SCREEN_HEIGHT - 1));
}

/**
 * @brief 標準フォント(6x8px)を全画面128x32分表示（21文字 x 4行 = 84文字）
 */
static bool _test_full_text(void)
{
    const char *p_chars = "0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ!@#$%&*+-=/?";
    uint8_t char_len = 47;
    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t idx = 0;

    Serial.println(F("Full Text (21x4) Test"));

    s_display.clearDisplay();
    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setTextWrap(false);

    for (row = 0; row < 4; row++) {
        s_display.setCursor(0, (int16_t)(row * 8));
        for (col = 0; col < 21; col++) {
            idx = (uint8_t)(((row * 21) + col) % char_len);
            s_display.write((uint8_t)p_chars[idx]);
        }
    }
    if (!_app_flush_display()) {
        return false;
    }

    return _wait_ms_with_detect(WAIT_TEXT_MS);
}

/**
 * @brief 画面の回転テスト（0° -> 90° -> 180° -> 270°）
 */
static bool _test_rotation(void)
{
    uint8_t rot = 0;

    Serial.println(F("Software Rotation Test"));

    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);

    for (rot = 0; rot < 4; rot++) {
        s_display.clearDisplay();
        s_display.setRotation(rot);
        s_display.drawRect(0, 0, s_display.width(), s_display.height(), SSD1306_WHITE);

        s_display.setCursor(4, 4);
        s_display.print(F("ROT:"));
        s_display.print(rot * 90);

        if (!_app_flush_display()) {
            s_display.setRotation(0);
            return false;
        }

        if (!_wait_ms_with_detect(WAIT_ROT_EACH_MS)) {
            s_display.setRotation(0);
            return false;
        }
    }

    s_display.setRotation(0);
    return true;
}

/**
 * @brief ハードウェアスクロールテスト（水平右/左 + 斜め右/左）
 */
static bool _test_scroll(void)
{
    Serial.println(F("Hardware Scroll (Horizontal & Diagonal) Test"));

    s_display.clearDisplay();
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(10, 8);
    s_display.print(F("SCROLL OK"));
    if (!_app_flush_display()) {
        return false;
    }

    // 水平右スクロール
    if (!_app_scroll_ctrl(DRV_SCROLL_RIGHT, 0x00U, 0x03U)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        (void)_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U);
        return false;
    }

    // 水平左スクロール
    if (!_app_scroll_ctrl(DRV_SCROLL_LEFT, 0x00U, 0x03U)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        (void)_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U);
        return false;
    }

    // 斜め右スクロール（垂直＋水平同時）
    if (!_app_scroll_ctrl(DRV_SCROLL_DIAG_RIGHT, 0x00U, 0x03U)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        (void)_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U);
        return false;
    }

    // 斜め左スクロール（垂直＋水平同時）
    if (!_app_scroll_ctrl(DRV_SCROLL_DIAG_LEFT, 0x00U, 0x03U)) {
        return false;
    }
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        (void)_app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U);
        return false;
    }

    return _app_scroll_ctrl(DRV_SCROLL_STOP, 0U, 0U);
}

// ===========================================================
// [エントリポイント: Core 0 (ドライバCPU)]
// ===========================================================

/**
 * @brief Core 0 初期化 (I2Cバス・ドライバ層初期化)
 */
void setup(void)
{
    Wire.begin();
    Wire.setClock(I2C_SCL_DEF);
    s_last_poll_ms = millis();
    g_is_core0_ready = true;
}

/**
 * @brief Core 0 メインループ (FIFOコマンド実行 & ホットプラグ監視)
 */
void loop(void)
{
    uint32_t msg = 0;
    uint32_t now_ms = 0;
    bool is_init_ok = false;

    // 1. Core 1からのドライバコマンド要求を最優先処理
    while (rp2040.fifo.pop_nb(&msg)) {
        _drv_handle_fifo_msg(msg);
    }

    // 2. 定期I2C ACKポーリングによる接続・抜去監視
    now_ms = millis();
    if ((now_ms - s_last_poll_ms) >= POLL_INTERVAL_MS) {
        s_last_poll_ms = now_ms;

        if (!g_is_oled_connected) {
            if (_drv_is_oled_ack()) {
                delay(50);
                is_init_ok = s_display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
                if (is_init_ok) {
                    g_is_oled_connected = true;
                }
            }
        } else {
            if (!_drv_is_oled_ack()) {
                g_is_oled_connected = false;
            }
        }
    }

    tight_loop_contents();
}

// ===========================================================
// [エントリポイント: Core 1 (アプリCPU)]
// ===========================================================

/**
 * @brief Core 1 初期化 (Serialログ・アプリ層初期化)
 */
void setup1(void)
{
    Serial.begin(115200);
    while (!g_is_core0_ready) {
        tight_loop_contents();
    }
    Serial.println(F("OLED Dual-Core Batch Inspection Ready (Core0:Driver / Core1:App)."));
}

/**
 * @brief Core 1 メインループ (検査シーケンス実行)
 */
void loop1(void)
{
    static bool s_is_prev_connected = false;
    uint8_t i = 0;
    bool is_connected = g_is_oled_connected;
    bool is_ret = false;

    // 接続状態のエッジ検出とログ通知
    if (is_connected && (!s_is_prev_connected)) {
        Serial.println(F("=== New OLED Detected: Start Inspection ==="));
    } else if ((!is_connected) && s_is_prev_connected) {
        Serial.println(F("OLED Disconnected. Waiting for next unit..."));
    }
    s_is_prev_connected = is_connected;

    if (!is_connected) {
        delay(10);
        return;
    }

    // 検査関数のテーブルを順次実行
    for (i = 0; i < OLED_TEST_FUNC_TBL_CNT; i++) {
        if (!g_is_oled_connected) {
            return;
        }

        Serial.print(F("["));
        Serial.print(i + 1);
        Serial.print(F("/"));
        Serial.print(OLED_TEST_FUNC_TBL_CNT);
        Serial.println(F("] Executing..."));

        is_ret = s_oled_test_func_tbl[i]();
        if (!is_ret) {
            return;
        }
    }
}