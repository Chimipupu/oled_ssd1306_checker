/**
 * @file oled_ssd1306_checker.ino
 * @author Chimipupu(https://github.com/Chimipupu)
 * @brief OLED SSD1306の画面検査ファームウェア(マイコン: RP2040)
 * @version 0.1
 * @date 2026-11-14
 * @copyright Copyright (c) 2026 Chimipupu All Rights Reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------
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

static Adafruit_SSD1306 s_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool s_is_connected = false;

static bool _is_oled_ack(void);
static bool _read_oled_status(uint8_t *p_status);
static bool _write_page_pattern(uint8_t page, uint8_t pattern);
static bool _wait_ms_with_detect(uint32_t wait_ms);
static bool _write_page_buffer(uint8_t page, const uint8_t *p_page_data, bool is_invert);

// 検査関数ポインタ
typedef bool (*oled_test_func_t)(void);

// 検査関数プロト
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
// -----------------------------------------------------------
// [Static関数]

/**
 * @brief I2Cバス上でOLEDの応答(ACK)があるか確認
 */
static bool _is_oled_ack(void)
{
    uint8_t err = 0;

    Wire.beginTransmission(OLED_I2C_ADDR);
    err = Wire.endTransmission();

    return (err == 0);
}

/**
 * @brief SSD1306のステータスレジスタをI2C経由で1バイト読み出す
 */
static bool _read_oled_status(uint8_t *p_status)
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
 * @brief Page Addressing Modeで指定ページ(128バイト)にパターンを書き込む
 */
static bool _write_page_pattern(uint8_t page, uint8_t pattern)
{
    uint8_t offset = 0;
    uint8_t i = 0;
    uint8_t err = 0;

    // ページアドレス(0xB0〜0xB7)とカラム開始位置(0x00, 0x10)を指定
    s_display.ssd1306_command((uint8_t)(0xB0U | (page & 0x07U)));
    s_display.ssd1306_command(SSD1306_SETLOWCOLUMN | 0x00U);
    s_display.ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x00U);

    // I2Cバッファ溢れ防止のため16バイト単位で分割送信
    for (offset = 0; offset < SCREEN_WIDTH; offset += OLED_I2C_CHUNK_SIZE) {
        Wire.beginTransmission(OLED_I2C_ADDR);
        Wire.write((uint8_t)0x40U); // Co=0, D/C#=1 (GDDRAMデータ書き込み)
        for (i = 0; i < OLED_I2C_CHUNK_SIZE; i++) {
            Wire.write(pattern);
        }
        err = Wire.endTransmission();
        if (err != 0) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 抜去を監視しながら指定時間待機する（抜かれたらfalseを返す）
 */
static bool _wait_ms_with_detect(uint32_t wait_ms)
{
    uint32_t elapsed = 0;
    uint32_t step = 0;

    while (elapsed < wait_ms) {
        if (!_is_oled_ack()) {
            s_is_connected = false;
            Serial.println(F("OLED Disconnected. Waiting for next unit..."));
            return false;
        }

        step = ((wait_ms - elapsed) > POLL_INTERVAL_MS) ? POLL_INTERVAL_MS : (wait_ms - elapsed);
        delay(step);
        elapsed += step;
    }

    return true;
}

/**
 * @brief 全画面のピクセル点灯検査（ドット抜け・輝度ムラ確認）
 */
static bool _test_all_pixels(void)
{
    Serial.println(F("All Pixels ON Test"));

    s_display.stopscroll();
    s_display.setRotation(0);
    s_display.fillScreen(SSD1306_WHITE);
    s_display.display();

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
    s_display.display();

    if (!_wait_ms_with_detect(WAIT_ENTIRE_ON_MS)) {
        return false;
    }

    // GDDRAMを書き換えずに全画素強制点灯
    s_display.ssd1306_command(SSD1306_DISPLAYALLON);
    if (!_wait_ms_with_detect(WAIT_ENTIRE_ON_MS)) {
        s_display.ssd1306_command(SSD1306_DISPLAYALLON_RESUME);
        return false;
    }

    // GDDRAM表示へ復帰
    s_display.ssd1306_command(SSD1306_DISPLAYALLON_RESUME);
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
    s_display.display();

    if (!_wait_ms_with_detect(WAIT_ON_OFF_MS)) {
        return false;
    }

    // 画面OFF（スリープ移行）
    s_display.ssd1306_command(SSD1306_DISPLAYOFF);
    is_read_ok = _read_oled_status(&status_byte);
    if (is_read_ok) {
        Serial.print(F("  Status(OFF): 0x"));
        Serial.print(status_byte, HEX);
        Serial.println(((status_byte & OLED_STATUS_OFF_MASK) != 0) ? F(" [OK: D6=1]") : F(" [WARN: D6!=1]"));
    } else {
        Serial.println(F("  Status(OFF): I2C Read NACK (Write-only module)"));
    }

    if (!_wait_ms_with_detect(WAIT_ON_OFF_MS)) {
        s_display.ssd1306_command(SSD1306_DISPLAYON);
        return false;
    }

    // 画面ON（スリープ復帰・RAM内容維持確認）
    s_display.ssd1306_command(SSD1306_DISPLAYON);
    is_read_ok = _read_oled_status(&status_byte);
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
    const uint8_t p_levels[] = { 0x00, 0x20, 0x7F, 0xFF };
    uint8_t level_cnt = (uint8_t)(sizeof(p_levels) / sizeof(p_levels[0]));
    uint8_t i = 0;

    Serial.println(F("Brightness (Contrast) Test"));

    for (i = 0; i < level_cnt; i++) {
        s_display.clearDisplay();
        s_display.fillRect(0, 0, SCREEN_WIDTH, 12, SSD1306_WHITE);
        s_display.setTextSize(1);
        s_display.setTextColor(SSD1306_WHITE);
        s_display.setCursor(4, 18);
        s_display.print(F("CONTRAST: 0x"));
        if (p_levels[i] < 0x10) {
            s_display.print(F("0"));
        }
        s_display.print(p_levels[i], HEX);
        s_display.display();

        // コントラストレジスタ設定
        s_display.ssd1306_command(SSD1306_SETCONTRAST);
        s_display.ssd1306_command(p_levels[i]);

        if (!_wait_ms_with_detect(WAIT_BRIGHTNESS_MS)) {
            s_display.ssd1306_command(SSD1306_SETCONTRAST);
            s_display.ssd1306_command(OLED_CONTRAST_DEFAULT);
            return false;
        }
    }

    // デフォルト輝度へ復帰
    s_display.ssd1306_command(SSD1306_SETCONTRAST);
    s_display.ssd1306_command(OLED_CONTRAST_DEFAULT);
    return true;
}

/**
 * @brief Page Addressing Modeで指定ページ(128バイト)に任意バッファを書き込む
 */
static bool _write_page_buffer(uint8_t page, const uint8_t *p_page_data, bool is_invert)
{
    uint8_t offset = 0;
    uint8_t i = 0;
    uint8_t err = 0;
    uint8_t data_byte = 0;

    if (p_page_data == NULL) {
        return false;
    }

    // ページアドレス(0xB0〜0xB7)とカラム開始位置(0x00, 0x10)を指定
    s_display.ssd1306_command((uint8_t)(0xB0U | (page & 0x07U)));
    s_display.ssd1306_command(SSD1306_SETLOWCOLUMN | 0x00U);
    s_display.ssd1306_command(SSD1306_SETHIGHCOLUMN | 0x00U);

    // I2Cバッファ溢れ防止のため16バイト単位で分割送信
    for (offset = 0; offset < SCREEN_WIDTH; offset += OLED_I2C_CHUNK_SIZE) {
        Wire.beginTransmission(OLED_I2C_ADDR);
        Wire.write((uint8_t)0x40U); // Co=0, D/C#=1 (GDDRAMデータ書き込み)
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
 * @brief GDDRAM ページ切り替え検査（Page 0〜3のラベル表示＋対象ページ反転）
 */
static bool _test_page_switch(void)
{
    uint8_t *p_buf = NULL;
    uint8_t active_page = 0;
    uint8_t target_page = 0;
    bool is_ok = true;

    Serial.println(F("Page Switch (Page Addressing Mode) Test"));

    // フレームバッファ上に各ページ(8px高)のラベルを用意（転送はしない）
    s_display.clearDisplay();
    s_display.setTextSize(1);
    s_display.setTextColor(SSD1306_WHITE);
    for (target_page = 0; target_page < OLED_PAGE_CNT; target_page++) {
        s_display.setCursor(4, (int16_t)(target_page * 8));
        s_display.print(F("PAGE "));
        s_display.print(target_page);
    }

    p_buf = s_display.getBuffer();
    if (p_buf == NULL) {
        return false;
    }

    // Memory Addressing ModeをPage Addressing Mode (0x02) へ変更
    s_display.ssd1306_command(SSD1306_MEMORYMODE);
    s_display.ssd1306_command(0x02U);

    for (active_page = 0; active_page < OLED_PAGE_CNT; active_page++) {
        // 選択中のページだけを白背景・黒文字(反転)で1ページ単位書き込み
        for (target_page = 0; target_page < OLED_PAGE_CNT; target_page++) {
            if (!_write_page_buffer(target_page,
                                   &p_buf[(uint16_t)target_page * SCREEN_WIDTH],
                                    (target_page == active_page))) {
                is_ok = false;
                break;
            }
        }

        if (!is_ok) {
            break;
        }

        if (!_wait_ms_with_detect(WAIT_PAGE_EACH_MS)) {
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

    s_display.clearDisplay();
    s_display.display();

    return is_ok;
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

    s_display.stopscroll();
    s_display.setRotation(0);
    p_buf = s_display.getBuffer();

    if (p_buf == NULL) {
        return false;
    }

    for (phase = 0; phase < 2; phase++) {
        even_pattern = (phase == 0) ? 0x55U : 0xAAU;
        odd_pattern  = (phase == 0) ? 0xAAU : 0x55U;

        for (i = 0; i < SCREEN_BUF_SIZE; i++) {
            p_buf[i] = ((i & 0x01) == 0) ? even_pattern : odd_pattern;
        }
        s_display.display();

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
    s_display.display();

    s_display.invertDisplay(true);
    if (!_wait_ms_with_detect(WAIT_INVERT_MS)) {
        s_display.invertDisplay(false);
        return false;
    }

    s_display.invertDisplay(false);
    return _wait_ms_with_detect(WAIT_INVERT_MS);
}

/**
 * @brief ハードウェア左右・上下ミラー反転検査（Segment Re-map & COM Scan Dir）
 */
static bool _test_hw_flip(void)
{
    uint8_t mode = 0;

    Serial.println(F("Hardware Mirror Flip Test"));

    s_display.clearDisplay();
    s_display.drawTriangle(4, 26, 14, 6, 24, 26, SSD1306_WHITE);
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(32, 8);
    s_display.print(F("HW FLIP"));
    s_display.display();

    for (mode = 0; mode < 4; mode++) {
        // bit0: Segment Re-map (0=Normal 0xA1, 1=Mirror 0xA0)
        // bit1: COM Scan Dir   (0=Normal 0xC8, 1=Flip   0xC0)
        s_display.ssd1306_command(((mode & 0x01U) != 0) ? SSD1306_SEGREMAP : (SSD1306_SEGREMAP | 0x01U));
        s_display.ssd1306_command(((mode & 0x02U) != 0) ? SSD1306_COMSCANINC : SSD1306_COMSCANDEC);

        if (!_wait_ms_with_detect(WAIT_HW_FLIP_MS)) {
            s_display.ssd1306_command(SSD1306_SEGREMAP | 0x01U);
            s_display.ssd1306_command(SSD1306_COMSCANDEC);
            return false;
        }
    }

    // 通常向きへ復帰
    s_display.ssd1306_command(SSD1306_SEGREMAP | 0x01U);
    s_display.ssd1306_command(SSD1306_COMSCANDEC);
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
    s_display.display();

    for (step = 0; step <= 4; step++) {
        offset = (uint8_t)((step * 8U) % SCREEN_HEIGHT);
        s_display.ssd1306_command(SSD1306_SETDISPLAYOFFSET);
        s_display.ssd1306_command(offset);

        if (!_wait_ms_with_detect(WAIT_OFFSET_EACH_MS)) {
            s_display.ssd1306_command(SSD1306_SETDISPLAYOFFSET);
            s_display.ssd1306_command(0x00U);
            return false;
        }
    }

    // オフセット0へ復帰
    s_display.ssd1306_command(SSD1306_SETDISPLAYOFFSET);
    s_display.ssd1306_command(0x00U);
    return true;
}

/**
 * @brief マルチプレクス比（スキャンライン数制限）検査（0xA8: Multiplex Ratio）
 */
static bool _test_multiplex_ratio(void)
{
    const uint8_t p_mux_ratios[] = { 15, 23, 31 };
    uint8_t ratio_cnt = (uint8_t)(sizeof(p_mux_ratios) / sizeof(p_mux_ratios[0]));
    uint8_t i = 0;

    Serial.println(F("Multiplex Ratio Test"));

    s_display.fillScreen(SSD1306_WHITE);
    s_display.display();

    for (i = 0; i < ratio_cnt; i++) {
        s_display.ssd1306_command(SSD1306_SETMULTIPLEX);
        s_display.ssd1306_command(p_mux_ratios[i]);

        if (!_wait_ms_with_detect(WAIT_MUX_EACH_MS)) {
            s_display.ssd1306_command(SSD1306_SETMULTIPLEX);
            s_display.ssd1306_command((uint8_t)(SCREEN_HEIGHT - 1));
            return false;
        }
    }

    // 全ライン駆動へ復帰
    s_display.ssd1306_command(SSD1306_SETMULTIPLEX);
    s_display.ssd1306_command((uint8_t)(SCREEN_HEIGHT - 1));
    return true;
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
    s_display.display();

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

        s_display.display();

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
    s_display.display();

    // 水平右スクロール
    s_display.startscrollright(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        s_display.stopscroll();
        return false;
    }

    // 水平左スクロール
    s_display.startscrollleft(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        s_display.stopscroll();
        return false;
    }

    // 斜め右スクロール（垂直＋水平同時）
    s_display.startscrolldiagright(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        s_display.stopscroll();
        return false;
    }

    // 斜め左スクロール（垂直＋水平同時）
    s_display.startscrolldiagleft(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        s_display.stopscroll();
        return false;
    }

    s_display.stopscroll();
    return true;
}

// -----------------------------------------------------------
// [main()]

/**
 * @brief メイン初期化
 */
void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    Serial.println(F("OLED Batch Inspection Ready."));
}

/**
 * @brief メインループ
 */
void loop(void)
{
    uint8_t i = 0;
    bool is_init_ok = false;
    bool is_ret = false;

    // OLEDがつながってるかポーリング
    if (!s_is_connected) {
        if (_is_oled_ack()) {
            delay(50);
            is_init_ok = s_display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
            if (is_init_ok) {
                s_is_connected = true;
                Serial.println(F("=== New OLED Detected: Start Inspection ==="));
            }
        } else {
            delay(POLL_INTERVAL_MS);
        }
        return;
    }

    // 検査関数のテーブルを回す
    for (i = 0; i < OLED_TEST_FUNC_TBL_CNT; i++)
    {
        Serial.print(F("["));
        Serial.print(i + 1);
        Serial.print(F("/"));
        Serial.print(OLED_TEST_FUNC_TBL_CNT);
        Serial.println(F("] Executing..."));

        // 検査関数実行
        is_ret = s_oled_test_func_tbl[i]();

        if (!is_ret) {
            return;
        }
    }
}
// -----------------------------------------------------------