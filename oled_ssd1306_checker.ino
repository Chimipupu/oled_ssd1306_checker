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
#define SCREEN_WIDTH        (128)
#define SCREEN_HEIGHT       (32)
#define SCREEN_BUF_SIZE     ((SCREEN_WIDTH * SCREEN_HEIGHT) / 8)
#define OLED_RESET          (-1)
#define OLED_I2C_ADDR       (0x3C)
#define WAIT_ALL_ON_MS      (1500)
#define WAIT_CHECKER_MS     (1000)
#define WAIT_TEXT_MS        (1500)
#define WAIT_ROT_EACH_MS    (600)
#define WAIT_SCROLL_EACH_MS (1000)
#define POLL_INTERVAL_MS    (50)

static Adafruit_SSD1306 s_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool s_is_connected = false;

typedef bool (*oled_test_func_t)(void);
static bool _is_oled_ack(void);
static bool _wait_ms_with_detect(uint32_t wait_ms);
static bool _test_all_pixels(void);
static bool _test_checkerboard(void);
static bool _test_full_text(void);
static bool _test_rotation(void);
static bool _test_scroll(void);

// 検査関数ポインタテーブル
static const oled_test_func_t s_oled_test_func_tbl[] = {
    // 全画面のピクセル点灯検査（ドット抜け・輝度ムラ確認）
    _test_all_pixels,

    // 市松模様テスト（隣接ピクセル干渉・全画素ON/OFF網羅確認）
    _test_checkerboard,

    // 標準フォント(6x8px)を全画面128x32分表示（21文字 x 4行 = 84文字）
    _test_full_text,

    // 画面の回転テスト（0° -> 90° -> 180° -> 270°）
    _test_rotation,

    // 文字スクロールテスト
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
 * @brief 市松模様テスト（隣接ピクセル干渉・全画素ON/OFF網羅確認）
 */
static bool _test_checkerboard(void)
{
    uint8_t *p_buf = NULL;
    uint16_t i = 0;
    uint8_t phase = 0;
    uint8_t even_pattern = 0;
    uint8_t odd_pattern = 0;

    s_display.stopscroll();
    s_display.setRotation(0);
    p_buf = s_display.getBuffer();

    if (p_buf == NULL) {
        return false;
    }

    for (phase = 0; phase < 2; phase++) {
        even_pattern = (phase == 0) ? 0x55U : 0xAA;
        odd_pattern  = (phase == 0) ? 0xAAU : 0x55;

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
 * @brief 標準フォント(6x8px)を全画面128x32分表示（21文字 x 4行 = 84文字）
 */
static bool _test_full_text(void)
{
    const char *p_chars = "0123456789ABCDEFGHIJKLMNPQRSTUVWXYZ!@#$%&*+-=/?";
    uint8_t char_len = 47;
    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t idx = 0;

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
 * @brief 文字スクロールテスト
 */
static bool _test_scroll(void)
{
    s_display.clearDisplay();
    s_display.setTextSize(2);
    s_display.setTextColor(SSD1306_WHITE);
    s_display.setCursor(10, 8);
    s_display.print(F("SCROLL OK"));
    s_display.display();

    s_display.startscrollright(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
        return false;
    }

    s_display.startscrollleft(0x00, 0x03);
    if (!_wait_ms_with_detect(WAIT_SCROLL_EACH_MS)) {
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
    bool ret = false;

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
        ret = s_oled_test_func_tbl[i]();

        if (ret != true) {
            return;
        }
    }
}
// -----------------------------------------------------------