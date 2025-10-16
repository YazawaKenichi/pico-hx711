// Raspberry Pi Pico + micro-ROS で HX711 を読み取り、[g] を Publish するサンプル
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include "main.h"

// ====== Pin設定（必要に応じて配線に合わせて変更してください）======
#define HX711_DOUT_PIN 2   // HX711 DOUT -> Pico GPIO2
#define HX711_SCK_PIN  3   // HX711 SCK  -> Pico GPIO3
#define LED_PIN        25  // On-board LED

// ====== micro-ROS パラメータ ======
#define PUB_TOPIC "hx711/raw"
#define TIMER_PERIOD_MS 1  // Publish周期

// ====== HX711 スケール設定（ロードセル仕様に合わせて変更）======
// 例: 1kg ボード／ロードセル
#define OUT_VOL   0.001f    // 定格出力[V/V]での出力例をそのまま使用（参考: Arduino版）
#define LOAD      1000.0f   // 定格容量[g]

// 内部計算用（Arduino版ロジックを踏襲）
#define HX711_R1      20000.0f
#define HX711_R2       8200.0f
#define HX711_VBG         1.25f
#define HX711_AVDD   3.3f   // = HX711_VBG*((R1+R2)/R2) の近似値
#define HX711_ADC_1BIT  (HX711_AVDD/16777216.0f) // 2^24
#define HX711_PGA       128.0f
#define HX711_SCALE    (OUT_VOL * HX711_AVDD / LOAD * HX711_PGA)

// ====== グローバル ======
rcl_publisher_t publisher;
rclc_executor_t executor;
std_msgs__msg__Int32 msg;
float tare_offset_g = 0.0f;

// ====== HX711 関数群（Arduino版をPico SDKに移植）======
static inline void hx711_init(void) {
    gpio_init(HX711_SCK_PIN);
    gpio_set_dir(HX711_SCK_PIN, GPIO_OUT);
    gpio_put(HX711_SCK_PIN, 0);

    gpio_init(HX711_DOUT_PIN);
    gpio_set_dir(HX711_DOUT_PIN, GPIO_IN);
}

static inline void hx711_reset(void) {
    gpio_put(HX711_SCK_PIN, 1);
    sleep_us(100);
    gpio_put(HX711_SCK_PIN, 0);
    sleep_us(100);
}

static inline bool hx711_is_ready(void) {
    // DOUT が Low でデータ準備完了
    return gpio_get(HX711_DOUT_PIN) == 0;
}

static inline uint32_t hx711_read_raw24(void) {
    // DOUT が Low になるまで待機（ブロッキング）
    while (!hx711_is_ready()) {
        tight_loop_contents();
    }
    sleep_us(5);

    uint32_t data = 0;
    for (int i = 0; i < 24; i++) {
        gpio_put(HX711_SCK_PIN, 1);
        sleep_us(2);
        data = (data << 1) | (gpio_get(HX711_DOUT_PIN) & 0x1);
        gpio_put(HX711_SCK_PIN, 0);
        sleep_us(2);
    }
    // チャネル/ゲイン設定（128x, Aチャンネル）: 追加クロック1パルス
    gpio_put(HX711_SCK_PIN, 1);
    sleep_us(5);
    gpio_put(HX711_SCK_PIN, 0);
    sleep_us(5);

    if (data & 0x800000u)
    {
        data |= 0xFF000000u;
    }
    // Arduino版に合わせて符号処理（XOR 0x800000）
    return (int32_t) data;
}

static inline int32_t hx711_read_average_raw(int samples) {
    int64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += (int32_t)hx711_read_raw24();
    }
    return (int32_t)(sum / samples);
}

static inline float hx711_raw_to_volt(int32_t raw24_avg) {
    // 24bit → [V] へ
    return (float)raw24_avg * HX711_ADC_1BIT;
}

static inline float hx711_get_grams(int samples) {
    int32_t raw_avg = hx711_read_average_raw(samples);
    float v = hx711_raw_to_volt(raw_avg);
    // Arduino 版の換算式を踏襲
    float g = v / HX711_SCALE;
    return g;
}

// ====== タイマーコールバック ======
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    // 複数回平均（ノイズ低減）
    // float grams = hx711_get_grams(5) - tare_offset_g;
    // 生のデータを取得
    uint32_t raw = hx711_read_raw24();

    msg.data = raw;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    (void)ret;
}

// ====== メイン ======
int main(void) {
    // UART トランスポート（ros.c と同等設定） ← 参照: :contentReference[oaicite:2]{index=2}
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    stdio_init_all();

    // GPIO 初期化
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // HX711 初期化
    hx711_init();
    hx711_reset();

    // micro-ROS 基本構成
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t ret;

    // エージェントPing待ち（最大約2分） ← 参照: :contentReference[oaicite:3]{index=3}
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "hx711_node", "", &support);

    // Publisher (Float32)
    rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        PUB_TOPIC);

    // タイマー作成
    rcl_timer_t timer;
    rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(TIMER_PERIOD_MS),
        timer_callback);

    // エグゼキュータ
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // ゼロ点（風袋）取得：30回平均（Arduino版の習慣に合わせる） ← 参照: :contentReference[oaicite:4]{index=4}
    tare_offset_g = hx711_get_grams(30);

    // ループ
    bool led = false;
    while (true) {
        led = !led;
        gpio_put(LED_PIN, led);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}


