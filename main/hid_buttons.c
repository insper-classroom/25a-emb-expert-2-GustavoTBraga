#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "btstack.h"

#define LED_PIN      11
#define BTN_A_PIN     9
#define BTN_W_PIN    17
#define BTN_S_PIN    14
#define BTN_D_PIN    12

static const uint8_t BTN_PINS[4] = { BTN_A_PIN, BTN_W_PIN, BTN_S_PIN, BTN_D_PIN };
static const uint8_t KEY_USAGE[4] = { 0x04, 0x1A, 0x16, 0x07 };   // A W S D

#define REPORT_ID 0x01
const uint8_t HID_DESC[] = {
    0x05,0x01,0x09,0x06,0xA1,0x01,0x85,REPORT_ID,
    0x75,0x01,0x95,0x08,0x05,0x07,0x19,0xE0,0x29,0xE7,
    0x15,0x00,0x25,0x01,0x81,0x02,
    0x75,0x08,0x95,0x01,0x81,0x01,
    0x75,0x08,0x95,0x06,0x15,0x00,0x25,0xFF,
    0x05,0x07,0x19,0x00,0x29,0xFF,0x81,0x00,
    0xC0
};

static btstack_timer_source_t led_tmr, btn_tmr;
static bool    led_state            = false;
static uint8_t last_btn_state       = 0;
static bool    pending_report       = false;
static uint8_t pending_key          = 0;
static uint16_t hid_cid             = 0;

enum { APP_BOOT, APP_IDLE, APP_CONN } app_state = APP_BOOT;

static inline void led_set(bool on){ gpio_put(LED_PIN, on); }

static void led_tick(btstack_timer_source_t *ts){
    watchdog_update();
    led_state = !led_state;
    led_set(led_state);
    btstack_run_loop_set_timer(ts, 500);
    btstack_run_loop_add_timer(ts);
}
static void led_blink_start(void){
    btstack_run_loop_remove_timer(&led_tmr);
    led_tmr.process = led_tick;
    btstack_run_loop_set_timer(&led_tmr, 0);
    btstack_run_loop_add_timer(&led_tmr);
}
static void led_on(void){
    btstack_run_loop_remove_timer(&led_tmr);
    led_set(true);
}

static inline bool btn_pressed(int i){ return gpio_get(BTN_PINS[i]) == 0; }

static void btn_tick(btstack_timer_source_t *ts){
    watchdog_update();

    uint8_t st = 0;
    for(int i=0;i<4;++i) if(btn_pressed(i)) st |= 1u<<i;
    uint8_t changed = st & ~last_btn_state;

    if(changed && app_state==APP_CONN && !pending_report){
        for(int i=0;i<4;++i) if(changed & (1u<<i)){
            pending_key    = KEY_USAGE[i];
            pending_report = true;
            hid_device_request_can_send_now_event(hid_cid);
            break;
        }
    }
    last_btn_state = st;
    btstack_run_loop_set_timer(ts, 10);
    btstack_run_loop_add_timer(ts);
}

static void send_report(int mod, int key){
    uint8_t msg[] = { 0xA1, REPORT_ID, mod, 0, key, 0,0,0,0,0 };
    hid_device_send_interrupt_message(hid_cid, msg, sizeof(msg));
}

static void pk_handler(uint8_t type, uint16_t ch, uint8_t *pkt, uint16_t) {
    (void)ch;
    if (type!=HCI_EVENT_PACKET) return;
    switch(hci_event_packet_get_type(pkt)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(pkt)==HCI_STATE_WORKING) {
            app_state = APP_IDLE;
            led_blink_start();
        }
        break;

    case HCI_EVENT_HID_META:
        switch(hci_event_hid_meta_get_subevent_code(pkt)) {
        case HID_SUBEVENT_CONNECTION_OPENED:
            if (hid_subevent_connection_opened_get_status(pkt)) {
                app_state=APP_IDLE; hid_cid=0; led_blink_start();
            } else {
                app_state=APP_CONN;
                hid_cid = hid_subevent_connection_opened_get_hid_cid(pkt);
                led_on();
            }
            break;

        case HID_SUBEVENT_CONNECTION_CLOSED:
            app_state=APP_IDLE; hid_cid=0; led_blink_start();
            break;

        case HID_SUBEVENT_CAN_SEND_NOW:
            if (pending_report && pending_key) {
                send_report(0, pending_key);
                pending_key = 0; pending_report=false;
                hid_device_request_can_send_now_event(hid_cid);
            } else {
                send_report(0,0);
            }
            break;
        default: break;
        }
        break;
    default: break;
    }
}

#define WDT_TIMEOUT_MS 3000

int btstack_main(int, const char**) {
    // GPIO
    for (int i=0;i<4;++i) {
        gpio_init(BTN_PINS[i]);
        gpio_set_dir(BTN_PINS[i], GPIO_IN);
        gpio_pull_up(BTN_PINS[i]);
    }
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led_set(false);

    // Timers
    btn_tmr.process = btn_tick;
    btstack_run_loop_set_timer(&btn_tmr, 10);
    btstack_run_loop_add_timer(&btn_tmr);

    led_blink_start();

    watchdog_enable(WDT_TIMEOUT_MS, 0);
    watchdog_update();

    // BTstack
    gap_discoverable_control(1);
    gap_set_class_of_device(0x2540);
    gap_set_local_name("PicoW-HID-AWSD");
    gap_set_default_link_policy_settings(
        LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    l2cap_init(); sdp_init();

    static uint8_t sdp_buf[300];
    static hid_sdp_record_t rec = {
        0x2540,33,1,1,1,1,0,1600,3200,3200,
        HID_DESC,sizeof(HID_DESC),"PicoW-HID-AWSD"
    };
    hid_create_sdp_record(sdp_buf, sdp_create_service_record_handle(), &rec);
    sdp_register_service(sdp_buf);

    hid_device_init(0,sizeof(HID_DESC),HID_DESC);
    hid_device_register_packet_handler(pk_handler);

    hci_power_control(HCI_POWER_ON);
    return 0;
}
