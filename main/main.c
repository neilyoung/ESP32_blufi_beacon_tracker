#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bt.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

extern void bt_task(void *ignore);

static const char tag[] = "BLE";

static uint8_t ibeacon_prefix[] = {
    0x02, 0x01, 0x00, 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15};


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    esp_ble_gap_cb_param_t *p = (esp_ble_gap_cb_param_t *)param;

    if (p->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
        // Check for iBeacon adv prefix. Ignore 3rd byte, this varies from beacon type to beacon type
        p->scan_rst.ble_adv[2] = 0x00;
        for (int i = 0; i < sizeof(ibeacon_prefix); i++) {
            if (p->scan_rst.ble_adv[i] != ibeacon_prefix[i]) {
                return;
            }
        }

        ESP_LOGI(tag, "BDA %02X:%02X:%02X:%02X:%02X:%02X, RSSI %d, UUID %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X, MAJ %d, MIN %d, TXPWR %d",
                 p->scan_rst.bda[0],
                 p->scan_rst.bda[1],
                 p->scan_rst.bda[2],
                 p->scan_rst.bda[3],
                 p->scan_rst.bda[4],
                 p->scan_rst.bda[5],

                 p->scan_rst.rssi,

                 p->scan_rst.ble_adv[9],
                 p->scan_rst.ble_adv[10],
                 p->scan_rst.ble_adv[11],
                 p->scan_rst.ble_adv[12],

                 p->scan_rst.ble_adv[13],
                 p->scan_rst.ble_adv[14],

                 p->scan_rst.ble_adv[15],
                 p->scan_rst.ble_adv[16],

                 p->scan_rst.ble_adv[17],
                 p->scan_rst.ble_adv[18],

                 p->scan_rst.ble_adv[19],
                 p->scan_rst.ble_adv[20],
                 p->scan_rst.ble_adv[21],
                 p->scan_rst.ble_adv[22],
                 p->scan_rst.ble_adv[23],
                 p->scan_rst.ble_adv[24],

                 (p->scan_rst.ble_adv[25] << 8) | p->scan_rst.ble_adv[26],

                 (p->scan_rst.ble_adv[27] << 8) | p->scan_rst.ble_adv[28],

                 p->scan_rst.ble_adv[29] - 256

                 );
    }
}

void bt_task(void *ignore) {
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(tag, "%s enable bt controller failed\n", __func__);
        goto end;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "%s init bluedroid failed\n", __func__);
        goto end;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(tag, "%s enable bluedroid failed\n", __func__);
        goto end;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "esp_ble_gap_register_callback: rc=%d", ret);
        goto end;
    }

    static esp_ble_scan_params_t ble_scan_params = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window =  0x30};

    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "esp_ble_gap_set_scan_params: rc=%d", ret);
        goto end;
    }

    ret = esp_ble_gap_start_scanning(60);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "esp_ble_gap_start_scanning: rc=%d", ret);
        goto end;
    }
    ESP_LOGI(tag, "Wait for scans...");
end:
    vTaskDelete(NULL);
}

void app_main(void) {
	nvs_flash_init();
    xTaskCreatePinnedToCore(&bt_task, "btTask", 2048, NULL, 5, NULL, 0);
}