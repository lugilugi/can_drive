#include "can_driver.h"
#include "esp_log.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char* TAG = "CAN_DRV";

// Notification bits for precise state machine transitions
#define NOTIFY_BUS_OFF      (1 << 0)
#define NOTIFY_BUS_ACTIVE   (1 << 1)

// Wrapper to ensure deep copy of data in the Tx queue
typedef struct {
    twai_frame_t frame;
    uint8_t data[8]; 
} CanTxItem_t;

// Routing Map
typedef struct { uint32_t can_id; size_t offset; uint8_t len; HBIndex_t hb; } CanRoute_t;
static const CanRoute_t ROUTE_TABLE[] = {
    { CAN_ID_PEDAL, offsetof(VehicleDB_t, pedal), sizeof(PedalPayload), HB_PEDAL },
    { CAN_ID_AUX_CTRL, offsetof(VehicleDB_t, aux), sizeof(AuxControlPayload), HB_AUX },
    { CAN_ID_PWR_MONITOR_780, offsetof(VehicleDB_t, pwr_780), sizeof(PowerPayload), HB_PWR_780 },
    { CAN_ID_PWR_MONITOR_740, offsetof(VehicleDB_t, pwr_740), sizeof(PowerPayload), HB_PWR_740 },
    { CAN_ID_PWR_ENERGY, offsetof(VehicleDB_t, energy), 5, HB_ENERGY }
};
#define ROUTE_COUNT (sizeof(ROUTE_TABLE) / sizeof(CanRoute_t))

static VehicleDB_t v_db = {0};
static portMUX_TYPE db_mux = portMUX_INITIALIZER_UNLOCKED;
static can_rx_hook_t app_hook = NULL;

static twai_node_handle_t node_hdl = NULL;
static QueueHandle_t tx_queue = NULL;
static TaskHandle_t tx_task_handle = NULL;
static TaskHandle_t recovery_task_handle = NULL;
static bool is_running = false;

esp_err_t can_driver_deinit(void) {
    if (!is_running && !node_hdl && !tx_queue) return ESP_OK;

    if (node_hdl) {
        esp_err_t ret = twai_node_disable(node_hdl); //
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Node disable failed: %s", esp_err_to_name(ret));
        }
    }

    if (tx_task_handle) vTaskDelete(tx_task_handle);
    if (recovery_task_handle) vTaskDelete(recovery_task_handle);
    if (node_hdl) twai_node_delete(node_hdl); //
    if (tx_queue) vQueueDelete(tx_queue);

    tx_task_handle = NULL;
    recovery_task_handle = NULL;
    node_hdl = NULL;
    tx_queue = NULL;
    is_running = false;
    return ESP_OK;
}

// ===================================================================================
// INTERNAL HELPERS
// ===================================================================================

static void configure_hardware_filter(const uint32_t* ids, size_t count) {
    if (count == 0) return;
    uint32_t c_ones = ids[0], c_zeros = ~ids[0] & 0x7FF;
    for (size_t i = 1; i < count; i++) {
        c_ones &= ids[i];
        c_zeros &= (~ids[i] & 0x7FF);
    }
    // ESP-IDF v5.5.3 Logic: 1 = match exactly, 0 = ignore
    uint32_t match_mask = (c_ones | c_zeros) & 0x7FF;
    twai_mask_filter_config_t cfg = { .id = c_ones, .mask = match_mask, .is_ext = false };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_hdl, 0, &cfg)); // Must be called while disabled
}

// ===================================================================================
// ISR CALLBACKS
// ===================================================================================

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *ctx) {
    twai_frame_t rx;
    uint8_t buf[8];
    rx.buffer = buf;
    rx.buffer_len = 8;

    if (twai_node_receive_from_isr(handle, &rx) == ESP_OK) {
        if (rx.header.fdf) return false; // Incompatible FD frames are treated as errors
        if (app_hook) app_hook(&rx);

        for (int i = 0; i < ROUTE_COUNT; i++) {
            if (rx.header.id == ROUTE_TABLE[i].can_id) {
                taskENTER_CRITICAL_ISR(&db_mux);
                memcpy((uint8_t*)&v_db + ROUTE_TABLE[i].offset, rx.buffer, ROUTE_TABLE[i].len);
                v_db.last_seen[ROUTE_TABLE[i].hb] = xTaskGetTickCountFromISR();
                taskEXIT_CRITICAL_ISR(&db_mux);
                break;
            }
        }
    }
    return false;
}

static bool twai_state_cb(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *ctx) {
    BaseType_t woken = pdFALSE;
    if (recovery_task_handle == NULL) return false;

    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        xTaskNotifyFromISR(recovery_task_handle, NOTIFY_BUS_OFF, eSetBits, &woken);
    } else if (edata->new_sta == TWAI_ERROR_ACTIVE) {
        xTaskNotifyFromISR(recovery_task_handle, NOTIFY_BUS_ACTIVE, eSetBits, &woken);
    }
    return woken == pdTRUE;
}

// ===================================================================================
// TASKS
// ===================================================================================

static void recovery_task(void *arg) {
    uint32_t delay = 100, bits;
    while (1) {
        // Wait specifically for BUS_OFF bit and clear on exit
        xTaskNotifyWait(0, NOTIFY_BUS_OFF | NOTIFY_BUS_ACTIVE, &bits, portMAX_DELAY);
        if (!(bits & NOTIFY_BUS_OFF)) continue; // Spurious wakeup, ignore

        ESP_LOGE(TAG, "Bus-Off! Suspending TX and waiting %lu ms", delay);
        vTaskSuspend(tx_task_handle);
        vTaskDelay(pdMS_TO_TICKS(delay));
        xQueueReset(tx_queue);
        
        // Non-blocking recovery initiation
        if (twai_node_recover(node_hdl) == ESP_OK) {
            // Wait for BUS_ACTIVE bit to ensure hardware is back online
            TickType_t timeout = pdMS_TO_TICKS(5000);
            if (xTaskNotifyWait(0, NOTIFY_BUS_ACTIVE, &bits, timeout) == pdFALSE || 
                !(bits & NOTIFY_BUS_ACTIVE)) {
                ESP_LOGE(TAG, "Recovery timed out. Bus may be physically damaged.");
                delay = (delay * 2 > 5000) ? 5000 : delay * 2;
                xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_BUS_OFF, eSetBits);
                continue;
            }
            ESP_LOGI(TAG, "Bus Recovered. Resuming TX.");
            vTaskResume(tx_task_handle);
            delay = 100; // Reset backoff on success
        } else {
            delay = (delay * 2 > 5000) ? 5000 : delay * 2;
            // Retry recovery on next loop by setting BUS_OFF bit
            xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_BUS_OFF, eSetBits); 
        }
    }
}

static void tx_task(void* arg) {
    CanTxItem_t item;
    while(xQueueReceive(tx_queue, &item, portMAX_DELAY)) {
        item.frame.buffer = item.data; 
        twai_node_transmit(node_hdl, &item.frame, pdMS_TO_TICKS(10));
    }
}

// ===================================================================================
// PUBLIC API
// ===================================================================================

esp_err_t can_driver_init(gpio_num_t tx, gpio_num_t rx, uint32_t baud, const uint32_t* f_ids, size_t f_count) {
    if (is_running) return ESP_OK;

    tx_queue = xQueueCreate(20, sizeof(CanTxItem_t));
    if (!tx_queue) return ESP_ERR_NO_MEM;

    twai_onchip_node_config_t n_cfg = { .io_cfg.tx=tx, .io_cfg.rx=rx, .bit_timing.bitrate=baud, .tx_queue_depth=20 };
    if (twai_new_node_onchip(&n_cfg, &node_hdl) != ESP_OK) return ESP_FAIL;

    twai_event_callbacks_t cbs = { .on_rx_done = twai_rx_cb, .on_state_change = twai_state_cb };
    twai_node_register_event_callbacks(node_hdl, &cbs, NULL); // Registered before enable
    configure_hardware_filter(f_ids, f_count);

    // FIX: Handles are assigned BEFORE enable to ensure callbacks don't crash on initial state change
    if (xTaskCreate(tx_task, "CAN_Tx", 4096, NULL, 9, &tx_task_handle) != pdPASS ||
        xTaskCreate(recovery_task, "CAN_Rec", 2048, NULL, 11, &recovery_task_handle) != pdPASS) return ESP_FAIL;

    if (twai_node_enable(node_hdl) != ESP_OK) return ESP_FAIL;

    is_running = true;
    return ESP_OK;
}

void can_set_rx_hook(can_rx_hook_t hook) { app_hook = hook; }

void can_get_state_internal(size_t off, void* dst, size_t sz) {
    taskENTER_CRITICAL(&db_mux);
    memcpy(dst, (uint8_t*)&v_db + off, sz);
    taskEXIT_CRITICAL(&db_mux);
}

bool can_is_stale(HBIndex_t idx, uint32_t ms) {
    return (xTaskGetTickCount() - v_db.last_seen[idx]) > pdMS_TO_TICKS(ms);
}

esp_err_t can_publish(uint32_t id, const void* pld, uint8_t len) {
    if (len > 8 || !tx_queue) return ESP_ERR_INVALID_ARG;
    CanTxItem_t item = { .frame = { .header.id = id, .header.fdf = false, .buffer_len = len } };
    memcpy(item.data, pld, len); 
    return xQueueSend(tx_queue, &item, 0) == pdTRUE ? ESP_OK : ESP_FAIL;
}