#pragma once

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "can_payloads.h"

/**
 * @brief Heartbeat Tracking Indices
 * These map to the last_seen array in the database to track node health.
 */
typedef enum { 
    HB_PEDAL = 0, 
    HB_AUX, 
    HB_PWR_780, 
    HB_PWR_740, 
    HB_ENERGY, 
    HB_MAX 
} HBIndex_t;

/**
 * @brief Shared Vehicle State Database
 * This struct represents the most recent state of the vehicle.
 * Instead of tasks waiting on queues, they "pull" data from here.
 */
typedef struct {
    PedalPayload      pedal;      // Latest throttle/brake state
    AuxControlPayload aux;        // Latest lights/accessories state
    PowerPayload      pwr_780;    // High-current traction system data
    PowerPayload      pwr_740;    // Low-current auxiliary system data
    EnergyPayload     energy;     // 40-bit energy accumulator data
    uint32_t          last_seen[HB_MAX]; // Tick counts of last received messages
} VehicleDB_t;

/**
 * @brief Function pointer type for the SD Logger or other "Listeners"
 * NOTE: This hook is called within an ISR context.
 */
typedef void (*can_rx_hook_t)(const twai_frame_t* frame);

// --- Driver Lifecycle ---

/**
 * @brief Initialize the TWAI driver and internal tasks.
 * Includes transactional cleanup; if any part of the init fails, 
 * it calls deinit internally to prevent resource leaks.
 */
esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud, const uint32_t* filter_ids, size_t id_count);

/**
 * @brief Safely stop the node, delete tasks, and free memory.
 */
esp_err_t can_driver_deinit(void);

// --- Data Access & Interaction ---

/**
 * @brief Registers a callback for raw frame monitoring (e.g., SD logging).
 */
void can_set_rx_hook(can_rx_hook_t hook);

/**
 * @brief Checks if a specific node's data is older than the specified timeout.
 */
bool can_is_stale(HBIndex_t index, uint32_t timeout_ms);

/**
 * @brief Non-blocking publish to the CAN bus.
 * Enforces Classic CAN constraints (max 8 bytes, no FD).
 */
esp_err_t can_publish(uint32_t id, const void* payload, uint8_t len);

/**
 * @brief Internal backend for the CAN_GET_STATE macro.
 */
void can_get_state_internal(size_t offset, void* dest, size_t size);

/**
 * @brief Macro for type-safe state retrieval.
 * Uses offsetof to calculate the memory address within the global struct.
 */
#define CAN_GET_STATE(field, dest_ptr) \
    can_get_state_internal(offsetof(VehicleDB_t, field), dest_ptr, sizeof(((VehicleDB_t*)0)->field))
    