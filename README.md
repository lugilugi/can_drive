# can_driver

## ECT CAN Driver for ESP32

This component abstracts the ESP32's TWAI peripheral into a robust Publisher/Subscriber model, ensuring telemetry tasks never interfere with critical motor control.

---

## 🌟 Key Features

- **Non-Blocking Architecture:** Uses FreeRTOS queues for background TX/RX. Your main code never hangs waiting for the CAN bus.
- **Topic-Based Routing:** Multiple tasks can subscribe to the same CAN ID (fan-out).
- **Automatic Fault Recovery:** Handles "Bus-Off" states and EMI-induced hardware hangs automatically.
- **Efficient Bit-Packing:** Includes a dictionary for 40-bit energy accumulators (INA780/740) to save bus bandwidth.

---

## 🛠 Installation

Add this component to your ESP-IDF project via the registry:

```bash
idf.py component add lugilugi/can_driver
```

Or add it to your `idf_component.yml`:

```yaml
dependencies:
    lugilugi/can_driver: "^1.0.0"
```

---

## 📖 API Reference

### Header Files

- **can_driver.h:** Main API and macros for CAN driver usage.
- **can_payloads.h:** CAN message IDs and payload packing/unpacking helpers.

---

### Driver Lifecycle

#### `esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud, const uint32_t* filter_ids, size_t id_count);`

- Initializes the CAN driver and internal tasks.
- Parameters:
    - `tx_pin`: GPIO for CAN TX
    - `rx_pin`: GPIO for CAN RX
    - `baud`: CAN bus baud rate (e.g., 500000)
    - `filter_ids`: Optional array of CAN IDs to filter (NULL for all)
    - `id_count`: Number of IDs in filter_ids
- Returns `ESP_OK` on success.

#### `esp_err_t can_driver_deinit(void);`

- Stops the node, deletes tasks, and frees memory.
- Safe to call before re-initializing.

---

### Data Access & Interaction

#### `esp_err_t can_publish(uint32_t id, const void* payload, uint8_t len);`

- Non-blocking publish to the CAN bus.
- Enforces Classic CAN constraints (max 8 bytes, no FD).
- Returns `ESP_OK` if queued, `ESP_FAIL` otherwise.

#### `void can_get_state_internal(size_t offset, void* dest, size_t size);`

- Internal backend for the `CAN_GET_STATE` macro.
- Copies the latest received data for a field from the global vehicle database.

#### `#define CAN_GET_STATE(field, dest_ptr)`

- Macro for type-safe state retrieval.
- Example: `CAN_GET_STATE(pedal, &my_pedal)`

#### `void can_set_rx_hook(can_rx_hook_t hook);`

- Registers a callback for raw frame monitoring (e.g., SD logging).
- Runs in ISR context; must not block.

#### `bool can_is_stale(HBIndex_t index, uint32_t timeout_ms);`

- Checks if a specific node's data is older than the specified timeout.
- Returns true if stale.

---

### Data Structures

#### `VehicleDB_t`

```c
typedef struct {
        PedalPayload      pedal;      // Latest throttle/brake state
        AuxControlPayload aux;        // Latest lights/accessories state
        PowerPayload      pwr_780;    // High-current traction system data
        PowerPayload      pwr_740;    // Low-current auxiliary system data
        EnergyPayload     energy;     // 40-bit energy accumulator data
        uint32_t          last_seen[HB_MAX]; // Tick counts of last received messages
} VehicleDB_t;
```

#### `HBIndex_t`

Heartbeat tracking indices for node health:

```c
typedef enum {
        HB_PEDAL = 0,
        HB_AUX,
        HB_PWR_780,
        HB_PWR_740,
        HB_ENERGY,
        HB_MAX
} HBIndex_t;
```

#### `can_rx_hook_t`

Callback type for frame monitoring:

```c
typedef void (*can_rx_hook_t)(const twai_frame_t* frame);
```

---

### Payload Packing Helpers

See `can_payloads.h` for detailed packing/unpacking functions:

- **PedalPayload**
    - `PedalPayload_set(PedalPayload* p, float throttlePercent, bool brakePressed)`
    - `float PedalPayload_getThrottle(const PedalPayload* p)`
    - `bool PedalPayload_isBrakePressed(const PedalPayload* p)`

- **AuxControlPayload**
    - Bitfields for lights, wipers, horn, etc.

- **PowerPayload**
    - `PowerPayload_setRaw(PowerPayload* p, uint16_t raw_volts, int16_t raw_amps)`
    - `float PowerPayload_getVoltage(const PowerPayload* p)`
    - `float PowerPayload_getCurrent_780(const PowerPayload* p)`
    - `float PowerPayload_getCurrent_740(const PowerPayload* p)`

- **EnergyPayload**
    - `EnergyPayload_setRaw(EnergyPayload* p, uint64_t raw_energy_reg)`
    - `double EnergyPayload_getJoules_780(const EnergyPayload* p)`
    - `double EnergyPayload_getJoules_740(const EnergyPayload* p)`

---

## 🧑‍💻 Example Application

See `examples/basic_test/main/main.c` for a complete working demo with simulated pedal and motor controller tasks. Each function demonstrates a feature of the API.

---

## 📚 License & Contributions

Open to contributions! Please submit issues or pull requests for improvements.