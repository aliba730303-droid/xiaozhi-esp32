#include "water_heater_controller.h"

#include <cstring>

#include "config.h"
#include "mcp_server.h"

#include <driver/i2c_slave.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace {

constexpr char kTag[] = "HajAliI2C";
constexpr uint8_t kSlaveAddress = 0x13;
constexpr uint8_t kCurrentTemperatureIndex = 0;
constexpr uint8_t kTargetTemperatureIndex = HajAliI2C::kTxTargetTemperature;
constexpr uint8_t kDefaultTargetTemperature = 50;
constexpr uint8_t kMinimumTemperature = 10;
constexpr uint8_t kMaximumTemperature = 90;

enum class EventType : uint8_t { Receive, Request };

struct I2CEvent {
    EventType type;
    uint32_t length;
    uint8_t data[HajAliI2C::kDataSize];
};

i2c_slave_dev_handle_t g_slave = nullptr;
QueueHandle_t g_eventQueue = nullptr;
TaskHandle_t g_workerTask = nullptr;
portMUX_TYPE g_dataMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t g_rxData[HajAliI2C::kDataSize] = {};
uint8_t g_txData[HajAliI2C::kDataSize] = {};
bool g_receivedFrame = false;
bool g_ready = false;

// ESP-IDF invokes these callbacks in ISR context. They only copy data and queue work.
bool OnReceive(i2c_slave_dev_handle_t,
               const i2c_slave_rx_done_event_data_t* event,
               void*) {
    if (event == nullptr || event->buffer == nullptr || g_eventQueue == nullptr) {
        return false;
    }

    I2CEvent queued{};
    queued.type = EventType::Receive;
    queued.length = event->length;
    const size_t copyLength = event->length < HajAliI2C::kDataSize
                                  ? event->length
                                  : HajAliI2C::kDataSize;
    std::memcpy(queued.data, event->buffer, copyLength);

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    const BaseType_t sent = xQueueSendFromISR(g_eventQueue, &queued, &higherPriorityTaskWoken);
    (void)sent;
    return higherPriorityTaskWoken == pdTRUE;
}

bool OnRequest(i2c_slave_dev_handle_t,
               const i2c_slave_request_event_data_t*,
               void*) {
    if (g_eventQueue == nullptr) {
        return false;
    }

    I2CEvent queued{};
    queued.type = EventType::Request;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    const BaseType_t sent = xQueueSendFromISR(g_eventQueue, &queued, &higherPriorityTaskWoken);
    (void)sent;
    return higherPriorityTaskWoken == pdTRUE;
}

void I2CWorker(void*) {
    I2CEvent event{};
    while (true) {
        if (xQueueReceive(g_eventQueue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == EventType::Receive) {
            if (event.length != HajAliI2C::kDataSize) {
                ESP_LOGW(kTag, "Ignoring I2C frame with invalid length: %lu",
                         static_cast<unsigned long>(event.length));
                continue;
            }

            taskENTER_CRITICAL(&g_dataMux);
            std::memcpy(g_rxData, event.data, HajAliI2C::kDataSize);
            g_receivedFrame = true;
            taskEXIT_CRITICAL(&g_dataMux);
            ESP_LOGD(kTag, "Received 25-byte master frame");
        } else {
            uint8_t response[HajAliI2C::kDataSize];
            taskENTER_CRITICAL(&g_dataMux);
            std::memcpy(response, g_txData, sizeof(response));
            taskEXIT_CRITICAL(&g_dataMux);

            uint32_t written = 0;
            const esp_err_t err =
                i2c_slave_write(g_slave, response, sizeof(response), &written, 100);
            if (err != ESP_OK || written != sizeof(response)) {
                ESP_LOGW(kTag, "I2C response write incomplete: err=%s, bytes=%lu",
                         esp_err_to_name(err), static_cast<unsigned long>(written));
            } else {
                ESP_LOGI(kTag, "TX frame sent: 25 bytes, target=%u, marker=%u",
                         response[HajAliI2C::kTxTargetTemperature],
                         response[HajAliI2C::kTxFrameMarker]);
            }
        }
    }
}

}  // namespace

namespace HajAliI2C {

bool Initialize() {
    // Set an explicit tag level: some XiaoZhi builds filter newly introduced tags.
    esp_log_level_set(kTag, ESP_LOG_INFO);
    ESP_LOGE(kTag, "INIT_STEP=ENTER");
    if (g_ready) {
        return true;
    }

    ESP_LOGE(kTag, "INIT_STEP=CREATE_QUEUE");
    g_eventQueue = xQueueCreate(8, sizeof(I2CEvent));
    if (g_eventQueue == nullptr) {
        ESP_LOGE(kTag, "Failed to create I2C event queue");
        return false;
    }

    ESP_LOGE(kTag, "INIT_STEP=QUEUE_OK");
    g_txData[kTargetTemperatureIndex] = kDefaultTargetTemperature;
    // Fixed frame marker requested by the master-side protocol.
    g_txData[HajAliI2C::kTxFrameMarker] = HajAliI2C::kTxFrameMarkerValue;

    const i2c_slave_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = WATER_HEATER_I2C_SDA_GPIO,
        .scl_io_num = WATER_HEATER_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .send_buf_depth = 64,
        .receive_buf_depth = 64,
        .slave_addr = kSlaveAddress,
        .addr_bit_len = I2C_ADDR_BIT_LEN_7,
        .intr_priority = 0,
        .flags = {
            .allow_pd = 0,
            .enable_internal_pullup = 1,
        },
    };

    ESP_LOGE(kTag, "INIT_STEP=NEW_SLAVE port=%d SDA=%d SCL=%d addr=0x%02X", (int)config.i2c_port, (int)config.sda_io_num, (int)config.scl_io_num, kSlaveAddress);
    esp_err_t err = i2c_new_slave_device(&config, &g_slave);
    ESP_LOGE(kTag, "INIT_STEP=NEW_SLAVE_RETURN err=%s", esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C slave initialization failed: %s", esp_err_to_name(err));
        vQueueDelete(g_eventQueue);
        g_eventQueue = nullptr;
        return false;
    }

    ESP_LOGE(kTag, "INIT_STEP=REGISTER_CALLBACKS");
    const i2c_slave_event_callbacks_t callbacks = {
        .on_request = OnRequest,
        .on_receive = OnReceive,
    };
    err = i2c_slave_register_event_callbacks(g_slave, &callbacks, nullptr);
    ESP_LOGE(kTag, "INIT_STEP=CALLBACKS_RETURN err=%s", esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C callback registration failed: %s", esp_err_to_name(err));
        i2c_del_slave_device(g_slave);
        g_slave = nullptr;
        vQueueDelete(g_eventQueue);
        g_eventQueue = nullptr;
        return false;
    }

    ESP_LOGE(kTag, "INIT_STEP=CREATE_WORKER");
    if (xTaskCreate(I2CWorker, "hajali_i2c", 4096, nullptr, 6, &g_workerTask) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create I2C worker task");
        i2c_del_slave_device(g_slave);
        g_slave = nullptr;
        vQueueDelete(g_eventQueue);
        g_eventQueue = nullptr;
        return false;
    }

    ESP_LOGE(kTag, "INIT_STEP=WORKER_CREATED");
    g_ready = true;
    ESP_LOGE(kTag, "I2C slave ready: address=0x%02X SDA=GPIO%d SCL=GPIO%d",
             kSlaveAddress, static_cast<int>(WATER_HEATER_I2C_SDA_GPIO),
             static_cast<int>(WATER_HEATER_I2C_SCL_GPIO));
    return true;
}

bool IsReady() {
    return g_ready;
}

bool SetTxValue(uint8_t index, uint8_t value, uint8_t minimum, uint8_t maximum) {
    if (index >= kDataSize || index == kTxFrameMarker || value < minimum || value > maximum) {
        return false;
    }
    taskENTER_CRITICAL(&g_dataMux);
    g_txData[index] = value;
    taskEXIT_CRITICAL(&g_dataMux);
    ESP_LOGI(kTag, "TX byte[%u] updated to %u", index, value);
    return true;
}

uint8_t GetTxValue(uint8_t index) {
    if (index >= kDataSize) return 0;
    uint8_t value;
    taskENTER_CRITICAL(&g_dataMux);
    value = g_txData[index];
    taskEXIT_CRITICAL(&g_dataMux);
    return value;
}

bool SetTargetTemperature(uint8_t temperature) {
    if (temperature < kMinimumTemperature || temperature > kMaximumTemperature) {
        return false;
    }

    taskENTER_CRITICAL(&g_dataMux);
    g_txData[kTargetTemperatureIndex] = temperature;
    taskEXIT_CRITICAL(&g_dataMux);
    return true;
}

uint8_t GetCurrentTemperature() {
    uint8_t value;
    taskENTER_CRITICAL(&g_dataMux);
    value = g_rxData[kCurrentTemperatureIndex];
    taskEXIT_CRITICAL(&g_dataMux);
    return value;
}

uint8_t GetTargetTemperature() {
    uint8_t value;
    taskENTER_CRITICAL(&g_dataMux);
    value = g_txData[kTargetTemperatureIndex];
    taskEXIT_CRITICAL(&g_dataMux);
    return value;
}

bool HasReceivedMasterFrame() {
    bool value;
    taskENTER_CRITICAL(&g_dataMux);
    value = g_receivedFrame;
    taskEXIT_CRITICAL(&g_dataMux);
    return value;
}

}  // namespace HajAliI2C

void InitializeWaterHeaterTools() {
    static bool toolsRegistered = false;
    if (toolsRegistered) {
        return;
    }
    toolsRegistered = true;

    auto& mcp = McpServer::GetInstance();
    mcp.AddTool(
        "water_heater.set_temperature",
        "Set the requested water temperature in Celsius. Allowed range is 10 to 90 degrees. "
        "Use this when the user asks to raise, lower, or set the water heater temperature.",
        PropertyList({Property("temperature", kPropertyTypeInteger, 50, 10, 90)}),
        [](const PropertyList& properties) -> ReturnValue {
            const int temperature = properties["temperature"].value<int>();
            if (!HajAliI2C::IsReady()) {
                return std::string("Water-heater I2C link is not ready; the temperature command was not sent.");
            }
            if (!HajAliI2C::SetTargetTemperature(static_cast<uint8_t>(temperature))) {
                return std::string("Temperature must be between 10 and 90 degrees Celsius.");
            }
            return std::string("Requested water temperature ") +
                   std::to_string(temperature) +
                   " degrees Celsius is queued. The controller receives it on its next I2C read.";
        });


    // Keep the home-control API compact: the XiaoZhi model/client may only
    // consume the first MCP tools/list page. One binary tool covers all switches.
    mcp.AddTool(
        "home.control",
        "Control a home device. Use device identifiers: heating_element, kitchen_light, living_room_light, reception_light, room1_light, room2_light, bedroom_light, bedside_light, yard_light, storage_light, window, electric_shutter, gas_stove, heater, building_gas_valve, building_water_valve, socket1, socket2, range_hood. action must be on, off, or status. Call this tool when the user asks to turn a listed device on or off, or asks its state. Do not say the device is not connected; the I2C master reads the requested state.",
        PropertyList({Property("device", kPropertyTypeString), Property("action", kPropertyTypeString)}),
        [](const PropertyList& properties) -> ReturnValue {
            const std::string device = properties["device"].value<std::string>();
            const std::string action = properties["action"].value<std::string>();
            uint8_t index = 255;
            if (device == "heating_element") index = HajAliI2C::kTxHeatingElement;
            else if (device == "kitchen_light") index = HajAliI2C::kTxKitchenLight;
            else if (device == "living_room_light") index = HajAliI2C::kTxLivingRoomLight;
            else if (device == "reception_light") index = HajAliI2C::kTxReceptionLight;
            else if (device == "room1_light") index = HajAliI2C::kTxRoom1Light;
            else if (device == "room2_light") index = HajAliI2C::kTxRoom2Light;
            else if (device == "bedroom_light") index = HajAliI2C::kTxBedroomLight;
            else if (device == "bedside_light") index = HajAliI2C::kTxBedsideLight;
            else if (device == "yard_light") index = HajAliI2C::kTxYardLight;
            else if (device == "storage_light") index = HajAliI2C::kTxStorageLight;
            else if (device == "window") index = HajAliI2C::kTxWindow;
            else if (device == "electric_shutter") index = HajAliI2C::kTxElectricShutter;
            else if (device == "gas_stove") index = HajAliI2C::kTxGasStove;
            else if (device == "heater") index = HajAliI2C::kTxHeater;
            else if (device == "building_gas_valve") index = HajAliI2C::kTxBuildingGasValve;
            else if (device == "building_water_valve") index = HajAliI2C::kTxBuildingWaterValve;
            else if (device == "socket1") index = HajAliI2C::kTxSocket1;
            else if (device == "socket2") index = HajAliI2C::kTxSocket2;
            else if (device == "range_hood") index = HajAliI2C::kTxRangeHood;

            if (index == 255) return std::string("Unknown device. Use one of the device identifiers in the tool description.");
            if (action == "status") {
                return std::string("{\\"device\\":\\"") + device +
                       "\\\",\\"value\\":" + std::to_string(HajAliI2C::GetTxValue(index)) + "}";
            }
            if (action != "on" && action != "off") {
                return std::string("Invalid action. Use on, off, or status.");
            }
            if (!HajAliI2C::IsReady()) return std::string("I2C slave is not initialized.");
            const uint8_t value = action == "on" ? 1 : 0;
            if (!HajAliI2C::SetTxValue(index, value, 0, 1)) {
                return std::string("Failed to update device state.");
            }
            ESP_LOGI(kTag, "MCP control: device=%s action=%s TX[%u]=%u",
                     device.c_str(), action.c_str(), index, value);
            return std::string(device) + (value ? " turned on; " : " turned off; ") +
                   "the new state is stored in the I2C transmit frame.";
        });

    // Numeric controls share one compact MCP tool to avoid MCP list pagination.
    mcp.AddTool(
        "home.set_level",
        "Set a numeric home-control value. device identifiers and ranges: air_conditioner (0-100 percent), lighting_effect (0-255), water_heater_power (0-100 percent), operating_mode (0=High Performance, 1=Smart, 2=Economy, 3=Low Performance). Call this for numeric level, power, lighting effect, or operating-mode requests.",
        PropertyList({Property("device", kPropertyTypeString), Property("value", kPropertyTypeInteger)}),
        [](const PropertyList& properties) -> ReturnValue {
            const std::string device = properties["device"].value<std::string>();
            const int value = properties["value"].value<int>();
            uint8_t index = 255;
            int minimum = 0;
            int maximum = 100;
            if (device == "air_conditioner") index = HajAliI2C::kTxAirConditioner;
            else if (device == "lighting_effect") { index = HajAliI2C::kTxLightingEffect; maximum = 255; }
            else if (device == "water_heater_power") index = HajAliI2C::kTxWaterHeaterPower;
            else if (device == "operating_mode") { index = HajAliI2C::kTxOperatingMode; maximum = 3; }
            if (index == 255) return std::string("Unknown numeric device.");
            if (value < minimum || value > maximum) {
                return std::string("Value outside allowed range: ") + std::to_string(minimum) +
                       " to " + std::to_string(maximum) + ".";
            }
            if (!HajAliI2C::IsReady()) return std::string("I2C slave is not initialized.");
            if (!HajAliI2C::SetTxValue(index, static_cast<uint8_t>(value),
                                       static_cast<uint8_t>(minimum), static_cast<uint8_t>(maximum))) {
                return std::string("Failed to update numeric value.");
            }
            ESP_LOGI(kTag, "MCP numeric control: device=%s value=%d TX[%u]",
                     device.c_str(), value, index);
            return std::string(device) + " updated to " + std::to_string(value) +
                   "; the new value is stored in the I2C transmit frame.";
        });

    mcp.AddTool(
        "water_heater.get_temperature",
        "Get the measured and requested water temperatures in Celsius.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            if (!HajAliI2C::IsReady()) {
                return std::string("Water-heater I2C link is not ready.");
            }
            if (!HajAliI2C::HasReceivedMasterFrame()) {
                return std::string("The water-heater controller has not sent its first I2C status frame yet.");
            }
            return std::string("{\"current_temperature\":") +
                   std::to_string(HajAliI2C::GetCurrentTemperature()) +
                   ",\"target_temperature\":" +
                   std::to_string(HajAliI2C::GetTargetTemperature()) + "}";
        });
}
