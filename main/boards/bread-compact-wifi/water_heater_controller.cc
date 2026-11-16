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


    // Register each home control as an actual MCP tool. Each operation updates
    // the corresponding byte in the 25-byte I2C frame.
    auto addBinaryControl = [&mcp](const char* toolPrefix, const char* label, uint8_t index) {
        const std::string onName = std::string(toolPrefix) + ".turn_on";
        const std::string offName = std::string(toolPrefix) + ".turn_off";
        const std::string stateName = std::string(toolPrefix) + ".get_state";
        const std::string onDescription = std::string("Turn on ") + label + " and set its I2C status byte to 1.";
        const std::string offDescription = std::string("Turn off ") + label + " and set its I2C status byte to 0.";
        const std::string stateDescription = std::string("Read the current requested state of ") + label + ".";
        mcp.AddTool(onName, onDescription, PropertyList(),
            [index](const PropertyList&) -> ReturnValue {
                if (!HajAliI2C::IsReady()) return std::string("I2C link is not ready.");
                HajAliI2C::SetTxValue(index, 1, 0, 1);
                return true;
            });
        mcp.AddTool(offName, offDescription, PropertyList(),
            [index](const PropertyList&) -> ReturnValue {
                if (!HajAliI2C::IsReady()) return std::string("I2C link is not ready.");
                HajAliI2C::SetTxValue(index, 0, 0, 1);
                return true;
            });
        mcp.AddTool(stateName, stateDescription, PropertyList(),
            [index](const PropertyList&) -> ReturnValue {
                return std::string("{\\"value\\":") + std::to_string(HajAliI2C::GetTxValue(index)) + "}";
            });
    };

    auto addNumericControl = [&mcp](const char* toolName, const char* description,
                                    const char* propertyName, uint8_t index,
                                    int defaultValue, int minimum, int maximum) {
        mcp.AddTool(toolName, description,
            PropertyList({Property(propertyName, kPropertyTypeInteger, defaultValue, minimum, maximum)}),
            [index, minimum, maximum, propertyName](const PropertyList& properties) -> ReturnValue {
                const int value = properties[propertyName].value<int>();
                if (!HajAliI2C::IsReady()) return std::string("I2C link is not ready.");
                if (value < minimum || value > maximum ||
                    !HajAliI2C::SetTxValue(index, static_cast<uint8_t>(value),
                                           static_cast<uint8_t>(minimum), static_cast<uint8_t>(maximum))) {
                    return std::string("Value is outside the allowed range.");
                }
                return std::string("Value updated to ") + std::to_string(value) +
                       ". It will be sent to the master on its next I2C read.";
            });
    };

    addBinaryControl("home.heating_element", "the water heater heating element", HajAliI2C::kTxHeatingElement);
    addNumericControl("home.set_operating_mode",
        "Set the water heater operating mode: 0=High Performance, 1=Smart Mode, 2=Economy, 3=Low Performance.",
        "mode", kTxOperatingMode, 0, 0, 3);
    addBinaryControl("home.kitchen_light", "the kitchen light", HajAliI2C::kTxKitchenLight);
    addBinaryControl("home.living_room_light", "the living room light", HajAliI2C::kTxLivingRoomLight);
    addBinaryControl("home.reception_light", "the reception light", HajAliI2C::kTxReceptionLight);
    addBinaryControl("home.room1_light", "the first room light", HajAliI2C::kTxRoom1Light);
    addBinaryControl("home.room2_light", "the second room light", HajAliI2C::kTxRoom2Light);
    addBinaryControl("home.bedroom_light", "the bedroom light", HajAliI2C::kTxBedroomLight);
    addBinaryControl("home.bedside_light", "the bedside light", HajAliI2C::kTxBedsideLight);
    addBinaryControl("home.yard_light", "the yard light", HajAliI2C::kTxYardLight);
    addBinaryControl("home.storage_light", "the storage room light", HajAliI2C::kTxStorageLight);
    addNumericControl("home.set_air_conditioner",
        "Set air conditioner level from 0 to 100 percent; 0 means off.",
        "level", kTxAirConditioner, 0, 0, 100);
    addBinaryControl("home.window", "the window", HajAliI2C::kTxWindow);
    addBinaryControl("home.electric_shutter", "the electric shutter", HajAliI2C::kTxElectricShutter);
    addNumericControl("home.set_lighting_effect",
        "Set the lighting effect value from 0 to 255.",
        "effect", kTxLightingEffect, 0, 0, 255);
    addNumericControl("water_heater.set_power",
        "Set water heater operating power from 0 to 100 percent.",
        "percent", kTxWaterHeaterPower, 0, 0, 100);
    addBinaryControl("home.gas_stove", "the gas stove", HajAliI2C::kTxGasStove);
    addBinaryControl("home.heater", "the room heater", HajAliI2C::kTxHeater);
    addBinaryControl("home.building_gas_valve", "the building gas valve", HajAliI2C::kTxBuildingGasValve);
    addBinaryControl("home.building_water_valve", "the building water valve", HajAliI2C::kTxBuildingWaterValve);
    addBinaryControl("home.socket1", "socket number one", HajAliI2C::kTxSocket1);
    addBinaryControl("home.socket2", "socket number two", HajAliI2C::kTxSocket2);
    addBinaryControl("home.range_hood", "the range hood", HajAliI2C::kTxRangeHood);

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
