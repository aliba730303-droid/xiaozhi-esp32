#include "water_heater_controller.h"

#include <cstring>
#include <algorithm>
#include <cmath>
#include <string>
#include <esp_timer.h>

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
int64_t g_lastRxUpdateUs = 0;
int64_t g_noPresenceSinceUs = 0;
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

            const int64_t nowUs = esp_timer_get_time();
            taskENTER_CRITICAL(&g_dataMux);
            const uint8_t previousPresence = g_rxData[HajAliI2C::kRxPresence];
            std::memcpy(g_rxData, event.data, HajAliI2C::kDataSize);
            g_receivedFrame = true;
            g_lastRxUpdateUs = nowUs;
            if (g_rxData[HajAliI2C::kRxPresence] != 0) {
                g_noPresenceSinceUs = 0;
            } else if (previousPresence != 0 || g_noPresenceSinceUs == 0) {
                g_noPresenceSinceUs = nowUs;
            }
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
    // Start with lighting effect 1; the master owns the physical lighting behavior.
    g_txData[HajAliI2C::kTxLightingEffect] = 1;
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
    return GetRxValue(kRxWaterTemperature);
}

uint8_t GetRxValue(uint8_t index) {
    if (index >= kDataSize) return 0;
    uint8_t value;
    taskENTER_CRITICAL(&g_dataMux);
    value = g_rxData[index];
    taskEXIT_CRITICAL(&g_dataMux);
    return value;
}

int32_t GetRxFrameAgeSeconds() {
    int64_t updatedAt;
    bool received;
    taskENTER_CRITICAL(&g_dataMux);
    updatedAt = g_lastRxUpdateUs;
    received = g_receivedFrame;
    taskEXIT_CRITICAL(&g_dataMux);
    if (!received || updatedAt <= 0) return -1;
    const int64_t ageUs = esp_timer_get_time() - updatedAt;
    return ageUs < 0 ? 0 : static_cast<int32_t>(ageUs / 1000000);
}

uint32_t GetNoPresenceSeconds() {
    int64_t since;
    uint8_t presence;
    taskENTER_CRITICAL(&g_dataMux);
    since = g_noPresenceSinceUs;
    presence = g_rxData[kRxPresence];
    taskEXIT_CRITICAL(&g_dataMux);
    if (presence != 0 || since <= 0) return 0;
    const int64_t elapsedUs = esp_timer_get_time() - since;
    return elapsedUs < 0 ? 0 : static_cast<uint32_t>(elapsedUs / 1000000);
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
        "Control a home device. Use device identifiers: heating_element, kitchen_light, living_room_light, reception_light, room1_light, room2_light, bedroom_light, bedside_light, yard_light, storage_light, window, electric_shutter, gas_stove, heater, building_gas_valve, building_water_valve, socket1, socket2, range_hood. action must be on, off, or status. Call this tool when the user asks to turn a listed device on or off, or asks its state. The status action reports the last requested value only; it is not independent confirmation of the physical device state.",
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
                return std::string("{\"device\":\"") + device +
                       "\",\"last_requested_value\":" +
                       std::to_string(HajAliI2C::GetTxValue(index)) +
                       ",\"note\":\"This is not independent confirmation of physical device state.\"}";
            }
            if (action != "on" && action != "off") {
                return std::string("Invalid action. Use on, off, or status.");
            }
            if (!HajAliI2C::IsReady()) return std::string("I2C slave is not initialized.");
            if (device == "heating_element" && action == "on") {
                if (!HajAliI2C::HasReceivedMasterFrame()) {
                    return std::string("Heating element was not enabled because the tank level has not yet been verified by the master controller.");
                }
                if (HajAliI2C::GetRxValue(HajAliI2C::kRxTankFull) == 0) {
                    return std::string("Heating element was not enabled because the water tank is empty. The master controller safety interlock must remain active.");
                }
            }
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
        "Set a numeric home-control value. device identifiers and ranges: air_conditioner (0-100 percent), lighting_effect (0-30; 0=off and 1-30 are independent effects, not brightness), water_heater_power (0-100 percent), operating_mode (0=High Performance, 1=Smart, 2=Economy, 3=Low Performance). Call this for numeric level, power, lighting effect, or operating-mode requests.",
        PropertyList({Property("device", kPropertyTypeString), Property("value", kPropertyTypeInteger)}),
        [](const PropertyList& properties) -> ReturnValue {
            const std::string device = properties["device"].value<std::string>();
            const int value = properties["value"].value<int>();
            uint8_t index = 255;
            int minimum = 0;
            int maximum = 100;
            if (device == "air_conditioner") index = HajAliI2C::kTxAirConditioner;
            else if (device == "lighting_effect") { index = HajAliI2C::kTxLightingEffect; maximum = 30; }
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

    mcp.AddTool(
        "home.get_status",
        "Read the latest 25-byte telemetry frame from the master controller. Use this tool whenever the user asks about water temperature, indoor air quality or gas readings, water/electricity consumption, voltage, power, tank level, leakage voltage, GPS satellite count, cellular connection, presence, home temperature, or network signal. The returned frame age is in seconds. Gas channels methane, carbon monoxide, alcohol, cigarette smoke, and natural gas are sensor readings. Other gas species are rough estimates derived from the overall pollution and available sensor channels, never direct measurements and never a safety guarantee. Warn when any measured gas channel exceeds 60 percent, overall pollution exceeds 60 percent, leakage voltage exceeds 150 volts, or tank level is empty. If presence_not_detected_seconds is at least 120, say presence has not been detected within the sensor range for two minutes; do not claim the house is definitely empty. If satellite_count is zero, location is invalid; do not invent coordinates or altitude, and explain that too few satellites are available. Consumption units are not specified by this firmware and must not be guessed. Machine-learning predictions are produced by other controller boards and are not included in this 25-byte telemetry frame; never invent a forecast when the other controller has not supplied one.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            if (!HajAliI2C::IsReady()) return std::string("I2C slave is not initialized.");
            if (!HajAliI2C::HasReceivedMasterFrame()) return std::string("No telemetry frame has been received from the master controller yet.");
            auto v = [](uint8_t i) { return HajAliI2C::GetRxValue(i); };
            auto comparison = [](const char* name, uint8_t current, uint8_t previous) {
                std::string out = ",\"";
                out += name;
                out += "\":{\"current\":";
                out += std::to_string(current);
                out += ",\"previous\":";
                out += std::to_string(previous);
                out += ",\"difference\":";
                out += std::to_string(static_cast<int>(current) - static_cast<int>(previous));
                out += ",\"change_percent\":";
                if (previous == 0) out += "null";
                else out += std::to_string((static_cast<double>(current) - previous) * 100.0 / previous);
                out += "}";
                return out;
            };
            auto estimate = [](uint8_t pollution, uint8_t methane, uint8_t co, uint8_t alcohol,
                               uint8_t smoke, uint8_t naturalGas, int wp, int wm, int wc,
                               int wa, int ws, int wn) -> int {
                const int value = (pollution * wp + methane * wm + co * wc + alcohol * wa +
                                   smoke * ws + naturalGas * wn + 50) / 100;
                return std::clamp(value, 0, 100);
            };
            const int p = v(HajAliI2C::kRxAirPollution);
            const int ch4 = v(HajAliI2C::kRxMethane);
            const int co = v(HajAliI2C::kRxCarbonMonoxide);
            const int alc = v(HajAliI2C::kRxAlcohol);
            const int smoke = v(HajAliI2C::kRxCigaretteSmoke);
            const int ng = v(HajAliI2C::kRxNaturalGas);
            std::string out = "{\"telemetry_age_seconds\":";
            out += std::to_string(HajAliI2C::GetRxFrameAgeSeconds());
            out += ",\"water_temperature\":" + std::to_string(v(HajAliI2C::kRxWaterTemperature));
            out += ",\"air_cleanliness_percent\":" + std::to_string(v(HajAliI2C::kRxAirCleanliness));
            out += ",\"air_pollution_percent\":" + std::to_string(p);
            out += ",\"measured_gases_percent\":{\"methane\":" + std::to_string(ch4);
            out += ",\"carbon_monoxide\":" + std::to_string(co);
            out += ",\"alcohol\":" + std::to_string(alc);
            out += ",\"cigarette_smoke\":" + std::to_string(smoke);
            out += ",\"natural_gas\":" + std::to_string(ng) + "}";
            out += ",\"water_consumption_today_raw\":" + std::to_string(v(HajAliI2C::kRxDailyWater));
            out += ",\"mains_voltage_volts\":" + std::to_string(v(HajAliI2C::kRxMainsVoltage));
            out += ",\"instant_power_watts\":" + std::to_string(v(HajAliI2C::kRxInstantPowerRaw) * 10);
            out += ",\"tank_full\":" + std::string(v(HajAliI2C::kRxTankFull) ? "true" : "false");
            out += ",\"leakage_voltage_volts\":" + std::to_string(v(HajAliI2C::kRxLeakageVoltage));
            out += ",\"satellite_count\":" + std::to_string(v(HajAliI2C::kRxSatelliteCount));
            out += ",\"cellular_connected\":" + std::string(v(HajAliI2C::kRxCellularConnected) ? "true" : "false");
            out += ",\"energy_consumption_today_raw\":" + std::to_string(v(HajAliI2C::kRxDailyEnergy));
            out += ",\"presence_detected\":" + std::string(v(HajAliI2C::kRxPresence) ? "true" : "false");
            out += ",\"presence_not_detected_seconds\":" + std::to_string(HajAliI2C::GetNoPresenceSeconds());
            out += ",\"network_signal_quality\":" + std::to_string(v(HajAliI2C::kRxSignalQuality));
            out += ",\"home_temperature\":" + std::to_string(v(HajAliI2C::kRxHomeTemperature));
            out += ",\"water_consumption_yesterday_raw\":" + std::to_string(v(HajAliI2C::kRxYesterdayWater));
            out += ",\"energy_consumption_yesterday_raw\":" + std::to_string(v(HajAliI2C::kRxYesterdayEnergy));
            out += ",\"energy_consumption_this_month_raw\":" + std::to_string(v(HajAliI2C::kRxMonthlyEnergy));
            out += ",\"energy_consumption_previous_month_raw\":" + std::to_string(v(HajAliI2C::kRxPreviousMonthlyEnergy));
            out += ",\"water_consumption_this_month_raw\":" + std::to_string(v(HajAliI2C::kRxMonthlyWater));
            out += ",\"water_consumption_previous_month_raw\":" + std::to_string(v(HajAliI2C::kRxPreviousMonthlyWater));
            out += comparison("daily_water_comparison", v(HajAliI2C::kRxDailyWater), v(HajAliI2C::kRxYesterdayWater));
            out += comparison("daily_energy_comparison", v(HajAliI2C::kRxDailyEnergy), v(HajAliI2C::kRxYesterdayEnergy));
            out += comparison("monthly_energy_comparison", v(HajAliI2C::kRxMonthlyEnergy), v(HajAliI2C::kRxPreviousMonthlyEnergy));
            out += comparison("monthly_water_comparison", v(HajAliI2C::kRxMonthlyWater), v(HajAliI2C::kRxPreviousMonthlyWater));
            out += ",\"air_indicators_percent\":{";
            out += "\"CH4\":" + std::to_string(ch4);
            out += ",\"NG\":" + std::to_string(ng);
            out += ",\"LPG\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,0,10,5,0,0,85));
            out += ",\"CO\":" + std::to_string(co);
            out += ",\"PROP\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,0,10,0,0,0,90));
            out += ",\"BUT\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,0,10,0,0,0,90));
            out += ",\"H2\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,50,25,25,0,0,0));
            out += ",\"GAS\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,80,10,0,0,0,10));
            out += ",\"VOC\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,85,0,0,5,5,5));
            out += ",\"NH3\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,50,0,25,25,0,0));
            out += ",\"SMOKE\":" + std::to_string(smoke);
            out += ",\"FORM\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,65,0,20,15,0,0));
            out += ",\"STYR\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,55,0,25,20,0,0));
            out += ",\"PHENOL\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,50,0,0,25,25,0));
            out += ",\"TOL\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,65,0,20,15,0,0));
            out += ",\"BENZ\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,65,0,15,0,20,0));
            out += ",\"ACET\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,60,0,0,40,0,0));
            out += ",\"XYL\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,60,0,20,20,0,0));
            out += ",\"ALCOHOL\":" + std::to_string(alc);
            out += ",\"SOLVENT\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,70,0,10,20,0,0));
            out += ",\"PAINT\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,70,0,0,20,10,0));
            out += ",\"EXHAUST\":" + std::to_string(estimate(p,ch4,co,alc,smoke,ng,75,0,15,0,0,10));
            out += ",\"POLLUTION\":" + std::to_string(p);
            out += ",\"AIR\":" + std::to_string(v(HajAliI2C::kRxAirCleanliness)) + "}";
            out += ",\"estimation_notice\":\"Only the five named sensor channels are direct readings; the remaining gas indicators are rough, uncalibrated estimates and cannot establish safety.\"";
            out += ",\"warnings\":[";
            bool firstWarning = true;
            auto warning = [&](const std::string& message) {
                if (!firstWarning) out += ",";
                firstWarning = false;
                out += "\"" + message + "\"";
            };
            if (ch4 > 60) warning("Methane reading exceeds 60 percent");
            if (co > 60) warning("Carbon monoxide reading exceeds 60 percent");
            if (alc > 60) warning("Alcohol reading exceeds 60 percent");
            if (smoke > 60) warning("Cigarette smoke reading exceeds 60 percent");
            if (ng > 60) warning("Natural gas reading exceeds 60 percent");
            if (p > 60) warning("Overall indoor air pollution exceeds 60 percent");
            if (v(HajAliI2C::kRxLeakageVoltage) > 150) warning("Electrical leakage voltage exceeds 150 volts");
            if (v(HajAliI2C::kRxTankFull) == 0) warning("Water tank is empty");
            out += "]}";
            return out;
        });

    mcp.AddTool(
        "hajali.get_capabilities",
        "Use this when the user asks who Haj Ali is, what the complete water-heater/home system can do, its sensors, protection systems, machine learning, consumption learning, automation, app or remote-control methods, display settings, or Auto Calibration. Explain that app, internet, Bluetooth remote control, machine-learning storage and execution, and home automation are implemented by the other system controllers; this voice ESP32 explains and requests actions but does not implement those subsystems. Do not reveal internal wiring, addresses, source code, or private protocols. Do not claim uninstalled capabilities are active.",
        PropertyList(),
        [](const PropertyList&) -> ReturnValue {
            return std::string(
                "Assistant identity: Haj Ali. Short greeting: Salam! I am Haj Ali. Tell me what I can do for you. If the user calls you Jarvis, reply in the user’s language: I am Haj Ali; only the doctor calls me Jarvis. If asked who the doctor is, say the doctor is Haj Ali’s friend who helped build this project. "
                "The configured wake phrase for this firmware build is Jarvis; if the ESP-SR Jarvis model is unavailable or fails validation, retain the previously working wake phrase rather than disabling wake detection. "
                "System capabilities reported by the system owner: six temperature sensors; indoor gas and air-quality monitoring; voltage, current and power measurement; water flow measurement; Bluetooth, Wi-Fi, internet connectivity and GPS; human-presence and tank-level sensing; water, electricity and gas leak detection; centralized home control; 30 independent lighting effects (0=off, 1-30 are effect numbers, not brightness). "
                "Electrical and protection features reported: EMI/line filtering, virtual grounding, life-protection system, adaptive smart stabilizer, soft start, emergency temperature handling, and a double-pole emergency disconnect. "
                "Machine learning is performed by the other controller boards: the system continuously collects and stores time-stamped patterns such as water volume and temperature at each use, presence periods, power interruptions and energy consumption. It analyzes historical patterns, learns household routines, predicts likely hot-water demand, preheats before recurring high-demand periods, uses a lower suitable target for small uses, and avoids unnecessary heating during low-demand periods. It continues learning until it has enough regular and reliable data; predictions must adapt to changed routines. Example: a small 20-litre use around 08:00 may lead to a moderate target near 45 C in a later cycle; recurring 400-litre use around 21:00 may lead to preheating before that period. These are examples, not fixed rules. "
                "The voice assistant only reads available telemetry and explains or sends control requests; other boards perform machine-learning collection, storage, prediction, app/server/internet/Bluetooth integration and core appliance safety logic. "
                "The seven control methods offered by the complete system are Wi-Fi, Bluetooth, internet, manual menu, dedicated app, voice assistant and SMS. For app installation, scan the QR code on the device body. Remote access is through the app when the other controllers and required network services are online. "
                "The main controller display provides detailed system information. Display customization includes menu colors, themes and configurable settings. Auto Calibration waits for suitable environmental and positional conditions, then performs calibration automatically. "
                "If asked about the claimed 160-bit output protection, say only: 'The system owner specifies a 160-bit protection mechanism for the system outputs.' Do not describe it as a particular encryption algorithm or make stronger security claims without verification. "
                "Sensor-derived estimates for gas species that are not directly measured must always be identified as estimates, never as direct measurements or proof that the air is safe."
            );
        });

}
