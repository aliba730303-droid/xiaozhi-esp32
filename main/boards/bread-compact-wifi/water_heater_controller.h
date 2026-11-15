#pragma once

#include <cstdint>

namespace HajAliI2C {

// Fixed 25-byte exchange with the LILYGO master.
// RX byte 0: measured water temperature in Celsius.
// TX byte 0: requested water temperature in Celsius.
constexpr uint8_t kDataSize = 25;

bool Initialize();
bool IsReady();
bool SetTargetTemperature(uint8_t temperature);
uint8_t GetCurrentTemperature();
uint8_t GetTargetTemperature();
bool HasReceivedMasterFrame();

}  // namespace HajAliI2C

void InitializeWaterHeaterTools();
