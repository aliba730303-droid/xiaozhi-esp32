#pragma once

#include <cstdint>

namespace HajAliI2C {

// Fixed 25-byte frame sent by Haj Ali (I2C slave) to the LILYGO master.
// Byte indices are zero-based. The master-to-slave RX frame is not changed yet.
constexpr uint8_t kDataSize = 25;
constexpr uint8_t kTxTargetTemperature = 0;
constexpr uint8_t kTxHeatingElement = 1;
constexpr uint8_t kTxOperatingMode = 2;
constexpr uint8_t kTxKitchenLight = 3;
constexpr uint8_t kTxLivingRoomLight = 4;
constexpr uint8_t kTxReceptionLight = 5;
constexpr uint8_t kTxRoom1Light = 6;
constexpr uint8_t kTxRoom2Light = 7;
constexpr uint8_t kTxBedroomLight = 8;
constexpr uint8_t kTxBedsideLight = 9;
constexpr uint8_t kTxYardLight = 10;
constexpr uint8_t kTxStorageLight = 11;
constexpr uint8_t kTxAirConditioner = 12;
constexpr uint8_t kTxWindow = 13;
constexpr uint8_t kTxElectricShutter = 14;
constexpr uint8_t kTxLightingEffect = 15;
constexpr uint8_t kTxWaterHeaterPower = 16;
constexpr uint8_t kTxGasStove = 17;
constexpr uint8_t kTxHeater = 18;
constexpr uint8_t kTxBuildingGasValve = 19;
constexpr uint8_t kTxBuildingWaterValve = 20;
constexpr uint8_t kTxSocket1 = 21;
constexpr uint8_t kTxSocket2 = 22;
constexpr uint8_t kTxRangeHood = 23;
constexpr uint8_t kTxFrameMarker = 24;
constexpr uint8_t kTxFrameMarkerValue = 116;

bool Initialize();
bool IsReady();
bool SetTargetTemperature(uint8_t temperature);
uint8_t GetCurrentTemperature();
uint8_t GetTargetTemperature();
bool HasReceivedMasterFrame();

}  // namespace HajAliI2C

void InitializeWaterHeaterTools();
