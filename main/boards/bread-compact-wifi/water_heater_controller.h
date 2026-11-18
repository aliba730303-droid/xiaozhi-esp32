#pragma once

#include <cstdint>

namespace HajAliI2C {

// Fixed 25-byte frame sent by Haj Ali (I2C slave) to the LILYGO master.
// Byte indices are zero-based. Both master-to-slave RX and slave-to-master TX frames are 25 bytes.
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

// Fixed 25-byte frame received from the LILYGO master. Byte indices are zero-based.
constexpr uint8_t kRxWaterTemperature = 0;
constexpr uint8_t kRxAirCleanliness = 1;
constexpr uint8_t kRxAirPollution = 2;
constexpr uint8_t kRxMethane = 3;
constexpr uint8_t kRxCarbonMonoxide = 4;
constexpr uint8_t kRxAlcohol = 5;
constexpr uint8_t kRxCigaretteSmoke = 6;
constexpr uint8_t kRxNaturalGas = 7;
constexpr uint8_t kRxDailyWater = 8;
constexpr uint8_t kRxMainsVoltage = 9;
constexpr uint8_t kRxInstantPowerRaw = 10;
constexpr uint8_t kRxTankFull = 11;
constexpr uint8_t kRxLeakageVoltage = 12;
constexpr uint8_t kRxSatelliteCount = 13;
constexpr uint8_t kRxCellularConnected = 14;
constexpr uint8_t kRxDailyEnergy = 15;
constexpr uint8_t kRxPresence = 16;
constexpr uint8_t kRxSignalQuality = 17;
constexpr uint8_t kRxHomeTemperature = 18;
constexpr uint8_t kRxYesterdayWater = 19;
constexpr uint8_t kRxYesterdayEnergy = 20;
constexpr uint8_t kRxMonthlyEnergy = 21;
constexpr uint8_t kRxPreviousMonthlyEnergy = 22;
constexpr uint8_t kRxMonthlyWater = 23;
constexpr uint8_t kRxPreviousMonthlyWater = 24;


bool Initialize();
bool IsReady();
bool SetTargetTemperature(uint8_t temperature);
bool SetTxValue(uint8_t index, uint8_t value, uint8_t minimum, uint8_t maximum);
uint8_t GetTxValue(uint8_t index);
uint8_t GetCurrentTemperature();
uint8_t GetRxValue(uint8_t index);
int32_t GetRxFrameAgeSeconds();
uint32_t GetNoPresenceSeconds();
uint8_t GetTargetTemperature();
bool HasReceivedMasterFrame();

}  // namespace HajAliI2C

void InitializeWaterHeaterTools();
