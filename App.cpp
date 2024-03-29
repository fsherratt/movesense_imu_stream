#include "CustomGATTSvcClient.h"
#include "BatteryManager.h"
#include "movesense.h"

MOVESENSE_APPLICATION_STACKSIZE(1024)

MOVESENSE_PROVIDERS_BEGIN(2)
MOVESENSE_PROVIDER_DEF(CustomGATTSvcClient)
MOVESENSE_PROVIDER_DEF(BatteryManager)
MOVESENSE_PROVIDERS_END(2)

MOVESENSE_FEATURES_BEGIN()
// Explicitly enable or disable Movesense framework core modules.
// List of modules and their default state is found in documentation
OPTIONAL_CORE_MODULE(DataLogger, false)
OPTIONAL_CORE_MODULE(Logbook, false)
OPTIONAL_CORE_MODULE(LedService, true)
OPTIONAL_CORE_MODULE(IndicationService, true)
OPTIONAL_CORE_MODULE(BleService, true)
OPTIONAL_CORE_MODULE(EepromService, false)
OPTIONAL_CORE_MODULE(BypassService, false)
OPTIONAL_CORE_MODULE(SystemMemoryService, false)
OPTIONAL_CORE_MODULE(DebugService, true)
OPTIONAL_CORE_MODULE(BleStandardHRS, false)
OPTIONAL_CORE_MODULE(BleNordicUART, false)
OPTIONAL_CORE_MODULE(CustomGattService, true)

// NOTE: If building a simulator build, these macros are obligatory!
DEBUGSERVICE_BUFFER_SIZE(6, 120); // 6 lines, 120 characters total
DEBUG_EEPROM_MEMORY_AREA(false, 0, 2048) // EEPROM storage disabled
LOGBOOK_MEMORY_AREA(0, 384 * 1024);

APPINFO_NAME("IMU Data Logging GATT");
APPINFO_VERSION("0.1");
APPINFO_COMPANY("F_Sherratt");

// NOTE: SERIAL_COMMUNICATION macro has been DEPRECATED
BLE_COMMUNICATION(true)
MOVESENSE_FEATURES_END()
