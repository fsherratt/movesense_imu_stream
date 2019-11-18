#include "movesense.h"

#include "CustomGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "DebugLogger.hpp"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"

const char* const CustomGATTSvcClient::LAUNCHABLE_NAME = "CstGattS";

const uint16_t accelCharUUID16 = 0x0001;
const uint16_t gyroCharUUID16 = 0x0002;

CustomGATTSvcClient::CustomGATTSvcClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION)
{
}

CustomGATTSvcClient::~CustomGATTSvcClient()
{
}

bool CustomGATTSvcClient::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void CustomGATTSvcClient::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool CustomGATTSvcClient::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Configure custom gatt service
    configGattSvc();
    return true;
}

void CustomGATTSvcClient::stopModule()
{
    // Stop timer
    unsubscribeAccel();
    unsubscribeGyro();
    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void CustomGATTSvcClient::configGattSvc()
{
    WB_RES::GattSvc customGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &accelChar = characteristics[0];
    WB_RES::GattChar &gyroChar = characteristics[1];

    constexpr uint8_t SENSOR_DATASERVICE_UUID[] = {0x78, 0xDA, 0xAA, 0x46, 0xA0, 0x01, 0x45, \
                                                   0x2E, 0x97, 0x30, 0xDF, 0x01, 0x3D, 0x22, 0x68, 0x8F};
    
    // Define the CMD characteristics
    WB_RES::GattProperty accelCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty gyroCharProp = WB_RES::GattProperty::NOTIFY;

    accelChar.props = wb::MakeArray<WB_RES::GattProperty>( &accelCharProp, 1);
    accelChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&accelCharUUID16), 2);

    gyroChar.props = wb::MakeArray<WB_RES::GattProperty>( &gyroCharProp, 1);
    gyroChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&gyroCharUUID16), 2);

    // Combine chars to service
    customGattSvc.uuid = wb::MakeArray<uint8_t>(SENSOR_DATASERVICE_UUID, sizeof(SENSOR_DATASERVICE_UUID));
    customGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 2);

    // Create custom service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, customGattSvc);
}

void CustomGATTSvcClient::subscribeAccel()
{
    // Set Accelerometer FSR
    WB_RES::AccConfig accConfig;
    accConfig.gRange = 16; // 2, 4, 8, 16

    asyncPut(WB_RES::LOCAL::MEAS_ACC_CONFIG(), AsyncRequestOptions::Empty, accConfig);

    // Subscribe Accelerometer
    // Sample Rate - 13, 26, 52, 104, 208, 416, 833, 1666
    wb::Result result = getResource("Meas/Acc/104", mMeasAccResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }
    
    result = asyncSubscribe(mMeasAccResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
    }
}

void CustomGATTSvcClient::unsubscribeAccel()
{
    wb::Result result = asyncUnsubscribe(mMeasAccResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasAccResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::subscribeGyro()
{
    // Set Gyro DPS Range
    WB_RES::GyroConfig gyroConfig;
    gyroConfig.dPSRange = 2000; // 245,500,1000,2000

    asyncPut(WB_RES::LOCAL::MEAS_GYRO_CONFIG(), AsyncRequestOptions::Empty, gyroConfig);

    // Subscribe Accelerometer
    // Sample Rate - 13, 26, 52, 104, 208, 416, 833, 1666
    wb::Result result = getResource("Meas/Gyro/104", mMeasGyroResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }
    
    result = asyncSubscribe(mMeasGyroResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
    }
}

void CustomGATTSvcClient::unsubscribeGyro()
{
    wb::Result result = asyncUnsubscribe(mMeasGyroResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasAccResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::onGetResult(wb::RequestId requestId,
                                      wb::ResourceId resourceId,
                                      wb::Result resultCode,
                                      const wb::Value& rResultData)
{
    DEBUGLOG("CustomGATTSvcClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc &>();
            for (size_t i=0; i<svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));
                
                if(uuid16 == accelCharUUID16)
                    mAccelCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                else if(uuid16 == gyroCharUUID16)
                    mGyroCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
            }

            if (!mAccelCharHandle || !mGyroCharHandle)
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mAccelCharHandle);
            getResource(pathBuffer, mAccelCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mGyroCharHandle);
            getResource(pathBuffer, mGyroCharResource);

            // Subscribe to listen to accelChar notifications (someone enables/disables the INDICATE characteristic) 
            asyncSubscribe(mAccelCharResource, AsyncRequestOptions::Empty);
            // Subscribe to listen to gyroChar notifications (someone enables/disables the INDICATE characteristic) 
            asyncSubscribe(mGyroCharResource, AsyncRequestOptions::Empty);
            break;
        }
    }
}

void CustomGATTSvcClient::onNotify(wb::ResourceId resourceId,
                                   const wb::Value& rValue,
                                   const wb::ParameterList& rParameters)
{
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::LID:
        {
            const WB_RES::AccData& linearAccelerationValue = rValue.convertTo<const WB_RES::AccData&>();

            if ( linearAccelerationValue.arrayAcc.size() <= 0 )
                return;
            else if ( linearAccelerationValue.arrayAcc.size() > 8 )
                return;

            const wb::Array<wb::FloatVector3D>& accelData = linearAccelerationValue.arrayAcc;
            const uint32_t timestamp = linearAccelerationValue.timestamp;

            uTripleFloatToBytes accel;
            accel.data.timestamp = timestamp;

            for ( size_t i = 0; i < accelData.size()-1; i++ )
            // size_t i = 0;
            {
                wb::FloatVector3D accValue = accelData[i];

                // int16_t max = 32767
                // FSR = 16
                // Scalar = int16_t_max/16 = 2047.9 => 2000

                accel.data.floatData[0+(i*3)] = (int16_t)(accValue.mX*2000);
                accel.data.floatData[1+(i*3)] = (int16_t)(accValue.mY*2000);
                accel.data.floatData[2+(i*3)] = (int16_t)(accValue.mZ*2000);

                WB_RES::Characteristic newAccelCharValue;
                newAccelCharValue.bytes = wb::MakeArray<uint8_t>(accel.b, sizeof(accel.b));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                            mMeasSvcHandle, mAccelCharHandle, newAccelCharValue);
            }
            break;
        }
        case WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE::LID:
        {
            const WB_RES::GyroData& gyroscopeValue = rValue.convertTo<const WB_RES::GyroData&>();

            if ( gyroscopeValue.arrayGyro.size() <= 0 )
                return;
            else if ( gyroscopeValue.arrayGyro.size() > 8 )
                return;

            const wb::Array<wb::FloatVector3D>& gyroData = gyroscopeValue.arrayGyro;
            const uint32_t timestamp = gyroscopeValue.timestamp;

            uTripleFloatToBytes gyro;
            gyro.data.timestamp = timestamp;

            for ( size_t i = 0; i < gyroData.size()-1; i++ )
            {
                wb::FloatVector3D gyroValue = gyroData[i];

                // int16_t max = 32767
                // dPS = 2000
                // Scalar = int16_t_max/2000 = 16.38 => 16

                gyro.data.floatData[0+(i*3)] = (int16_t)(gyroValue.mX*16);
                gyro.data.floatData[1+(i*3)] = (int16_t)(gyroValue.mY*16);
                gyro.data.floatData[2+(i*3)] = (int16_t)(gyroValue.mZ*16);

                WB_RES::Characteristic newGyroCharValue;
                newGyroCharValue.bytes = wb::MakeArray<uint8_t>(gyro.b, sizeof(gyro.b));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                            mMeasSvcHandle, mGyroCharHandle, newGyroCharValue);
            }
            break;
        }
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mAccelCharHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    subscribeAccel();
                }
                else
                {
                    unsubscribeAccel();
                }
                
            }
            else if (parameterRef.getCharHandle() == mGyroCharHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mGyroCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    subscribeGyro();
                }
                else
                {
                    unsubscribeGyro();
                }
                
            }
            break;
        }
    }
}

void CustomGATTSvcClient::onPostResult(wb::RequestId requestId, 
                                       wb::ResourceId resourceId, 
                                       wb::Result resultCode, 
                                       const wb::Value& rResultData)
{
    DEBUGLOG("CustomGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Custom Gatt service was created
        mMeasSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Custom Gatt service was created. handle: %d", mMeasSvcHandle);
        
        // Request more info about created svc so we get the char handles
        asyncGet(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(), AsyncRequestOptions::Empty, mMeasSvcHandle);
    }
}
