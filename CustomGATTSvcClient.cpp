#include "movesense.h"

#include "CustomGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "DebugLogger.hpp"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"


#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"
#include "meas_temp/resources.h"
#include "meas_hr/resources.h"

#include "ui_ind/resources.h"

const char* const CustomGATTSvcClient::LAUNCHABLE_NAME = "CstGattS";

const uint16_t cCharLrUUID16 = 0x0001;
const uint16_t cCharHrUUID16 = 0x0002;

#define TEMP_UPDATE_OFFSET 0
#define HR_UPDATE_OFFSET 1
#define RR_INT_UPDATE_OFFSET 2
#define IMU_UPDATE_OFFSET 3

CustomGATTSvcClient::CustomGATTSvcClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mMeasSvcHandle(0),
    mCharLRHandle(0),
    mCharHRHandle(0),
    dataHighRate(false),
    temperature(0),
    heartrate(0),
    rrInterval(0),
    updatedBitmask(0),
    mCommBlePeersResourceId(wb::ID_INVALID_RESOURCE),
    mMeasIMUResourceId(wb::ID_INVALID_RESOURCE),
    mMeasTmpResourceId(wb::ID_INVALID_RESOURCE),
    mMeasHrResourceId(wb::ID_INVALID_RESOURCE),
    deviceConnected(WB_RES::PeerStateValues::DISCONNECTED)
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
    subscribeBlePeers();
    return true;
}

void CustomGATTSvcClient::stopModule()
{
    // Stop timer
    unsubscribeIMU();
    unsubscribeBlePeers();
    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void CustomGATTSvcClient::subscribeBlePeers()
{
    if ( mCommBlePeersResourceId != wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = getResource("/Comm/Ble/Peers", mCommBlePeersResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }
    
    result = asyncSubscribe(mCommBlePeersResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
    }
}

void CustomGATTSvcClient::unsubscribeBlePeers()
{
    if ( mCommBlePeersResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mCommBlePeersResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mCommBlePeersResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::configGattSvc()
{
    WB_RES::GattSvc customGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &measLrChar = characteristics[0];
    WB_RES::GattChar &measHrChar = characteristics[1];

    constexpr uint8_t SENSOR_DATASERVICE_UUID[] = {0x78, 0xDA, 0xAA, 0x46, 0xA0, 0x01, 0x45, 0x2E, \
                                                   0x97, 0x30, 0xDF, 0x01, 0x3D, 0x22, 0x68, 0x8F};
    
    // Define the CMD characteristics
    WB_RES::GattProperty CharProp = WB_RES::GattProperty::NOTIFY;
    measLrChar.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp, 1);
    measLrChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cCharLrUUID16), 2);

    measHrChar.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp, 1);
    measHrChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cCharHrUUID16), 2);

    // Combine chars to service
    customGattSvc.uuid = wb::MakeArray<uint8_t>(SENSOR_DATASERVICE_UUID, sizeof(SENSOR_DATASERVICE_UUID));
    customGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 2);// 2 = Number of characteristics

    // Create custom service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, customGattSvc);
}

void CustomGATTSvcClient::configAccel()
{
    // Set Accelerometer FSR
    WB_RES::AccConfig accConfig;
    accConfig.gRange = 16; // 2, 4, 8, 16

    asyncPut(WB_RES::LOCAL::MEAS_ACC_CONFIG(), AsyncRequestOptions::Empty, accConfig);
}

void CustomGATTSvcClient::configGyro()
{
    // Set Gyro DPS Range
    WB_RES::GyroConfig gyroConfig;
    gyroConfig.dPSRange = 2000; // 245,500,1000,2000

    asyncPut(WB_RES::LOCAL::MEAS_GYRO_CONFIG(), AsyncRequestOptions::Empty, gyroConfig);
}

void CustomGATTSvcClient::configMagn()
{
}

bool CustomGATTSvcClient::subscribeIMU(const char* streamName)
{
    if ( mMeasIMUResourceId != wb::ID_INVALID_RESOURCE )
        return false;
    
    wb::Result result = getResource(streamName, mMeasIMUResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return false;
    }

    configAccel();
    configGyro();
    configMagn();

    result = asyncSubscribe(mMeasIMUResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
        return false;
    }

    return true;
}

void CustomGATTSvcClient::unsubscribeIMU()
{
    if ( mMeasIMUResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasIMUResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasIMUResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::subscribeTemp()
{
    if ( mMeasTmpResourceId != wb::ID_INVALID_RESOURCE )
        return;
    
    wb::Result result = getResource("/Meas/Temp", mMeasTmpResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }

    // Get initial value
    asyncGet( WB_RES::LOCAL::MEAS_TEMP() );

    result = asyncSubscribe(mMeasTmpResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
        return;
    }
}

void CustomGATTSvcClient::unsubscribeTemp()
{
     if ( mMeasTmpResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasTmpResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasTmpResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::subscribeHr()
{
    if ( mMeasHrResourceId != wb::ID_INVALID_RESOURCE )
        return;
    
    wb::Result result = getResource("/Meas/Hr", mMeasHrResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }

    result = asyncSubscribe(mMeasHrResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
        return;
    }
}

void CustomGATTSvcClient::unsubscribeHr()
{
     if ( mMeasHrResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasHrResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasHrResourceId = wb::ID_INVALID_RESOURCE;
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

void CustomGATTSvcClient::onGetResult(wb::RequestId requestId,
                                      wb::ResourceId resourceId,
                                      wb::Result resultCode,
                                      const wb::Value& rValue)
{
    DEBUGLOG("CustomGATTSvcClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            const WB_RES::GattSvc &svc = rValue.convertTo<const WB_RES::GattSvc &>();
            for (size_t i=0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));
                
                if(uuid16 == cCharLrUUID16)
                {
                    mCharLRHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                } 
                else if (uuid16 == cCharHrUUID16)
                {
                    mCharHRHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                }
            }

            if (!mCharLRHandle || !mCharHRHandle )
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mCharLRHandle);
            getResource(pathBuffer, mCharLRResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mCharHRHandle);
            getResource(pathBuffer, mCharHRResource);

            // Subscribe to listen to characteristic notifications (someone enables/disables the NOTIFY characteristic) 
            asyncSubscribe(mCharLRResource, AsyncRequestOptions::Empty);
            asyncSubscribe(mCharHRResource, AsyncRequestOptions::Empty);
            
            break;
        }

        case WB_RES::LOCAL::MEAS_TEMP::LID:
        {
            const WB_RES::TemperatureValue& tempValue = rValue.convertTo<const WB_RES::TemperatureValue&>();
            temperature = (uint32_t)((tempValue.measurement -273.15) * 1000);
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
        case WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::LID:
        {
            static uIMU9_8 uCharData;

            const WB_RES::IMU9Data& imuValue = rValue.convertTo<const WB_RES::IMU9Data&>();
            uCharData.s.timestamp = imuValue.timestamp;

            if ( imuValue.arrayAcc.size() != 8)
            {
                DebugLogger::error("Inavlid data array");
                return;
            }
            
            const uint8_t ELEMENTS = 3;
            const uint8_t SAMPLESxELEMENTS = 24;
            const uint8_t SAMPLESxELEMENTSx2 = 48;
            
            const float accelUint16Scalar = 256; // Max 128 m/s (13G)
            const float gyroUint16Scalar = 32; // Max 1000 DPS
            const float magnUint16Scalar = 1; // Max 32000 uT

            uCharData.s.temperature = temperature;
            uCharData.s.heartrate = heartrate;
            uCharData.s.rrInterval = rrInterval;

            for ( size_t i = 0; i < imuValue.arrayAcc.size(); i++ )
            {
                const wb::FloatVector3D *accelValue = &imuValue.arrayAcc[i];
                const wb::FloatVector3D *gyroValue = &imuValue.arrayGyro[i];
                const wb::FloatVector3D *magnValue = &imuValue.arrayMagn[i];

                uint8_t ixELEMENTS = i*ELEMENTS;

                uCharData.s.data[ixELEMENTS + 0] = (int16_t)(accelValue->mX * accelUint16Scalar);
                uCharData.s.data[ixELEMENTS + 1] = (int16_t)(accelValue->mY * accelUint16Scalar);
                uCharData.s.data[ixELEMENTS + 2] = (int16_t)(accelValue->mZ * accelUint16Scalar);

                uCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 0] = (int16_t)(gyroValue->mX * gyroUint16Scalar);
                uCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 1] = (int16_t)(gyroValue->mY * gyroUint16Scalar);
                uCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 2] = (int16_t)(gyroValue->mZ * gyroUint16Scalar);

                uCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 0] = (int16_t)(magnValue->mX * magnUint16Scalar);
                uCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 1] = (int16_t)(magnValue->mY * magnUint16Scalar);
                uCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 2] = (int16_t)(magnValue->mZ * magnUint16Scalar);
            }

            updatedBitmask |= (0x01 << IMU_UPDATE_OFFSET);

            uCharData.s.updated = updatedBitmask;
            WB_RES::Characteristic newCharValue;
            newCharValue.bytes = wb::MakeArray<uint8_t>(uCharData.b, sizeof(uCharData.b));
            if (dataHighRate) {
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHRHandle, newCharValue);
            } else {
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharLRHandle, newCharValue);
            }

            updatedBitmask = 0;

            break;
        }

        case WB_RES::LOCAL::MEAS_TEMP::LID:
        {
            const WB_RES::TemperatureValue& tempValue = rValue.convertTo<const WB_RES::TemperatureValue&>();
            temperature = (int16_t)((tempValue.measurement -273.15) * 512.0f);

            updatedBitmask |= (0x01 << TEMP_UPDATE_OFFSET);

            break;
        }

        case WB_RES::LOCAL::MEAS_HR::LID:
        {
            const WB_RES::HRData& hrdata = rValue.convertTo<const WB_RES::HRData&>();
            heartrate = (uint16_t)(hrdata.average * 256.0f);
            updatedBitmask |= (0x01 << HR_UPDATE_OFFSET);

            if ( hrdata.rrData.size() > 0 ) {
                rrInterval = hrdata.rrData[0];
                updatedBitmask |= (0x01 << RR_INT_UPDATE_OFFSET);
            }

            break;
        }

        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mCharLRHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    if (! subscribeIMU("Meas/IMU9/104") )
                        return;

                    subscribeTemp();
                    subscribeHr();

                    DebugLogger::verbose("Characteristic LR Subscription started");
                    dataHighRate = false;

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::CONTINUOUS_VISUAL_INDICATION;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
                else
                {
                    if (dataHighRate)
                        return;

                    unsubscribeIMU();
                    unsubscribeTemp();
                    unsubscribeHr();

                    DebugLogger::verbose("Characteristic LR Subscription stopped");

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::NO_VISUAL_INDICATIONS;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
                
            }
            else if (parameterRef.getCharHandle() == mCharHRHandle)
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    if (! subscribeIMU("Meas/IMU9/208") )
                        return;

                    subscribeTemp();
                    subscribeHr();

                    DebugLogger::verbose("Characteristic HR Subscription started");
                    dataHighRate = true;

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::CONTINUOUS_VISUAL_INDICATION;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
                else
                {
                    if (!dataHighRate)
                        return;

                    unsubscribeIMU();
                    unsubscribeTemp();
                    unsubscribeHr();

                    DebugLogger::verbose("Characteristic HR Subscription stopped");

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::NO_VISUAL_INDICATIONS;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
            } 
            break;
        }

        case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
        {
            const WB_RES::PeerChange& peers = rValue.convertTo<const WB_RES::PeerChange&>();

            if ( deviceConnected != peers.state )
            {
                // Connection state has changed
                deviceConnected = peers.state;

                if (deviceConnected == WB_RES::PeerStateValues::DISCONNECTED)
                {
                    unsubscribeIMU();
                    unsubscribeTemp();
                    unsubscribeHr();
                }

                const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::SHORT_VISUAL_INDICATION;
                asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
            }
            break;                
        }
    }
}
