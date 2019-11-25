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

#include "ui_ind/resources.h"

const char* const CustomGATTSvcClient::LAUNCHABLE_NAME = "CstGattS";

const uint16_t cCharUUID16 = 0x0001;
const uint16_t cChar2UUID16 = 0x0002;

const uint32_t SAMPLE_PERIOD_MS = 5;

const float accelUint16Scalar = 2000;
const float gyroUint16Scalar = 16;
const float magnUint16Scalar = 4;

CustomGATTSvcClient::CustomGATTSvcClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mMeasSvcHandle(0),
    mCharHandle(0),
    highRate(0),
    mCommBlePeersResourceId(wb::ID_INVALID_RESOURCE),
    mMeasAccResourceId(wb::ID_INVALID_RESOURCE),
    mMeasGyroResourceId(wb::ID_INVALID_RESOURCE),
    mMeasMagnResourceId(wb::ID_INVALID_RESOURCE),
    mMeasIMUResourceId(wb::ID_INVALID_RESOURCE),
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
    unsubscribeAccel();
    unsubscribeGyro();
    unsubscribeMagn();
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
    WB_RES::GattChar &measChar = characteristics[0];
    WB_RES::GattChar &measChar2 = characteristics[1];

    constexpr uint8_t SENSOR_DATASERVICE_UUID[] = {0x78, 0xDA, 0xAA, 0x46, 0xA0, 0x01, 0x45, 0x2E, \
                                                   0x97, 0x30, 0xDF, 0x01, 0x3D, 0x22, 0x68, 0x8F};
    
    // Define the CMD characteristics
    WB_RES::GattProperty CharProp = WB_RES::GattProperty::NOTIFY;
    measChar.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp, 1);
    measChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cCharUUID16), 2);

    WB_RES::GattProperty CharProp2 = WB_RES::GattProperty::NOTIFY;
    measChar2.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp2, 1);
    measChar2.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cChar2UUID16), 2);

    // Combine chars to service
    customGattSvc.uuid = wb::MakeArray<uint8_t>(SENSOR_DATASERVICE_UUID, sizeof(SENSOR_DATASERVICE_UUID));
    customGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 2);

    // Create custom service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, customGattSvc);
}

void CustomGATTSvcClient::subscribeChar()
{
    subscribeIMU();
}

void CustomGATTSvcClient::unsubscribeChar()
{
    unsubscribeIMU();
}

void CustomGATTSvcClient::configAccel()
{
    // Set Accelerometer FSR
    WB_RES::AccConfig accConfig;
    accConfig.gRange = 16; // 2, 4, 8, 16

    asyncPut(WB_RES::LOCAL::MEAS_ACC_CONFIG(), AsyncRequestOptions::Empty, accConfig);
}

void CustomGATTSvcClient::subscribeAccel()
{
    configAccel();

    if ( mMeasAccResourceId != wb::ID_INVALID_RESOURCE )
        return;

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
    if ( mMeasAccResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasAccResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasAccResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::configGyro()
{
    // Set Gyro DPS Range
    WB_RES::GyroConfig gyroConfig;
    gyroConfig.dPSRange = 2000; // 245,500,1000,2000

    asyncPut(WB_RES::LOCAL::MEAS_GYRO_CONFIG(), AsyncRequestOptions::Empty, gyroConfig);
}

void CustomGATTSvcClient::subscribeGyro()
{
    configGyro();

    if ( mMeasGyroResourceId != wb::ID_INVALID_RESOURCE )
        return;

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
    if ( mMeasGyroResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasGyroResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasGyroResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::configMagn()
{
}

void CustomGATTSvcClient::subscribeMagn()
{
    configMagn();

    if ( mMeasMagnResourceId != wb::ID_INVALID_RESOURCE )
        return;
    // Subscribe Accelerometer
    // Sample Rate - 13, 26, 52, 104, 208, 416, 833, 1666
    wb::Result result = getResource("Meas/Magn/104", mMeasMagnResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }
    
    result = asyncSubscribe(mMeasMagnResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
    }
}

void CustomGATTSvcClient::unsubscribeMagn()
{
    if ( mMeasMagnResourceId == wb::ID_INVALID_RESOURCE )
        return;

    wb::Result result = asyncUnsubscribe(mMeasMagnResourceId, NULL);

    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncUnsubscribe threw error: %u", result);
    }

    mMeasMagnResourceId = wb::ID_INVALID_RESOURCE;
}

void CustomGATTSvcClient::subscribeIMU()
{
    configAccel();
    configGyro();
    configMagn();

    if ( mMeasIMUResourceId != wb::ID_INVALID_RESOURCE )
        return;
    
    wb::Result result = getResource("Meas/IMU9/208", mMeasIMUResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return;
    }
    
    result = asyncSubscribe(mMeasIMUResourceId, AsyncRequestOptions(NULL, 0, true));
    if (!wb::RETURN_OKC(result))
    {
        DebugLogger::error("asyncsubscribe threw error: %u", result);
    }
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
                                      const wb::Value& rResultData)
{
    DEBUGLOG("CustomGATTSvcClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc &>();
            for (size_t i=0; i < svc.chars.size(); i++)
            {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));
                
                if(uuid16 == cCharUUID16)
                {
                    mCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                }
                else if(uuid16 == cChar2UUID16)
                {
                    mChar2Handle = c.handle.hasValue() ? c.handle.getValue() : 0;
                }
            }

            if (!mCharHandle || !mChar2Handle )
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mCharHandle);
            getResource(pathBuffer, mCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mChar2Handle);
            getResource(pathBuffer, mChar2Resource);

            // Subscribe to listen to characteristic 1 notifications (someone enables/disables the NOTIFY characteristic) 
            asyncSubscribe(mCharResource, AsyncRequestOptions::Empty);
            // Subscribe to listen to characteristic 2 notifications (someone enables/disables the NOTIFY characteristic) 
            asyncSubscribe(mChar2Resource, AsyncRequestOptions::Empty);
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
            uData_104 uCharData;
            float floatToUint16Scalar = accelUint16Scalar;

            const WB_RES::AccData& linearAccelerationValue = rValue.convertTo<const WB_RES::AccData&>();

            const wb::Array<wb::FloatVector3D>& arrayData = linearAccelerationValue.arrayAcc;
            uCharData.s.timestamp = linearAccelerationValue.timestamp;

            if ( arrayData.size() <= 0 )
                return;

            for ( size_t i = 0; i < arrayData.size()-1; i++ )
            {
                wb::FloatVector3D sensorValue = arrayData[i];

                uCharData.s.data[i][0] = (int16_t)(sensorValue.mX * floatToUint16Scalar);
                uCharData.s.data[i][1] = (int16_t)(sensorValue.mY * floatToUint16Scalar);
                uCharData.s.data[i][2] = (int16_t)(sensorValue.mZ * floatToUint16Scalar);
            }

            WB_RES::Characteristic newCharValue;
            newCharValue.bytes = wb::MakeArray<uint8_t>(uCharData.b, sizeof(uCharData.b));
            asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHandle, newCharValue);
            break;
        }

        case WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE::LID:
        {
            uData_104 uCharData;
            float floatToUint16Scalar = gyroUint16Scalar;

            const WB_RES::GyroData& gyroscopeValue = rValue.convertTo<const WB_RES::GyroData&>();

            const wb::Array<wb::FloatVector3D>& arrayData = gyroscopeValue.arrayGyro;
            uCharData.s.timestamp = gyroscopeValue.timestamp;

            if ( arrayData.size() <= 0 )
                return;

            for ( size_t i = 0; i < arrayData.size()-1; i++ )
            {
                wb::FloatVector3D sensorValue = arrayData[i];

                uCharData.s.data[i][0] = (int16_t)(sensorValue.mX * floatToUint16Scalar);
                uCharData.s.data[i][1] = (int16_t)(sensorValue.mY * floatToUint16Scalar);
                uCharData.s.data[i][2] = (int16_t)(sensorValue.mZ * floatToUint16Scalar);
            }

            WB_RES::Characteristic newCharValue;
            newCharValue.bytes = wb::MakeArray<uint8_t>(uCharData.b, sizeof(uCharData.b));
            asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHandle, newCharValue);
            break;
        }

        case WB_RES::LOCAL::MEAS_MAGN_SAMPLERATE::LID:
        {
            uData_104 uCharData;
            float floatToUint16Scalar = magnUint16Scalar;

            const WB_RES::MagnData& magnetometerValue = rValue.convertTo<const WB_RES::MagnData&>();

            const wb::Array<wb::FloatVector3D>& arrayData = magnetometerValue.arrayMagn;
            uCharData.s.timestamp = magnetometerValue.timestamp;

            if ( arrayData.size() <= 0 )
                return;

            for ( size_t i = 0; i < arrayData.size()-1; i++ )
            {
                wb::FloatVector3D sensorValue = arrayData[i];

                uCharData.s.data[i][0] = (int16_t)(sensorValue.mX * floatToUint16Scalar);
                uCharData.s.data[i][1] = (int16_t)(sensorValue.mY * floatToUint16Scalar);
                uCharData.s.data[i][2] = (int16_t)(sensorValue.mZ * floatToUint16Scalar);
            }

            WB_RES::Characteristic newCharValue;
            newCharValue.bytes = wb::MakeArray<uint8_t>(uCharData.b, sizeof(uCharData.b));
            asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHandle, newCharValue);
            break;
        }

        case WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::LID:
        {
            uIMU9_104 uLowCharData;
            uIMU9_104 uHghCharData;

            const WB_RES::IMU9Data& imuValue = rValue.convertTo<const WB_RES::IMU9Data&>();
            uLowCharData.s.timestamp = imuValue.timestamp;
            uHghCharData.s.timestamp = imuValue.timestamp + SAMPLE_PERIOD_MS;

            if ( imuValue.arrayAcc.size() <= 0 || imuValue.arrayAcc.size() > 8)
            {
                DebugLogger::error("Inavlid data array");
                return;
            }

            for ( size_t i = 0; i < imuValue.arrayAcc.size()-1; i+=2 )
            {
                uint8_t j = i/2;
                wb::FloatVector3D sensorValue = imuValue.arrayAcc[i];

                uLowCharData.s.accel[j][0] = (int16_t)(sensorValue.mX * accelUint16Scalar);
                uLowCharData.s.accel[j][1] = (int16_t)(sensorValue.mY * accelUint16Scalar);
                uLowCharData.s.accel[j][2] = (int16_t)(sensorValue.mZ * accelUint16Scalar);

                sensorValue = imuValue.arrayAcc[i+1];

                uHghCharData.s.accel[j][0] = (int16_t)(sensorValue.mX * accelUint16Scalar);
                uHghCharData.s.accel[j][1] = (int16_t)(sensorValue.mY * accelUint16Scalar);
                uHghCharData.s.accel[j][2] = (int16_t)(sensorValue.mZ * accelUint16Scalar);

                sensorValue = imuValue.arrayGyro[i];

                uLowCharData.s.gyro[j][0] = (int16_t)(sensorValue.mX * gyroUint16Scalar);
                uLowCharData.s.gyro[j][1] = (int16_t)(sensorValue.mY * gyroUint16Scalar);
                uLowCharData.s.gyro[j][2] = (int16_t)(sensorValue.mZ * gyroUint16Scalar);

                sensorValue = imuValue.arrayGyro[i+1];

                uHghCharData.s.gyro[j][0] = (int16_t)(sensorValue.mX * gyroUint16Scalar);
                uHghCharData.s.gyro[j][1] = (int16_t)(sensorValue.mY * gyroUint16Scalar);
                uHghCharData.s.gyro[j][2] = (int16_t)(sensorValue.mZ * gyroUint16Scalar);

                sensorValue = imuValue.arrayMagn[i];

                uLowCharData.s.magn[j][0] = (int16_t)(sensorValue.mX * magnUint16Scalar);
                uLowCharData.s.magn[j][1] = (int16_t)(sensorValue.mY * magnUint16Scalar);
                uLowCharData.s.magn[j][2] = (int16_t)(sensorValue.mZ * magnUint16Scalar);

                sensorValue = imuValue.arrayMagn[i+1];

                uHghCharData.s.magn[j][0] = (int16_t)(sensorValue.mX * magnUint16Scalar);
                uHghCharData.s.magn[j][1] = (int16_t)(sensorValue.mY * magnUint16Scalar);
                uHghCharData.s.magn[j][2] = (int16_t)(sensorValue.mZ * magnUint16Scalar);
            }

            WB_RES::Characteristic newCharLowValue;
            newCharLowValue.bytes = wb::MakeArray<uint8_t>(uLowCharData.b, sizeof(uLowCharData.b));
            asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHandle, newCharLowValue);

            if ( highRate ) 
            {
                WB_RES::Characteristic newCharHighValue;
                newCharHighValue.bytes = wb::MakeArray<uint8_t>(uHghCharData.b, sizeof(uHghCharData.b));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                            mMeasSvcHandle, mChar2Handle, newCharHighValue);
            }

            break;
        }

        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mCharHandle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    subscribeChar();
                    DebugLogger::verbose("Characteristic Subscription started");

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::CONTINUOUS_VISUAL_INDICATION;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
                else
                {
                    unsubscribeChar();
                    DebugLogger::verbose("Characteristic Subscription stopped");

                    const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::NO_VISUAL_INDICATIONS;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
                }
                
            }
            else if (parameterRef.getCharHandle() == mChar2Handle) 
            {
                const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
                // Update the interval
                if (bNotificationsEnabled)
                {
                    highRate = true;
                    DebugLogger::verbose("Characteristic 2 Subscription started");
                }
                else
                {
                    highRate = false;
                    DebugLogger::verbose("Characteristic 2 Subscription stopped");
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
                    unsubscribeChar();
                }

                const WB_RES::VisualIndType type = WB_RES::VisualIndTypeValues::SHORT_VISUAL_INDICATION;
                asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty, type);
            }                    
        }
    }
}
