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

CustomGATTSvcClient::CustomGATTSvcClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
    LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
    mMeasSvcHandle(0),
    mCharHandle(0),
    highRate(0),
    mCommBlePeersResourceId(wb::ID_INVALID_RESOURCE),
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
    WB_RES::GattChar characteristics[1];
    WB_RES::GattChar &measChar = characteristics[0];
    // WB_RES::GattChar &measChar2 = characteristics[1];

    constexpr uint8_t SENSOR_DATASERVICE_UUID[] = {0x78, 0xDA, 0xAA, 0x46, 0xA0, 0x01, 0x45, 0x2E, \
                                                   0x97, 0x30, 0xDF, 0x01, 0x3D, 0x22, 0x68, 0x8F};
    
    // Define the CMD characteristics
    WB_RES::GattProperty CharProp = WB_RES::GattProperty::NOTIFY;
    measChar.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp, 1);
    measChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cCharUUID16), 2);

    // WB_RES::GattProperty CharProp2 = WB_RES::GattProperty::NOTIFY;
    // measChar2.props = wb::MakeArray<WB_RES::GattProperty>( &CharProp2, 1);
    // measChar2.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&cChar2UUID16), 2);

    // Combine chars to service
    customGattSvc.uuid = wb::MakeArray<uint8_t>(SENSOR_DATASERVICE_UUID, sizeof(SENSOR_DATASERVICE_UUID));
    customGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 1);

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

void CustomGATTSvcClient::subscribeIMU()
{
    configAccel();
    configGyro();
    configMagn();

    if ( mMeasIMUResourceId != wb::ID_INVALID_RESOURCE )
        return;
    
    wb::Result result = getResource("Meas/IMU9/104", mMeasIMUResourceId);
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
                // else if(uuid16 == cChar2UUID16)
                // {
                //     mChar2Handle = c.handle.hasValue() ? c.handle.getValue() : 0;
                // }
            }

            if (!mCharHandle )//|| !mChar2Handle )
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mCharHandle);
            getResource(pathBuffer, mCharResource);
            // snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mMeasSvcHandle, mChar2Handle);
            // getResource(pathBuffer, mChar2Resource);

            // Subscribe to listen to characteristic 1 notifications (someone enables/disables the NOTIFY characteristic) 
            asyncSubscribe(mCharResource, AsyncRequestOptions::Empty);
            // Subscribe to listen to characteristic 2 notifications (someone enables/disables the NOTIFY characteristic) 
            // asyncSubscribe(mChar2Resource, AsyncRequestOptions::Empty);
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
            static uIMU9_104 uLowCharData;
            // static uIMU9_104 uHighCharData;

            const WB_RES::IMU9Data& imuValue = rValue.convertTo<const WB_RES::IMU9Data&>();
            uLowCharData.s.timestamp = imuValue.timestamp;

            if ( imuValue.arrayAcc.size() != 8)
            {
                DebugLogger::error("Inavlid data array");
                return;
            }
            
            const uint8_t ELEMENTS = 3;
            const uint8_t SAMPLESxELEMENTS = 24;
            const uint8_t SAMPLESxELEMENTSx2 = 48;
            
            const float accelUint16Scalar = 1000;
            const float gyroUint16Scalar = 8;
            const float magnUint16Scalar = 2;

            for ( size_t i = 0; i < imuValue.arrayAcc.size(); i++ )
            {
                const wb::FloatVector3D *accelValue = &imuValue.arrayAcc[i];
                const wb::FloatVector3D *gyroValue = &imuValue.arrayGyro[i];
                const wb::FloatVector3D *magnValue = &imuValue.arrayMagn[i];

                uint8_t ixELEMENTS = i*ELEMENTS;

                uLowCharData.s.data[ixELEMENTS + 0] = (int16_t)(accelValue->mX * accelUint16Scalar);
                uLowCharData.s.data[ixELEMENTS + 1] = (int16_t)(accelValue->mY * accelUint16Scalar);
                uLowCharData.s.data[ixELEMENTS + 2] = (int16_t)(accelValue->mZ * accelUint16Scalar);

                uLowCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 0] = (int16_t)(gyroValue->mX * gyroUint16Scalar);
                uLowCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 1] = (int16_t)(gyroValue->mY * gyroUint16Scalar);
                uLowCharData.s.data[SAMPLESxELEMENTS + ixELEMENTS + 2] = (int16_t)(gyroValue->mZ * gyroUint16Scalar);

                uLowCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 0] = (int16_t)(magnValue->mX * magnUint16Scalar);
                uLowCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 1] = (int16_t)(magnValue->mY * magnUint16Scalar);
                uLowCharData.s.data[SAMPLESxELEMENTSx2 + ixELEMENTS + 2] = (int16_t)(magnValue->mZ * magnUint16Scalar);
            }

            WB_RES::Characteristic newCharLowValue;
            newCharLowValue.bytes = wb::MakeArray<uint8_t>(uLowCharData.b, sizeof(uLowCharData.b));
            asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty,
                        mMeasSvcHandle, mCharHandle, newCharLowValue);

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
            // else if (parameterRef.getCharHandle() == mChar2Handle) 
            // {
            //     const WB_RES::Characteristic &charValue = rValue.convertTo<const WB_RES::Characteristic &>();
            //     bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
            //     DEBUGLOG("onNotify: mAccelCharHadle: bNotificationsEnabled: %d", bNotificationsEnabled);
            //     // Update the interval
            //     if (bNotificationsEnabled)
            //     {
            //         highRate = true;
            //         DebugLogger::verbose("Characteristic 2 Subscription started");
            //     }
            //     else
            //     {
            //         highRate = false;
            //         DebugLogger::verbose("Characteristic 2 Subscription stopped");
            //     }
                
            // }
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
            break;                
        }
    }
}
