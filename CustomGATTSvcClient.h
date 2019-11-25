#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

class CustomGATTSvcClient FINAL : private wb::ResourceClient, public wb::LaunchableModule
{
public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;
    CustomGATTSvcClient();
    ~CustomGATTSvcClient();

private:
    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;
    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;

    /** @see whiteboard::ResourceClient::onGetResult */
    virtual void onGetResult(wb::RequestId requestId,
                             wb::ResourceId resourceId,
                             wb::Result resultCode,
                             const wb::Value& rResultData);

    /** @see whiteboard::ResourceClient::onPostResult */
    virtual void onPostResult(wb::RequestId requestId,
                              wb::ResourceId resourceId,
                              wb::Result resultCode,
                              const wb::Value& rResultData) OVERRIDE;
    
    /** @see whiteboard::ResourceClient::onNotify */
    virtual void onNotify(wb::ResourceId resourceId,
                          const wb::Value& rValue,
                          const wb::ParameterList& rParameters);

private:
    void subscribeBlePeers();
    void unsubscribeBlePeers();

    wb::ResourceId mCommBlePeersResourceId;
    bool deviceConnected;

    void configGattSvc();

    void subscribeChar();
    void unsubscribeChar();

    void configAccel();
    void subscribeAccel();
    void unsubscribeAccel();

    void configGyro();
    void subscribeGyro();
    void unsubscribeGyro();

    void configMagn();
    void subscribeMagn();
    void unsubscribeMagn();

    void subscribeIMU();
    void unsubscribeIMU();

    wb::ResourceId mMeasAccResourceId;
    wb::ResourceId mMeasGyroResourceId;
    wb::ResourceId mMeasMagnResourceId;
    wb::ResourceId mMeasIMUResourceId;

    wb::ResourceId mCharResource;
    wb::ResourceId mChar2Resource;

    int32_t mMeasSvcHandle;
    int32_t mCharHandle;
    int32_t mChar2Handle;

    bool highRate;

    union uData_104 {
        struct {
            uint32_t timestamp;
            int16_t data[8][3];
        } s;
        uint8_t b[52];
    };

    union uIMU9_104 {
        struct {
            uint32_t timestamp;
            int16_t accel[4][3];
            int16_t gyro[4][3];
            int16_t magn[4][3];
        } s;
        uint8_t b[76];
    };
};
