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

    void configAccel();
    void configGyro();
    void configMagn();

    bool subscribeIMU( const char* streamName );
    void unsubscribeIMU();
    void subscribeTemp();
    void unsubscribeTemp();
    void subscribeHr();
    void unsubscribeHr();

    void subscribeBatt();
    void unsubscribeBatt();

    uint8_t updatedBitmask;

    int16_t temperature;
    uint16_t heartrate;
    uint16_t rrInterval;

    wb::ResourceId mMeasIMUResourceId;
    wb::ResourceId mMeasTmpResourceId;
    wb::ResourceId mMeasHrResourceId;

    wb::ResourceId mCharLRResource;
    wb::ResourceId mCharHRResource;

    wb::ResourceId mMeasBattResourceId;

    bool dataHighRate;

    int32_t mMeasSvcHandle;
    int32_t mCharLRHandle; // Low rate Characteristic
    int32_t mCharHRHandle; // High rate Characteristic

    union uIMU9_8 {
        struct {
            uint32_t timestamp;
            int16_t temperature;
            uint16_t heartrate;
            uint16_t rrInterval;
            int16_t data[72];
            uint8_t updated;

        } s;
        uint8_t b[156];
    };
};
