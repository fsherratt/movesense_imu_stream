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
    void configGattSvc();

    void subscribeAccel();
    void unsubscribeAccel();

    void subscribeGyro();
    void unsubscribeGyro();

    wb::ResourceId mMeasAccResourceId;
    wb::ResourceId mMeasGyroResourceId;

    wb::ResourceId mAccelCharResource;
    wb::ResourceId mGyroCharResource;

    int32_t mMeasSvcHandle;
    int32_t mAccelCharHandle;
    int32_t mGyroCharHandle;

    union uTripleFloatToBytes {
            struct {
                uint32_t timestamp;
                int16_t floatData[24];
            } data;
            uint8_t b[52];
    };
};
