#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

class BatteryManager FINAL : private wb::ResourceClient, public wb::LaunchableModule
{
public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;

    BatteryManager();
    ~BatteryManager();

private:
    whiteboard::TimerId mTimer;

    /**
    *	Subscribe to the movement state
    */
    whiteboard::Result subscribeMovementState();
    void movementStateChange( uint32_t movement );

    void setTimer( size_t timeout );

    void clearTimer();
    
private:
    whiteboard::ResourceId mSysStateResourceId; /**< Training data subscription */

    virtual bool initModule() OVERRIDE; /**< @see whiteboard::ILaunchableModule::initModule */
    virtual void deinitModule() OVERRIDE; /**< @see whiteboard::ILaunchableModule::deinitModule */
    virtual bool startModule() OVERRIDE; /**< @see whiteboard::ILaunchableModule::startModule */
    virtual void stopModule() OVERRIDE; /**< @see whiteboard::ILaunchableModule::stopModule */

    /**
    *	Callback for asynchronous resource GET requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onGetResult(whiteboard::RequestId requestId, whiteboard::ResourceId resourceId, 
            whiteboard::Result resultCode, const whiteboard::Value& rResultData);

    /**
    *	Callback for resource notifications.
    *
    *	@param resourceId Resource id associated with the update
    *	@param rValue Current value of the resource
    *	@param rParameters Notification parameters
    */
    void onNotify( whiteboard::ResourceId resourceId, const whiteboard::Value& rValue, 
            const whiteboard::ParameterList& rParameters);
    /**
    *	Processing incoming asyncronous data 
    *
    *	@param resourceId Resource id associated with the update
    *	@param rValue Current value of the resource
    */
    void processIncoming( whiteboard::ResourceId resourceId, const whiteboard::Value& rValue );
    /**
    *	Timer callback.
    *
    *	@param timerId Id of timer that triggered
    */
    virtual void onTimer(whiteboard::TimerId timerId) OVERRIDE;
};