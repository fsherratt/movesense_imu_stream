#include "movesense.h"

#include "BatteryManager.h"
#include "app-resources/resources.h"
#include "whiteboard/builtinTypes/UnknownStructure.h"

#include "common/core/debug.h"
#include "DebugLogger.hpp"
#include "common/core/dbgassert.h"

#include "component_led/resources.h"
#include "component_max3000x/resources.h"
#include "system_mode/resources.h"
#include "system_states/resources.h"
#include "ui_ind/resources.h"

#define POWEROFF_TIMEOUT_MS 1800000 // 30 Mins (ms)

const char* const BatteryManager::LAUNCHABLE_NAME = "BatteryManager";

BatteryManager::BatteryManager() : ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
                                   LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION)
{
    mTimer = whiteboard::ID_INVALID_TIMER;
}

BatteryManager::~BatteryManager()
{
}

bool BatteryManager::initModule()
{   
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void BatteryManager::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool BatteryManager::startModule()
{
    subscribeMovementState();
    
    mModuleState = WB_RES::ModuleStateValues::STARTED;
    return true;
}
 
void BatteryManager::stopModule()
{
    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

whiteboard::Result BatteryManager::subscribeMovementState()
{
    DEBUGLOG("BLEService::openSubscriptions");

    wb::Result result = getResource("/System/States/0", mSysStateResourceId);
    if (!wb::RETURN_OKC(result))
        return whiteboard::HTTP_CODE_BAD_REQUEST;

    // Get initial value
    asyncGet( mSysStateResourceId );

    // Subscribe to battery managment state
    result = asyncSubscribe(mSysStateResourceId, AsyncRequestOptions::Empty);

    if (!wb::RETURN_OKC(result))
    {
        DEBUGLOG("asyncSubscribe threw error: %u", result);
        return whiteboard::HTTP_CODE_BAD_REQUEST;
    }

    return whiteboard::HTTP_CODE_ACCEPTED;
}

void BatteryManager::onGetResult(whiteboard::RequestId requestId, whiteboard::ResourceId resourceId, 
        whiteboard::Result resultCode, const whiteboard::Value& rResultData)
{
    processIncoming( resourceId, rResultData );
}

void BatteryManager::onNotify(whiteboard::ResourceId resourceId, const whiteboard::Value& rValue, 
            const whiteboard::ParameterList& rParameters)
{
    processIncoming( resourceId, rValue );
}

void BatteryManager::processIncoming( whiteboard::ResourceId resourceId, const whiteboard::Value& rValue )
{
    // Process incoming data
    switch (resourceId.localResourceId)
    {
        // Movement State
        case WB_RES::LOCAL::SYSTEM_STATES_STATEID::LID:
        {
            const WB_RES::StateChange& movementState =  rValue.convertTo<const WB_RES::StateChange&>();
            movementStateChange( movementState.newState );
            break;
        }
    }
}

void BatteryManager::movementStateChange( uint32_t movement )
{
    // Device is moving
    if ( movement == 1)
    {
        clearTimer();
    }
    // Device is stationary
    else
    {
        setTimer(POWEROFF_TIMEOUT_MS);
    }
    
}

void BatteryManager::onTimer(whiteboard::TimerId timerId)
{
    if (timerId != mTimer)
    {
        return;
    }

    clearTimer();

    // Set wakup command to trigger from MAX300X
    asyncPut(WB_RES::LOCAL::COMPONENT_MAX3000X_WAKEUP::ID,
            AsyncRequestOptions(NULL, 0, true), (uint8_t)1);

    // Place in full power off mode
    asyncPut(WB_RES::LOCAL::SYSTEM_MODE::ID,
            AsyncRequestOptions(NULL, 0, true), // Force async
            (uint8_t)WB_RES::SystemMode::FULLPOWEROFF);
}

void BatteryManager::setTimer( size_t timeout )
{
    if ( mTimer != whiteboard::ID_INVALID_TIMER )
    {
        clearTimer();
    }
    
    // Start a single shot timer
    mTimer = whiteboard::ResourceClient::startTimer(timeout, false);
}

void BatteryManager::clearTimer()
{
    whiteboard::ResourceClient::stopTimer(mTimer);
    mTimer = whiteboard::ID_INVALID_TIMER;
}

/* EOF */
