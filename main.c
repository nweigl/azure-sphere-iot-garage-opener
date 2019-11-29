/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// Based on the sample C application for Azure Sphere demonstrating Azure IoT SDK C APIs provided by microsoft.

// The application uses the Azure IoT SDK C APIs to
// 1. Read the current position of two garage doors.
// 2. Open two garage doors using relays connected to the switches on an opener remote.

// You will need to provide four pieces of information to use this application, all of which are set
// in the app_manifest.json.
// 1. The Scope Id for your IoT Central application (set in 'CmdArgs')
// 2. The Tenant Id obtained from 'azsphere tenant show-selected' (set in 'DeviceAuthentication')
// 3. The Azure DPS Global endpoint address 'global.azure-devices-provisioning.net'
//    (set in 'AllowedConnections')
// 4. The IoT Hub Endpoint address for your IoT Central application (set in 'AllowedConnections')

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>

// By default, this sample is targeted at the MT3620 Reference Development Board (RDB).
// This can be changed using the "AzureSphereTargetHardwareDefinitionDirectory" property in
// CMakeSettings.json (for Visual Studio), or the value passed to
// -DAZURE_SPHERE_TARGET_HARDWARE_DEFINITION_DIRECTORY when invoking cmake from the command line.
//
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

#include "epoll_timerfd_utilities.h"

// Azure IoT SDK
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>

static volatile sig_atomic_t terminationRequired = false;

#include "parson.h"

#define DOOR_ONE_CONTROL MT3620_GPIO0
#define DOOR_TWO_CONTROL MT3620_GPIO1
#define DOOR_ONE_STATUS MT3620_GPIO16
#define DOOR_TWO_STATUS MT3620_GPIO17

// Azure IoT Hub/Central defines.
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in
                                     // app_manifest.json, CmdArgs

static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static int MethodCallback(const char* methodName, const unsigned char* payload, size_t size, unsigned char** response, size_t* response_size, void* userContextCallback);
static void TwinReportStringState(const char* propertyName, const char* propertyValue);
static void ReportStatusCallback(int result, void* context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SetupAzureClient(void);

// Initialization/Cleanup
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// LED
static int doorOneControlGpioFd = -1;
static int doorTwoControlGpioFd = -1;
static int doorOneStatusGpioFd = -1;
static int doorTwoStatusGpioFd = -1;

// Timer / polling
static int buttonPollTimerFd = -1;
static int azureTimerFd = -1;
static int epollFd = -1;

// Door Status Check
static GPIO_Value_Type doorOneStatus = GPIO_Value_Low;
static GPIO_Value_Type doorTwoStatus = GPIO_Value_Low;

// Azure IoT poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;

static int azureIoTPollPeriodSeconds = -1;

static void ButtonPollTimerEventHandler(EventData *eventData);
static void DoorOneStatusChangedHandler(void);
static void DoorTwoStatusChangedHandler(void);
static void AzureTimerEventHandler(EventData *eventData);

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("IoT Hub/Central Application starting.\n");

    if (argc == 2) {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    } else {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }

    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Main loop
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

    ClosePeripheralsAndHandlers();

    Log_Debug("Application exiting.\n");

    return 0;
}

/// <summary>
/// Button timer event:  Check the status of doors.
/// </summary>
static void ButtonPollTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
        terminationRequired = true;
        return;
    }
	DoorOneStatusChangedHandler();
	DoorTwoStatusChangedHandler();
}

/// <summary>
/// Azure timer event:  Check connection status and send telemetry
/// </summary>
static void AzureTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(azureTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) {
        if (isNetworkReady && !iothubAuthenticated) {
            SetupAzureClient();
			GPIO_GetValue(doorOneStatusGpioFd, &doorOneStatus);
			TwinReportStringState("DoorOneStatus", doorOneStatus == GPIO_Value_High ? "Open" : "Closed");
			GPIO_GetValue(doorTwoStatusGpioFd, &doorTwoStatus);
			TwinReportStringState("DoorTwoStatus", doorTwoStatus == GPIO_Value_High ? "Open" : "Closed");
        }
    } else {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated) {
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonPollEventData = {.eventHandler = &ButtonPollTimerEventHandler};
static EventData azureEventData = {.eventHandler = &AzureTimerEventHandler};

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

    // Open door one status as input
    Log_Debug("Opening DOOR_ONE_STATUS as input\n");
	doorOneStatusGpioFd = GPIO_OpenAsInput(DOOR_ONE_STATUS);
    if (doorOneStatusGpioFd < 0) {
        Log_Debug("ERROR: Could not open door one status: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    // Open door two status as input
    Log_Debug("Opening DOOR_TWO_STATUS as input\n");
    doorTwoStatusGpioFd = GPIO_OpenAsInput(DOOR_TWO_STATUS);
    if (doorTwoStatusGpioFd < 0) {
        Log_Debug("ERROR: Could not open door two status: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    // LED 4 Blue is used to show Device Twin settings state
    Log_Debug("Opening DOOR_ONE_CONTROL as output\n");
    doorOneControlGpioFd =
        GPIO_OpenAsOutput(DOOR_ONE_CONTROL, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (doorOneControlGpioFd < 0) {
        Log_Debug("ERROR: Could not open door one control: %s (%d).\n", strerror(errno), errno);
        return -1;
    }

	Log_Debug("Opening DOOR_TWO_CONTROL as output\n");
	doorTwoControlGpioFd =
		GPIO_OpenAsOutput(DOOR_TWO_CONTROL, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (doorTwoControlGpioFd < 0) {
		Log_Debug("ERROR: Could not open door two control: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

    // Set up a timer to poll for button events.
    struct timespec buttonPressCheckPeriod = {0, 1000 * 1000};
    buttonPollTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonPollEventData, EPOLLIN);
    if (buttonPollTimerFd < 0) {
        return -1;
    }

    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
    azureTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &azureTelemetryPeriod, &azureEventData, EPOLLIN);
    if (azureTimerFd < 0) {
        return -1;
    }

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    Log_Debug("Closing file descriptors\n");

    CloseFdAndPrintError(buttonPollTimerFd, "ButtonTimer");
    CloseFdAndPrintError(azureTimerFd, "AzureTimer");
    CloseFdAndPrintError(doorOneControlGpioFd, "DoorOneOpen");
	CloseFdAndPrintError(doorTwoControlGpioFd, "DoorTwoOpen");
	CloseFdAndPrintError(doorOneStatusGpioFd, "DoorOneStatus");
	CloseFdAndPrintError(doorTwoStatusGpioFd, "DoorTwoStatus");
    CloseFdAndPrintError(epollFd, "Epoll");
}

/// <summary>
///     Sets the IoT Hub authentication state for the app
///     The SAS Token expires which will set the authentication state
/// </summary>
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                        IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                        void *userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Hub Authenticated: %s\n", GetReasonString(reason));
}

/// <summary>
///     Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
///     When the SAS Token for a device expires the connection needs to be recreated
///     which is why this is not simply a one time call.
/// </summary>
static void SetupAzureClient(void)
{
    if (iothubClientHandle != NULL)
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);

    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000,
                                                                          &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              getAzureSphereProvisioningResultString(provResult));

    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK) {

        // If we fail to connect, reduce the polling frequency, starting at
        // AzureIoTMinReconnectPeriodSeconds and with a backoff up to
        // AzureIoTMaxReconnectPeriodSeconds
        if (azureIoTPollPeriodSeconds == AzureIoTDefaultPollPeriodSeconds) {
            azureIoTPollPeriodSeconds = AzureIoTMinReconnectPeriodSeconds;
        } else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }

        struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
        SetTimerFdToPeriod(azureTimerFd, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n",
                  azureIoTPollPeriodSeconds);
        return;
    }

    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
    SetTimerFdToPeriod(azureTimerFd, &azureTelemetryPeriod);

    iothubAuthenticated = true;

    if (IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_KEEP_ALIVE,
                                        &keepalivePeriodSeconds) != IOTHUB_CLIENT_OK) {
        Log_Debug("ERROR: failure setting option \"%s\"\n", OPTION_KEEP_ALIVE);
        return;
    }

	IoTHubDeviceClient_LL_SetDeviceMethodCallback(iothubClientHandle, MethodCallback, NULL);
    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle,
                                                      HubConnectionStatusCallback, NULL);
}

static int MethodCallback(const char* methodName, const unsigned char* payload, size_t size, unsigned char** response, size_t* response_size, void* userContextCallback)
{
	Log_Debug("Try to invoke method %s", methodName);
	const char* responseMessage = "\"Successfully invoke device method\"";
	int result = 200;

	if (strcmp(methodName, "CycleDoorOne") == 0)
	{
		const struct timespec sleepTime = { 1, 0 };
		GPIO_SetValue(doorOneControlGpioFd, GPIO_Value_Low);
		nanosleep(&sleepTime, NULL);
		GPIO_SetValue(doorOneControlGpioFd, GPIO_Value_High);
	}
	else if (strcmp(methodName, "CycleDoorTwo") == 0)
	{
		const struct timespec sleepTime = { 1, 0 };
		GPIO_SetValue(doorTwoControlGpioFd, GPIO_Value_Low);
		nanosleep(&sleepTime, NULL);
		GPIO_SetValue(doorTwoControlGpioFd, GPIO_Value_High);
	}
	else
	{
		Log_Debug("No method %s found", methodName);
		responseMessage = "\"No method found\"";
		result = 404;
	}

	*response_size = strlen(responseMessage) + 1;
	*response = (unsigned char*)strdup(responseMessage);

	return result;
}

/// <summary>
///     Converts the IoT Hub connection status reason to a string.
/// </summary>
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char *reasonString = "unknown reason";
    switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK";
        break;
    }
    return reasonString;
}

/// <summary>
///     Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
/// </summary>
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult)
{
    switch (provisioningResult.result) {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK";
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}

/// <summary>
///     Creates and enqueues a report containing the name and value pair of a Device Twin reported
///     property. The report is not sent immediately, but it is sent on the next invocation of
///     IoTHubDeviceClient_LL_DoWork().
/// </summary>
/// <param name="propertyName">the IoT Hub Device Twin property name</param>
/// <param name="propertyValue">the IoT Hub Device Twin property value</param>
static void TwinReportStringState(const char* propertyName, const char* propertyValue)
{
	if (iothubClientHandle == NULL) {
		Log_Debug("ERROR: client not initialized\n");
	}
	else {
		static char reportedPropertiesString[30] = { 0 };
		int len = snprintf(reportedPropertiesString, 30, "{\"%s\": \"%s\"}", propertyName, propertyValue);
		if (len < 0)
			return;

		if (IoTHubDeviceClient_LL_SendReportedState(
			iothubClientHandle, (unsigned char*)reportedPropertiesString,
			strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
			Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
		}
		else {
			Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName, propertyValue);
		}
	}
}

/// <summary>
///     Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
/// </summary>
static void ReportStatusCallback(int result, void* context)
{
	Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}

/// <summary>
/// Opening/Closing door one will:
///     Send an 'DoorOneStatus' event to Azure IoT Central
/// </summary>
static void DoorOneStatusChangedHandler(void)
{
	GPIO_Value_Type status;
	int result = GPIO_GetValue(doorOneStatusGpioFd, &status);
	if (result != 0)
	{
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
	}

	if (status != doorOneStatus)
	{
		TwinReportStringState("DoorOneStatus", status == GPIO_Value_High ? "Open" : "Closed");
		doorOneStatus = status;
	}
}

/// <summary>
/// Opening/Closing door two will:
///     Send an 'DoorTwoStatus' event to Azure IoT Central
/// </summary>
static void DoorTwoStatusChangedHandler(void)
{
	GPIO_Value_Type status;
	int result = GPIO_GetValue(doorTwoStatusGpioFd, &status);
	if (result != 0)
	{
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
	}

	if (status != doorTwoStatus)
	{
		TwinReportStringState("DoorTwoStatus", status == GPIO_Value_High ? "Open" : "Closed");
		doorTwoStatus = status;
	}
}
