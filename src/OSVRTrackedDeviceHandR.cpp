/** @file
    @brief OSVR tracked device

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include "OSVRTrackedDeviceHandR.h"

#include "osvr_compiler_detection.h"
#include "make_unique.h"
#include "matrix_cast.h"
#include "osvr_device_properties.h"
#include "ValveStrCpy.h"
#include "platform_fixes.h" // strcasecmp

// OpenVR includes
#include <openvr_driver.h>

// Library/third-party includes
#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Interface.h> 
#include <osvr/Util/EigenInterop.h>
#include <util/FixedLengthStringFunctions.h>

// Standard includes
#include <cstring>
#include <ctime>
#include <string>
#include <iostream>
#include <exception>

OSVRTrackedDeviceHandR::OSVRTrackedDeviceHandR(const std::string& display_description, osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::IDriverLog* driver_log) : m_DisplayDescription(display_description), m_Context(context), driver_host_(driver_host), logger_(driver_log), pose_(), deviceClass_(vr::TrackedDeviceClass_Controller)
{
    // do nothing
}

OSVRTrackedDeviceHandR::~OSVRTrackedDeviceHandR()
{
    vr::IDriverLog* logger_ = nullptr;
    vr::IServerDriverHost* driver_host_ = nullptr;
}

vr::EVRInitError OSVRTrackedDeviceHandR::Activate(uint32_t object_id)
{
    const std::time_t waitTime = 5; // wait up to 5 seconds for init

    // Register tracker callback
    if (m_TrackerInterface.notEmpty()) {
        m_TrackerInterface.free();
    }

	
    // Ensure context is fully started up
    logger_->Log("Waiting for the context to fully start up...\n");
    std::time_t startTime = std::time(nullptr);
    while (!m_Context.checkStatus()) {
        m_Context.update();
        if (std::time(nullptr) > startTime + waitTime) {
            logger_->Log("Context startup timed out!\n");
            return vr::VRInitError_Driver_Failed;
        }
    }

	logger_->Log("Right Hand Context started!\n");

	// Load config from steamvr.vrsettings
	settings_ = driver_host_->GetSettings(vr::IVRSettings_Version);

	// Check enabled status
	if (driver_host_) {
		if (settings_) {
			if (!settings_->GetBool("osvr", "rightHandEnable", true)) {
				logger_->Log("Right hand controller disabled!\n");
				return vr::VRInitError_Driver_Failed;
			}
		}
	}

	// init config values
	device_id_ = settings_->GetInt32("osvr", "rightHandDeviceId", 2);
	rightHandVecWorldFromDriverTranslationX_ = settings_->GetFloat("osvr", "rightHandVecWorldFromDriverTranslationX", 0);
	rightHandVecWorldFromDriverTranslationY_ = settings_->GetFloat("osvr", "rightHandVecWorldFromDriverTranslationY", -0.5);
	rightHandVecWorldFromDriverTranslationZ_ = settings_->GetFloat("osvr", "rightHandVecWorldFromDriverTranslationZ", -1.1);

	m_TrackerInterface = m_Context.getInterface(handdevice);
	m_TrackerInterface.registerCallback(&OSVRTrackedDeviceHandR::HandRTrackerCallback, this);

	// init state
	packetNumCounter = 0;
	state_.ulButtonPressed = 0;
	state_.ulButtonTouched = 0;
	state_.unPacketNum = 0;
	state_.rAxis[0] = { 0, 0 };
	state_.rAxis[1] = { 0, 0 };

	last_pose_.poseIsValid = false;

	// register hydra button callbacks
	m_ButtonInterface1 = m_Context.getInterface(buttondevice_SteamVR_Trigger);
	m_ButtonInterface1.registerCallback(&OSVRTrackedDeviceHandR::Button1Callback, this);

	m_ButtonInterface2 = m_Context.getInterface(buttondevice_Dashboard_Back);
	m_ButtonInterface2.registerCallback(&OSVRTrackedDeviceHandR::Button2Callback, this);

	m_ButtonInterface3 = m_Context.getInterface(buttondevice_ApplicationMenu);
	m_ButtonInterface3.registerCallback(&OSVRTrackedDeviceHandR::Button3Callback, this);

	m_ButtonInterface4 = m_Context.getInterface(buttondevice_A);
	m_ButtonInterface4.registerCallback(&OSVRTrackedDeviceHandR::Button4Callback, this);

	m_ButtonInterfaceMiddle = m_Context.getInterface(buttondevice_System);
	m_ButtonInterfaceMiddle.registerCallback(&OSVRTrackedDeviceHandR::ButtonMiddleCallback, this);

	m_ButtonInterfaceBumper = m_Context.getInterface(buttondevice_Grip);
	m_ButtonInterfaceBumper.registerCallback(&OSVRTrackedDeviceHandR::ButtonBumperCallback, this);

	m_ButtonInterfaceJoystickButton = m_Context.getInterface(buttondevice_SteamVR_Touchpad);
	m_ButtonInterfaceJoystickButton.registerCallback(&OSVRTrackedDeviceHandR::ButtonJoystickButtonCallback, this);

	// register hydra analog callbacks
	m_AnalogInterfaceJoystickX = m_Context.getInterface(analogdevice_JoystickX);
	m_AnalogInterfaceJoystickX.registerCallback(&OSVRTrackedDeviceHandR::AnalogJoystickXCallback, this);

	m_AnalogInterfaceJoystickY = m_Context.getInterface(analogdevice_JoystickY);
	m_AnalogInterfaceJoystickY.registerCallback(&OSVRTrackedDeviceHandR::AnalogJoystickYCallback, this);

	m_AnalogInterfaceTrigger = m_Context.getInterface(analogdevice_Trigger);
	m_AnalogInterfaceTrigger.registerCallback(&OSVRTrackedDeviceHandR::AnalogTriggerCallback, this);

    return vr::VRInitError_None;
}

void OSVRTrackedDeviceHandR::Deactivate()
{
    /// Have to force freeing here
    if (m_TrackerInterface.notEmpty()) {
        m_TrackerInterface.free();
    }
}

void OSVRTrackedDeviceHandR::PowerOff()
{
    // FIXME Implement
}

void* OSVRTrackedDeviceHandR::GetComponent(const char* component_name_and_version)
{
    if (!strcasecmp(component_name_and_version, vr::IVRControllerComponent_Version)) {
        return (vr::IVRControllerComponent*)this;
    }

    // Override this to add a component to a driver
    return NULL;
}

void OSVRTrackedDeviceHandR::DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size)
{
    // TODO
    // make use of (from vrtypes.h) static const uint32_t k_unMaxDriverDebugResponseSize = 32768;
}


vr::DriverPose_t OSVRTrackedDeviceHandR::GetPose()
{
    return pose_;
}

bool OSVRTrackedDeviceHandR::GetBoolTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const bool default_value = false;

    if (isWrongDataType(prop, bool())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetBoolTrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    switch (prop) {
    case vr::Prop_WillDriftInYaw_Bool:
        if (error)
            *error = vr::TrackedProp_Success;
		return false;
        break;
    case vr::Prop_ReportsTimeSinceVSync_Bool: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
        break;
	case vr::Prop_IsOnDesktop_Bool: // TODO
		if (error)
			*error = vr::TrackedProp_ValueNotProvidedByDevice;
		return default_value;
		break;
	case vr::Prop_HasCamera_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;
	case vr::Prop_Firmware_UpdateAvailable_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;
	case vr::Prop_DeviceProvidesBatteryStatus_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;
	case vr::Prop_DeviceCanPowerOff_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;
	case vr::Prop_BlockServerShutdown_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;
	case vr::Prop_ContainsProximitySensor_Bool:
		if (error)
			*error = vr::TrackedProp_Success;
		return false;
		break;


	
	}

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

float OSVRTrackedDeviceHandR::GetFloatTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const float default_value = 0.0f;

    if (isWrongDataType(prop, float())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetFloatTrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    switch (prop) {
    case vr::Prop_SecondsFromVsyncToPhotons_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_DisplayFrequency_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_UserIpdMeters_Float:
        if (error)
			*error = vr::TrackedProp_ValueNotProvidedByDevice;
		return default_value;
	case vr::Prop_UserHeadToEyeDepthMeters_Float:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewLeftDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewRightDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewTopDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_FieldOfViewBottomDegrees_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingRangeMinimumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingRangeMaximumMeters_Float: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

int32_t OSVRTrackedDeviceHandR::GetInt32TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const int32_t default_value = 0;

    if (isWrongDataType(prop, int32_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetInt32TrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    switch (prop) {
    case vr::Prop_DeviceClass_Int32:
        if (error)
            *error = vr::TrackedProp_Success;
        return deviceClass_;
    case vr::Prop_Axis0Type_Int32: // TODO
		if (error)
			*error = vr::TrackedProp_Success;
		return vr::EVRControllerAxisType::k_eControllerAxis_Joystick;
    case vr::Prop_Axis1Type_Int32: // TODO
        if (error)
			*error = vr::TrackedProp_Success;
		return vr::EVRControllerAxisType::k_eControllerAxis_Trigger;
    case vr::Prop_Axis2Type_Int32: // TODO
        if (error)
			*error = vr::TrackedProp_Success;
		return vr::EVRControllerAxisType::k_eControllerAxis_None;
	case vr::Prop_Axis3Type_Int32: // TODO
        if (error)
			*error = vr::TrackedProp_Success;
		return vr::EVRControllerAxisType::k_eControllerAxis_None;
	case vr::Prop_Axis4Type_Int32: // TODO
        if (error)
			*error = vr::TrackedProp_Success;
		return vr::EVRControllerAxisType::k_eControllerAxis_None;
	}

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

uint64_t OSVRTrackedDeviceHandR::GetUint64TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    const uint64_t default_value = 0;

    if (isWrongDataType(prop, uint64_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetUint64TrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    switch (prop) {
	case vr::Prop_SupportedButtons_Uint64: // TODO
		if (error)
			*error = vr::TrackedProp_Success;
		return 0xFFFFFFFFFFFFFFFF;
	case vr::Prop_HardwareRevision_Uint64: // TODO
		if (error)
			*error = vr::TrackedProp_ValueNotProvidedByDevice;
		return default_value;
		
	
	}

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

vr::HmdMatrix34_t OSVRTrackedDeviceHandR::GetMatrix34TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error)
{
    // Default value is identity matrix
    vr::HmdMatrix34_t default_value;
    map(default_value) = Matrix34f::Identity();

    if (isWrongDataType(prop, vr::HmdMatrix34_t())) {
        if (error)
            *error = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (error)
            *error = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (error)
            *error = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetMatrix34TrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    switch (prop) {
    case vr::Prop_StatusDisplayTransform_Matrix34: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

uint32_t OSVRTrackedDeviceHandR::GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, char *pchValue, uint32_t unBufferSize, vr::ETrackedPropertyError *pError)
{
    uint32_t default_value = 0;
    if (isWrongDataType(prop, pchValue)) {
        if (pError)
            *pError = vr::TrackedProp_WrongDataType;
        return default_value;
    }

    if (isWrongDeviceClass(prop, deviceClass_)) {
        if (pError)
            *pError = vr::TrackedProp_WrongDeviceClass;
        return default_value;
    }

    if (vr::TrackedDeviceClass_Invalid == deviceClass_) {
        if (pError)
            *pError = vr::TrackedProp_InvalidDevice;
        return default_value;
    }

	if (logDebugProps) {
		const std::string msg = "OSVRTrackedDeviceHandR::GetFloatTrackedDeviceProperty(): Requested property: " + std::to_string(prop) + "\n";
		logger_->Log(msg.c_str());
	}

    std::string sValue = GetStringTrackedDeviceProperty(prop, pError);
    if (*pError == vr::TrackedProp_Success) {
        if (sValue.size() + 1 > unBufferSize) {
            *pError = vr::TrackedProp_BufferTooSmall;
        } else {
            valveStrCpy(sValue, pchValue, unBufferSize);
        }
        return static_cast<uint32_t>(sValue.size()) + 1;
    }

    return 0;
}

    // ------------------------------------
    // Private Methods
    // ------------------------------------

std::string OSVRTrackedDeviceHandR::GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError *error)
{
    std::string default_value = "";

#include "ignore-warning/push"
#include "ignore-warning/switch-enum"

    switch (prop) {
    case vr::Prop_TrackingSystemName_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_ModelNumber_String: // TODO
        if (error)
            *error = vr::TrackedProp_Success;
		return "Razer Hydra Right Controller";
	case vr::Prop_SerialNumber_String:
        if (error)
            *error = vr::TrackedProp_Success;
        return this->GetId();
    case vr::Prop_RenderModelName_String:
        if (error)
			*error = vr::TrackedProp_Success;
		return "vr_controller_vive_1_5";
	case vr::Prop_ManufacturerName_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_TrackingFirmwareVersion_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_HardwareRevision_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_AttachedDeviceId_String: // TODO
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_AllWirelessDongleDescriptions_String:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    case vr::Prop_ConnectedWirelessDongle_String:
        if (error)
            *error = vr::TrackedProp_ValueNotProvidedByDevice;
        return default_value;
    }

#include "ignore-warning/pop"

    if (error)
        *error = vr::TrackedProp_UnknownProperty;
    return default_value;
}

void OSVRTrackedDeviceHandR::HandRTrackerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_PoseReport* report)
{
	if (!userdata) {
		return;
	}

    auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

    vr::DriverPose_t pose;
    pose.poseTimeOffset = 0; // close enough

	Eigen::Vector3d::Map(pose.vecWorldFromDriverTranslation) = Eigen::Vector3d(self->rightHandVecWorldFromDriverTranslationX_, self->rightHandVecWorldFromDriverTranslationY_, self->rightHandVecWorldFromDriverTranslationZ_);
	Eigen::Vector3d::Map(pose.vecDriverFromHeadTranslation) = Eigen::Vector3d::Zero();

    map(pose.qWorldFromDriverRotation) = Eigen::Quaterniond::Identity();

    map(pose.qDriverFromHeadRotation) = Eigen::Quaterniond::Identity();

    // Position
    Eigen::Vector3d::Map(pose.vecPosition) = osvr::util::vecMap(report->pose.translation);

	// Velocity and acceleration are not provided, so we try to make up something based on the 125hz refresh rate of the hydra + make it a little less jumpy
	if (self->last_pose_.poseIsValid) {
		float permanent_acceleration_ratio = 0.1;

		float dampening = 0.1;
		float refresh = 125;
		float minimum = 0.1;

		double xvel = abs(self->pose_.vecPosition[0] - self->last_pose_.vecPosition[0]) * refresh * dampening;
		if (xvel < minimum) xvel = 0;
		double yvel = abs(self->pose_.vecPosition[1] - self->last_pose_.vecPosition[1]) * refresh * dampening;
		if (yvel < minimum) yvel = 0;
		double zvel = abs(self->pose_.vecPosition[2] - self->last_pose_.vecPosition[2]) * refresh * dampening;
		if (zvel < minimum) zvel = 0;

		Eigen::Vector3d::Map(pose.vecVelocity) = Eigen::Vector3d(xvel, yvel, zvel);
		Eigen::Vector3d::Map(pose.vecAcceleration) = Eigen::Vector3d(xvel * permanent_acceleration_ratio, yvel * permanent_acceleration_ratio, zvel * permanent_acceleration_ratio);
	}
	else {
		Eigen::Vector3d::Map(pose.vecVelocity) = Eigen::Vector3d::Zero();
		Eigen::Vector3d::Map(pose.vecAcceleration) = Eigen::Vector3d::Zero();
	}


    // Orientation
    map(pose.qRotation) = osvr::util::fromQuat(report->pose.rotation);

    // Angular velocity and acceleration are not currently consistently provided
    Eigen::Vector3d::Map(pose.vecAngularVelocity) = Eigen::Vector3d::Zero();
    Eigen::Vector3d::Map(pose.vecAngularAcceleration) = Eigen::Vector3d::Zero();

    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;
    pose.willDriftInYaw = true;
    pose.shouldApplyHeadModel = false;
	pose.deviceIsConnected = true;

	self->last_pose_ = self->pose_;
	self->pose_ = pose;
    self->driver_host_->TrackedDevicePoseUpdated(self->device_id_, self->pose_); /// @fixme figure out ID correctly, don't hardcode to zero
}

void OSVRTrackedDeviceHandR::Button1Callback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed |= vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Touchpad, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed |= vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Touchpad, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::Button2Callback(void * userdata, const OSVR_TimeValue * timestamp, const OSVR_ButtonReport * report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Dashboard_Back);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_Dashboard_Back, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Dashboard_Back);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_Dashboard_Back, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::Button3Callback(void * userdata, const OSVR_TimeValue * timestamp, const OSVR_ButtonReport * report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_ApplicationMenu);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_ApplicationMenu, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_ApplicationMenu);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_ApplicationMenu, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::Button4Callback(void * userdata, const OSVR_TimeValue * timestamp, const OSVR_ButtonReport * report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_A);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_A, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_A);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_A, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::ButtonBumperCallback(void * userdata, const OSVR_TimeValue * timestamp, const OSVR_ButtonReport * report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Grip);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_Grip, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Grip);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_Grip, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::ButtonJoystickButtonCallback(void * userdata, const OSVR_TimeValue * timestamp, const OSVR_ButtonReport * report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Touchpad, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Touchpad, 0);
		break;
/*
	default:
		const std::string msg = "joy button state: " + std::to_string(report->state) + "\n";
		self->logger_->Log(msg.c_str());
*/
	}

}

void OSVRTrackedDeviceHandR::ButtonMiddleCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);

	switch (report->state) {
	case OSVR_BUTTON_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_System);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_System, 0);
		break;
	case OSVR_BUTTON_NOT_PRESSED:
		self->packetNumCounter++;
		self->state_.ulButtonPressed = self->state_.ulButtonPressed | vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_System);
		self->state_.unPacketNum = self->packetNumCounter;
		self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_System, 0);
		break;
	}
}

void OSVRTrackedDeviceHandR::AnalogJoystickXCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);
	self->state_.rAxis[0].x = report->state;
	self->driver_host_->TrackedDeviceAxisUpdated(self->device_id_, 0, self->state_.rAxis[0]);

	//const std::string msg = "analog x state: " + std::to_string(report->state) + "\n";
	//self->logger_->Log(msg.c_str());
}

void OSVRTrackedDeviceHandR::AnalogJoystickYCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);
	self->state_.rAxis[0].y = report->state;
	self->driver_host_->TrackedDeviceAxisUpdated(self->device_id_, 0, self->state_.rAxis[0]);

	//const std::string msg = "analog y state: " + std::to_string(report->state) + "\n";
	//self->logger_->Log(msg.c_str());
}

void OSVRTrackedDeviceHandR::AnalogTriggerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report)
{
	auto* self = static_cast<OSVRTrackedDeviceHandR*>(userdata);
	self->state_.rAxis[1].x = report->state;
	self->state_.rAxis[1].y = 0;
	self->driver_host_->TrackedDeviceAxisUpdated(self->device_id_, 1, self->state_.rAxis[1]);

	if (self->trigger_pressed_) {
		if (self->state_.rAxis[1].x < 0.90) {
			self->driver_host_->TrackedDeviceButtonUnpressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Trigger, 0);
			self->trigger_pressed_ = false;
		}
	}
	else {
		if (self->state_.rAxis[1].x > 0.90) {
			self->driver_host_->TrackedDeviceButtonPressed(self->device_id_, vr::EVRButtonId::k_EButton_SteamVR_Trigger, 0);
			self->trigger_pressed_ = true;
		}
	}

	//const std::string msg = "analog trigger state: " + std::to_string(report->state) + "\n";
	//self->logger_->Log(msg.c_str());
}

const char* OSVRTrackedDeviceHandR::GetId()
{
    /// @todo When available, return the actual unique ID of the HMD
    return "Right Hand Controller";
}

/** Gets the current state of a controller. */
vr::VRControllerState_t OSVRTrackedDeviceHandR::GetControllerState()
{
	logger_->Log("OSVRTrackedDeviceHandR::GetControllerState() R");
	return state_;
}

/** Returns a uint64 property. If the property is not available this function will return 0. */
bool OSVRTrackedDeviceHandR::TriggerHapticPulse(uint32_t unAxisId, uint16_t usPulseDurationMicroseconds)
{
	return 0;
}


