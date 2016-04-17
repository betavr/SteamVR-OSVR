/** @file
    @brief OSVR tracked device controller left hand

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

#ifndef INCLUDED_OSVRTrackedDeviceHandL_h_GUID_128E3B29_F5FC_4221_9B38_14E3F402E644
#define INCLUDED_OSVRTrackedDeviceHandL_h_GUID_128E3B29_F5FC_4221_9B38_14E3F402E644

// Internal Includes
#include "osvr_compiler_detection.h"    // for OSVR_OVERRIDE

// OpenVR includes
#include <openvr_driver.h>

// Library/third-party includes
#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Interface.h> 

// Standard includes
#include <string>

class OSVRTrackedDeviceHandL : public vr::ITrackedDeviceServerDriver, public vr::IVRControllerComponent {
friend class ServerDriver_OSVR;
public:
    OSVRTrackedDeviceHandL(const std::string& display_description, osvr::clientkit::ClientContext& context, vr::IServerDriverHost* driver_host, vr::IDriverLog* driver_log = nullptr);

    virtual ~OSVRTrackedDeviceHandL();
    // ------------------------------------
    // Management Methods
    // ------------------------------------
    /**
     * This is called before an HMD is returned to the application. It will
     * always be called before any display or tracking methods. Memory and
     * processor use by the ITrackedDeviceServerDriver object should be kept to
     * a minimum until it is activated.  The pose listener is guaranteed to be
     * valid until Deactivate is called, but should not be used after that
     * point.
     */
    virtual vr::EVRInitError Activate(uint32_t object_id) OSVR_OVERRIDE;

    /**
     * This is called when The VR system is switching from this Hmd being the
     * active display to another Hmd being the active display. The driver should
     * clean whatever memory and thread use it can when it is deactivated.
     */
    virtual void Deactivate() OSVR_OVERRIDE;

    /**
     * Handles a request from the system to power off this device.
     */
    virtual void PowerOff() OSVR_OVERRIDE;

    /**
     * Requests a component interface of the driver for device-specific
     * functionality. The driver should return NULL if the requested interface
     * or version is not supported.
     */
    virtual void* GetComponent(const char* component_name_and_version) OSVR_OVERRIDE;

    /**
     * A VR Client has made this debug request of the driver. The set of valid
     * requests is entirely up to the driver and the client to figure out, as is
     * the format of the response. Responses that exceed the length of the
     * supplied buffer should be truncated and null terminated.
     */
    virtual void DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size) OSVR_OVERRIDE;


	// ------------------------------------
	// Controller Methods
	// ------------------------------------

	/** Gets the current state of a controller. */
	virtual vr::VRControllerState_t GetControllerState() OSVR_OVERRIDE;

	/** Returns a uint64 property. If the property is not available this function will return 0. */
	virtual bool TriggerHapticPulse(uint32_t unAxisId, uint16_t usPulseDurationMicroseconds) OSVR_OVERRIDE;

	
	// ------------------------------------
    // Tracking Methods
    // ------------------------------------
    virtual vr::DriverPose_t GetPose() OSVR_OVERRIDE;

    // ------------------------------------
    // Property Methods
    // ------------------------------------

    /**
     * Returns a bool property. If the property is not available this function
     * will return false.
     */
    virtual bool GetBoolTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

    /**
     * Returns a float property. If the property is not available this function
     * will return 0.
     */
    virtual float GetFloatTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

    /**
     * Returns an int property. If the property is not available this function
     * will return 0.
     */
    virtual int32_t GetInt32TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

    /**
     * Returns a uint64 property. If the property is not available this function
     * will return 0.
     */
    virtual uint64_t GetUint64TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

    /**
     * Returns a matrix property. If the device index is not valid or the
     * property is not a matrix type, this function will return identity.
     */
    virtual vr::HmdMatrix34_t GetMatrix34TrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

    /**
     * Returns a string property. If the property is not available this function
     * will return 0 and @p error will be set to an error. Otherwise it returns
     * the length of the number of bytes necessary to hold this string including
     * the trailing null. If the buffer is too small the error will be
     * @c TrackedProp_BufferTooSmall. Strings will generally fit in buffers of
     * @c k_unTrackingStringSize characters. Drivers may not return strings longer
     * than @c k_unMaxPropertyStringSize.
     */
    virtual uint32_t GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, char* value, uint32_t buffer_size, vr::ETrackedPropertyError* error) OSVR_OVERRIDE;

protected:
    const char* GetId();


private:
    std::string GetStringTrackedDeviceProperty(vr::ETrackedDeviceProperty prop, vr::ETrackedPropertyError *error);

    /**
     * Callback function which is called whenever new data has been received
     * from the tracker.
     */
    static void HandLTrackerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_PoseReport* report);

	static void ButtonMiddleCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void Button1Callback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void Button2Callback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void Button3Callback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void Button4Callback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void ButtonBumperCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);
	static void ButtonJoystickButtonCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_ButtonReport* report);

	static void AnalogJoystickXCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report);
	static void AnalogJoystickYCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report);
	static void AnalogTriggerCallback(void* userdata, const OSVR_TimeValue* timestamp, const OSVR_AnalogReport* report);

    const std::string m_DisplayDescription;
    osvr::clientkit::ClientContext& m_Context;
    vr::IDriverLog* logger_ = nullptr;
    vr::IServerDriverHost* driver_host_ = nullptr;

	osvr::clientkit::Interface m_TrackerInterface;

	osvr::clientkit::Interface m_ButtonInterfaceMiddle;
	osvr::clientkit::Interface m_ButtonInterface1;
	osvr::clientkit::Interface m_ButtonInterface2;
	osvr::clientkit::Interface m_ButtonInterface3;
	osvr::clientkit::Interface m_ButtonInterface4;
	osvr::clientkit::Interface m_ButtonInterfaceBumper;
	osvr::clientkit::Interface m_ButtonInterfaceJoystickButton;

	osvr::clientkit::Interface m_AnalogInterfaceJoystickX;
	osvr::clientkit::Interface m_AnalogInterfaceJoystickY;
	osvr::clientkit::Interface m_AnalogInterfaceTrigger;

	vr::DriverPose_t pose_;
    vr::ETrackedDeviceClass deviceClass_;
	vr::VRControllerState_t state_;
	vr::IVRSettings *settings_;

	uint32_t device_id_;
	float leftHandVecWorldFromDriverTranslationX_;
	float leftHandVecWorldFromDriverTranslationY_;
	float leftHandVecWorldFromDriverTranslationZ_;

	char * handdevice = "/controller/left";
	char * buttondevice_SteamVR_Trigger = "/controller/left/1";
	char * buttondevice_Dashboard_Back = "/controller/left/2";
	char * buttondevice_ApplicationMenu = "/controller/left/3";
	char * buttondevice_A  = "/controller/left/4";
	char * buttondevice_Grip = "/controller/left/bumper"; 
	char * buttondevice_SteamVR_Touchpad = "/controller/left/joystick/button"; 
	char * buttondevice_System = "/controller/left/middle";
	char * analogdevice_JoystickX = "/controller/left/joystick/x";
	char * analogdevice_JoystickY = "/controller/left/joystick/y";
	char * analogdevice_Trigger = "/controller/left/trigger";

	uint32_t packetNumCounter;

	const bool logDebugProps = false;
};

#endif // INCLUDED_OSVRTrackedDeviceHandL_h_GUID_128E3B29_F5FC_4221_9B38_14E3F402E644

