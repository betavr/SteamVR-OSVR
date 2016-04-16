/** @file
    @brief OSVR server driver for OpenVR

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
#include "ServerDriver_OSVR.h"

#include "OSVRTrackedDeviceHandL.h" // for OSVRTrackedDeviceHandL
#include "OSVRTrackedDeviceHandR.h" // for OSVRTrackedDeviceHandR
#include "platform_fixes.h"         // strcasecmp
#include "make_unique.h"            // for std::make_unique

// Library/third-party includes
#include <openvr_driver.h>          // for everything in vr namespace

#include <osvr/ClientKit/Context.h> // for osvr::clientkit::ClientContext

// Standard includes
#include <vector>                   // for std::vector
#include <cstring>                  // for std::strcmp
#include <string>                   // for std::string, std::to_string

vr::EVRInitError ServerDriver_OSVR::Init(vr::IDriverLog* driver_log, vr::IServerDriverHost* driver_host, const char* user_driver_config_dir, const char* driver_install_dir)
{
    logger_ = driver_log;

	logger_->Log("ServerDriver_OSVR::Init() called.\n");
	logger_->Log("Razer Hydra support enabled\n");
	context_ = std::make_unique<osvr::clientkit::ClientContext>("com.osvr.SteamVR");

    //const std::string display_description = context_->getStringParameter("/display");
    //trackedDevices_.emplace_back(std::make_unique<OSVRTrackedDevice>(display_description, *(context_.get()), driver_host, logger_));

	const std::string left_hand_description = "left_hand_controller";
	trackedHandL_.emplace_back(std::make_unique<OSVRTrackedDeviceHandL>(left_hand_description, *(context_.get()), driver_host, logger_));

	const std::string right_hand_description = "right_hand_controller";
	trackedHandR_.emplace_back(std::make_unique<OSVRTrackedDeviceHandR>(right_hand_description, *(context_.get()), driver_host, logger_));

	return vr::VRInitError_None;
}

void ServerDriver_OSVR::Cleanup()
{
	trackedHandL_.clear();
	trackedHandR_.clear();
	context_.reset();
}

uint32_t ServerDriver_OSVR::GetTrackedDeviceCount()
{
    std::string msg = "ServerDriver_OSVR::GetTrackedDeviceCount(): Detected " + std::to_string(trackedHandL_.size() + trackedHandR_.size()) + " tracked devices.\n";
    logger_->Log(msg.c_str());
    return trackedHandL_.size() + trackedHandR_.size();
}

vr::ITrackedDeviceServerDriver* ServerDriver_OSVR::GetTrackedDeviceDriver(uint32_t index, const char* interface_version)
{
	std::string msg;

	msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): index " + std::to_string(index) + " requested.\n";
	logger_->Log(msg.c_str());
	
	if (0 != strcasecmp(interface_version, vr::ITrackedDeviceServerDriver_Version)) {
        std::string msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): ERROR: Incompatible SteamVR version!\n";
        logger_->Log(msg.c_str());
        return NULL;
    }


	switch (index) {
		//case 0:
			//msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): Returning tracked hmd .\n";
			//logger_->Log(msg.c_str());
			//return NULL;
		case 1:
			msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): Returning tracked left hand .\n";
			logger_->Log(msg.c_str());
			return trackedHandL_[0].get();
		case 0:
			msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): Returning tracked right hand .\n";
			logger_->Log(msg.c_str());
			return trackedHandR_[0].get();
		default:
			msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): ERROR: Index " + std::to_string(index) + " is out of range .\n";
			logger_->Log(msg.c_str());
			break;
	}

	/*
	if (index >= trackedDevices_.size()) {
        std::string msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): ERROR: Index " + std::to_string(index) + " is out of range [0.." + std::to_string(trackedDevices_.size()) + "].\n";
        logger_->Log(msg.c_str());
        return NULL;
	}
	else {
		std::string msg = "ServerDriver_OSVR::GetTrackedDeviceDriver(): Returning tracked device " + std::to_string(index) + ".\n";
		logger_->Log(msg.c_str());
		return trackedDevices_[index].get();
	}
	*/

	return NULL;
}

vr::ITrackedDeviceServerDriver* ServerDriver_OSVR::FindTrackedDeviceDriver(const char* id, const char* interface_version)
{
    if (0 != strcasecmp(interface_version, vr::ITrackedDeviceServerDriver_Version)) {
        std::string msg = "ServerDriver_OSVR::FindTrackedDeviceDriver(): ERROR: Incompatible SteamVR version!\n";
        logger_->Log(msg.c_str());
        return NULL;
    }

	for (auto& tracked_device : trackedHandL_) {
		if (0 == std::strcmp(id, tracked_device->GetId())) {
			std::string msg = "ServerDriver_OSVR::FindTrackedDeviceDriver(): Returning tracked device " + std::string(id) + ".\n";
			logger_->Log(msg.c_str());
			return tracked_device.get();
		}
	}

	for (auto& tracked_device : trackedHandR_) {
		if (0 == std::strcmp(id, tracked_device->GetId())) {
			std::string msg = "ServerDriver_OSVR::FindTrackedDeviceDriver(): Returning tracked device " + std::string(id) + ".\n";
			logger_->Log(msg.c_str());
			return tracked_device.get();
		}
	}

	std::string msg = "ServerDriver_OSVR::FindTrackedDeviceDriver(): ERROR: Failed to locate device named '" + std::string(id) + "'.\n";
    logger_->Log(msg.c_str());

    return NULL;
}

void ServerDriver_OSVR::RunFrame()
{
    context_->update();
}

bool ServerDriver_OSVR::ShouldBlockStandbyMode()
{
    return false;
}

void ServerDriver_OSVR::EnterStandby()
{
    // TODO
}

void ServerDriver_OSVR::LeaveStandby()
{
    // TODO
}

