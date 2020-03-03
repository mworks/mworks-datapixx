//
//  DATAPixxDevice.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/26/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDevice.hpp"

#include "libdpx.h"


BEGIN_NAMESPACE_MW


BEGIN_NAMESPACE()


inline bool logError(const char *msg) {
    if (DPxGetError() == DPX_SUCCESS) {
        return false;
    }
    merror(M_IODEVICE_MESSAGE_DOMAIN, "%s: %s (error = %d)", msg, DPxGetErrorString(), DPxGetError());
    DPxClearError();
    return true;
}


END_NAMESPACE()


void DATAPixxDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/datapixx");
}


DATAPixxDevice::DATAPixxDevice(const ParameterValueMap &parameters) :
    IODevice(parameters)
{
    if (deviceExists.test_and_set()) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Experiment can contain at most one DATAPixx device");
    }
}


DATAPixxDevice::~DATAPixxDevice() {
    DPxClose();
    logError("Cannot close DATAPixx device");
    deviceExists.clear();
}


void DATAPixxDevice::addChild(std::map<std::string, std::string> parameters,
                      ComponentRegistryPtr reg,
                      boost::shared_ptr<Component> child)
{
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid channel type for DATAPixx device");
}


bool DATAPixxDevice::initialize() {
    if ((DPxOpen(), logError("Cannot open DATAPixx device"))) {
        return false;
    }
    
    mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Opened DATAPixx device (ID = %d)", DPxGetID());
    
    if ((DPxStopAllScheds(), logError("Cannot stop existing DATAPixx schedules")) ||
        (DPxUpdateRegCache(), logError("Cannot update DATAPixx configuration")))
    {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDeviceIO() {
    return true;
}


bool DATAPixxDevice::stopDeviceIO() {
    return true;
}


std::atomic_flag DATAPixxDevice::deviceExists = ATOMIC_FLAG_INIT;


END_NAMESPACE_MW
