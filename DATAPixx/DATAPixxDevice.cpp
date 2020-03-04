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
    const auto error = DPxGetError();
    if (error == DPX_SUCCESS) {
        return false;
    }
    merror(M_IODEVICE_MESSAGE_DOMAIN, "%s: %s (error = %d)", msg, DPxGetErrorString(), error);
    DPxClearError();
    return true;
}


inline bool logConfigurationFailure() {
    DPxUpdateRegCache();
    return logError("Cannot update DATAPixx configuration");
}


int findFirstCommonBit(int mask1, int mask2) {
    auto commonBits = mask1 & mask2;
    if (!commonBits) {
        return -1;
    }
    int index = 0;
    while (!(commonBits & 1)) {
        commonBits >>= 1;
        index++;
    }
    return index;
}


END_NAMESPACE()


void DATAPixxDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/datapixx");
}


DATAPixxDevice::DATAPixxDevice(const ParameterValueMap &parameters) :
    IODevice(parameters),
    digitalOutputBitMask(0),
    running(false)
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
    if (auto channel = boost::dynamic_pointer_cast<DATAPixxDigitalOutputChannel>(child)) {
        digitalOutputChannels.push_back(channel);
        return;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid channel type for DATAPixx device");
}


bool DATAPixxDevice::initialize() {
    lock_guard lock(mutex);
    
    if ((DPxOpen(), logError("Cannot open DATAPixx device"))) {
        return false;
    }
    
    mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Opened DATAPixx device (ID = %d)", DPxGetID());
    
    if ((DPxStopAllScheds(), logError("Cannot stop existing DATAPixx schedules")) ||
        logConfigurationFailure())
    {
        return false;
    }
    
    if (haveDigitalOutputs() && !configureDigitalOutputs()) {
        return false;
    }
    
    // Commit all configuration changes
    if (logConfigurationFailure()) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDeviceIO() {
    lock_guard lock(mutex);
    
    if (!running) {
        if (haveDigitalOutputs() && !startDigitalOutputs()) {
            return false;
        }
        if (logConfigurationFailure()) {
            return false;
        }
        running = true;
    }
    
    return true;
}


bool DATAPixxDevice::stopDeviceIO() {
    lock_guard lock(mutex);
    
    if (running) {
        if (haveDigitalOutputs() && !stopDigitalOutputs()) {
            return false;
        }
        if (logConfigurationFailure()) {
            return false;
        }
        running = false;
    }
    
    return true;
}


bool DATAPixxDevice::configureDigitalOutputs() {
    boost::weak_ptr<DATAPixxDevice> weakThis(component_shared_from_this<DATAPixxDevice>());
    
    for (auto &channel : digitalOutputChannels) {
        const auto bitMask = channel->getBitMask();
        const auto duplicateBitNumber = findFirstCommonBit(digitalOutputBitMask, bitMask);
        if (-1 != duplicateBitNumber) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Digital output bit %d is already in use") % duplicateBitNumber);
        }
        digitalOutputBitMask |= bitMask;
        
        // It's OK to capture channel by reference, because it will remain alive (in
        // digitalOutputChannels) for as long as the device is alive
        auto callback = [weakThis, &channel, bitMask](const Datum &data, MWTime time) {
            if (auto sharedThis = weakThis.lock()) {
                lock_guard lock(sharedThis->mutex);
                if (sharedThis->running) {
                    auto bitValue = channel->getBitValue();
                    DPxSetDoutValue(bitValue, bitMask);
                    if (!logError("Cannot set DATAPixx digital output")) {
                        logConfigurationFailure();
                    }
                }
            }
        };
        channel->addNewValueNotification(boost::make_shared<VariableCallbackNotification>(callback));
    }
    
    // Initialize all configured digital output bits to zero
    if (!stopDigitalOutputs()) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDigitalOutputs() {
    int bitValue = 0;
    
    for (auto &channel : digitalOutputChannels) {
        bitValue |= channel->getBitValue();
    }
    
    if (DPxSetDoutValue(bitValue, digitalOutputBitMask), logError("Cannot initialize DATAPixx digital outputs")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::stopDigitalOutputs() {
    if (DPxSetDoutValue(0, digitalOutputBitMask), logError("Cannot reset DATAPixx digital outputs")) {
        return false;
    }
    return true;
}


std::atomic_flag DATAPixxDevice::deviceExists = ATOMIC_FLAG_INIT;


END_NAMESPACE_MW
