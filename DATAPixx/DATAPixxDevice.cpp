//
//  DATAPixxDevice.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/26/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDevice.hpp"


BEGIN_NAMESPACE_MW


void DATAPixxDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/datapixx");
}


DATAPixxDevice::DATAPixxDevice(const ParameterValueMap &parameters) :
    IODevice(parameters)
{
}


DATAPixxDevice::~DATAPixxDevice() {
}


void DATAPixxDevice::addChild(std::map<std::string, std::string> parameters,
                      ComponentRegistryPtr reg,
                      boost::shared_ptr<Component> child)
{
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid channel type for DATAPixx device");
}


bool DATAPixxDevice::initialize() {
    return true;
}


bool DATAPixxDevice::startDeviceIO() {
    return true;
}


bool DATAPixxDevice::stopDeviceIO() {
    return true;
}


END_NAMESPACE_MW
