//
//  DATAPixxDigitalChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDigitalChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxDigitalChannel::VALUE("value");


void DATAPixxDigitalChannel::describeComponent(ComponentInfo &info) {
    DATAPixxChannel::describeComponent(info);
    info.addParameter(VALUE);
}


DATAPixxDigitalChannel::DATAPixxDigitalChannel(const ParameterValueMap &parameters) :
    DATAPixxChannel(parameters),
    valueVar(parameters[VALUE])
{ }


void DATAPixxDigitalChannel::validateBitNumber(int bitNumber) {
    if (bitNumber < bitNumberMin || bitNumber > bitNumberMax) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, boost::format("Invalid bit number: %d") % bitNumber);
    }
}


END_NAMESPACE_MW
