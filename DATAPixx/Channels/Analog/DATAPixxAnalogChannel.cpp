//
//  DATAPixxAnalogChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/18/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxAnalogChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxAnalogChannel::CHANNEL_NUMBER("channel_number");


void DATAPixxAnalogChannel::describeComponent(ComponentInfo &info) {
    DATAPixxChannel::describeComponent(info);
    info.addParameter(CHANNEL_NUMBER);
}


DATAPixxAnalogChannel::DATAPixxAnalogChannel(const ParameterValueMap &parameters) :
    DATAPixxChannel(parameters),
    channelNumber(parameters[CHANNEL_NUMBER]),
    voltageMin(0.0),
    voltageMax(0.0)
{
    if (channelNumber < 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, boost::format("Invalid channel number: %d") % channelNumber);
    }
}


END_NAMESPACE_MW
