//
//  DATAPixxAnalogOutputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/18/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxAnalogOutputChannel.hpp"


BEGIN_NAMESPACE_MW


void DATAPixxAnalogOutputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxAnalogChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_analog_output");
}


DATAPixxAnalogOutputChannel::DATAPixxAnalogOutputChannel(const ParameterValueMap &parameters) :
    DATAPixxAnalogChannel(parameters),
    valueMin(0.0),
    valueMax(0.0)
{ }


double DATAPixxAnalogOutputChannel::getValue() const {
    auto value = valueVar->getValue().getFloat();
    if (value < valueMin || value > valueMax) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Value for DATAPixx analog output channel %d must be in the range [%g, %g]",
               getChannelNumber(),
               valueMin,
               valueMax);
        return 0.0;
    }
    return value;
}


END_NAMESPACE_MW
