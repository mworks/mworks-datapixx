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


int DATAPixxAnalogOutputChannel::getValue() const {
    const auto voltage = valueVar->getValue().getFloat();
    if (voltage < voltageMin || voltage > voltageMax) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Value for DATAPixx analog output channel %d must be in the range [%g, %g]",
               channelNumber,
               voltageMin,
               voltageMax);
        return 0;
    }
    const auto normVoltage = (voltage - voltageMin) / (voltageMax - voltageMin) - 0.5;  // Normalize to [-0.5, 0.5]
    const auto value = int(std::floor(normVoltage * 65536.0 + 0.5));  // Map to [-32768, 32768] with rounding (not truncation)
    return std::min(value, 32767);  // Maximum 16-bit, 2's complement, signed value is 32767
}


END_NAMESPACE_MW
