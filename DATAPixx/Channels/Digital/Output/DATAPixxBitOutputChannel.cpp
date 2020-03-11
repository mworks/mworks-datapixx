//
//  DATAPixxBitOutputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxBitOutputChannel.hpp"


BEGIN_NAMESPACE_MW


void DATAPixxBitOutputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxDigitalOutputChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_bit_output");
    info.addParameter(BIT_NUMBER);
}


DATAPixxBitOutputChannel::DATAPixxBitOutputChannel(const ParameterValueMap &parameters) :
    DATAPixxDigitalOutputChannel(parameters),
    bitNumber(parameters[BIT_NUMBER])
{
    validateBitNumber(bitNumber);
}


int DATAPixxBitOutputChannel::getBitMask() const {
    return (1 << bitNumber);
}


int DATAPixxBitOutputChannel::getBitValue() const {
    return (valueVar->getValue().getBool() << bitNumber);
}


END_NAMESPACE_MW
