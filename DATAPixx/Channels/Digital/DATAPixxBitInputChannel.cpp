//
//  DATAPixxBitInputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/5/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxBitInputChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxBitInputChannel::BIT_NUMBER("bit_number");


void DATAPixxBitInputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxDigitalInputChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_bit_input");
    info.addParameter(BIT_NUMBER);
}


DATAPixxBitInputChannel::DATAPixxBitInputChannel(const ParameterValueMap &parameters) :
    DATAPixxDigitalInputChannel(parameters),
    bitNumber(parameters[BIT_NUMBER])
{
    validateBitNumber(bitNumber);
}


void DATAPixxBitInputChannel::setBitValue(int bitValue) {
    const auto value = bool(bitValue & (1 << bitNumber));
    if (valueVar->getValue().getBool() != value) {
        valueVar->setValue(Datum(value));
    }
}


END_NAMESPACE_MW
