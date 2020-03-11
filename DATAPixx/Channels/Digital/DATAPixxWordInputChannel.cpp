//
//  DATAPixxWordInputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/11/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxWordInputChannel.hpp"


BEGIN_NAMESPACE_MW


void DATAPixxWordInputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxDigitalInputChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_word_input");
    info.addParameter(BIT_NUMBERS);
}


DATAPixxWordInputChannel::DATAPixxWordInputChannel(const ParameterValueMap &parameters) :
    DATAPixxDigitalInputChannel(parameters)
{
    evaluateBitNumbers(parameters[BIT_NUMBERS].str(), bitNumbers);
}


void DATAPixxWordInputChannel::setBitValue(int bitValue, MWTime deviceTimeNanos, MWTime time) {
    long long wordValue = 0;
    for (std::size_t i = 0; i < bitNumbers.size(); i++) {
        const auto bitNumber = bitNumbers.at(i);
        wordValue |= (bool(bitValue & (1 << bitNumber)) << i);
    }
    if (valueVar->getValue().getInteger() != wordValue) {
        valueVar->setValue(Datum(wordValue), time);
        setDeviceTimeNanos(deviceTimeNanos, time);
    }
}


END_NAMESPACE_MW
