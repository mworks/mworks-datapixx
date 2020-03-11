//
//  DATAPixxWordOutputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/11/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxWordOutputChannel.hpp"


BEGIN_NAMESPACE_MW


void DATAPixxWordOutputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxDigitalOutputChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_word_output");
    info.addParameter(BIT_NUMBERS);
}


DATAPixxWordOutputChannel::DATAPixxWordOutputChannel(const ParameterValueMap &parameters) :
    DATAPixxDigitalOutputChannel(parameters)
{
    evaluateBitNumbers(parameters[BIT_NUMBERS].str(), bitNumbers);
}


int DATAPixxWordOutputChannel::getBitMask() const {
    int bitMask = 0;
    for (auto bitNumber : bitNumbers) {
        bitMask |= (1 << bitNumber);
    }
    return bitMask;
}


int DATAPixxWordOutputChannel::getBitValue() const {
    const auto wordValue = valueVar->getValue().getInteger();
    int bitValue = 0;
    for (std::size_t i = 0; i < bitNumbers.size(); i++) {
        const auto bitNumber = bitNumbers.at(i);
        bitValue |= (bool(wordValue & (1 << i)) << bitNumber);
    }
    return bitValue;
}


END_NAMESPACE_MW
