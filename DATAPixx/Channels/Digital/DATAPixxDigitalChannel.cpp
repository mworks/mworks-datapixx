//
//  DATAPixxDigitalChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDigitalChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxDigitalChannel::BIT_NUMBER("bit_number");
const std::string DATAPixxDigitalChannel::BIT_NUMBERS("bit_numbers");
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


void DATAPixxDigitalChannel::evaluateBitNumbers(const std::string &bitNumbersExpr, std::vector<int> &bitNumbers) {
    std::vector<Datum> bitNumbersValues;
    ParsedExpressionVariable::evaluateExpressionList(bitNumbersExpr, bitNumbersValues);
    for (auto &bitNumberValue : bitNumbersValues) {
        int bitNumber = bitNumberValue.getInteger();
        validateBitNumber(bitNumber);
        if (std::find(bitNumbers.begin(), bitNumbers.end(), bitNumber) != bitNumbers.end()) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Bit %d appears multiple times in bit number list") % bitNumber);
        }
        bitNumbers.push_back(bitNumber);
    }
}


END_NAMESPACE_MW
