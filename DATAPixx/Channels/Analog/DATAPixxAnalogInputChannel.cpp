//
//  DATAPixxAnalogInputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/20/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxAnalogInputChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxAnalogInputChannel::REFERENCE_SOURCE("reference_source");


void DATAPixxAnalogInputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxAnalogChannel::describeComponent(info);
    info.setSignature("iochannel/datapixx_analog_input");
    info.addParameter(REFERENCE_SOURCE, "GND");
}


template<>
DATAPixxAnalogInputChannel::ReferenceSource ParameterValue::convert(const std::string &s, ComponentRegistryPtr reg) {
    if (s == "GND") {
        return DATAPixxAnalogInputChannel::ReferenceSource::GND;
    } else if (s == "DIFF") {
        return DATAPixxAnalogInputChannel::ReferenceSource::DIFF;
    } else if (s == "REF0") {
        return DATAPixxAnalogInputChannel::ReferenceSource::REF0;
    } else if (s == "REF1") {
        return DATAPixxAnalogInputChannel::ReferenceSource::REF1;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, boost::format("Invalid reference source: \"%s\"") % s);
}


DATAPixxAnalogInputChannel::DATAPixxAnalogInputChannel(const ParameterValueMap &parameters) :
    DATAPixxAnalogChannel(parameters),
    referenceSource(parameters[REFERENCE_SOURCE])
{ }


void DATAPixxAnalogInputChannel::setValue(int value, MWTime deviceTimeNanos, MWTime time) const {
    const auto voltage = (double(value) + 32768.0) / 65536.0 * (voltageMax - voltageMin) + voltageMin;
    valueVar->setValue(Datum(voltage), time);
    setDeviceTimeNanos(deviceTimeNanos, time);
}


END_NAMESPACE_MW
