//
//  DATAPixxChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxChannel::DEVICE_TIME_NANOS("device_time_nanos");


void DATAPixxChannel::describeComponent(ComponentInfo &info) {
    Component::describeComponent(info);
    info.addParameter(DEVICE_TIME_NANOS, false);
}


DATAPixxChannel::DATAPixxChannel(const ParameterValueMap &parameters) :
    Component(parameters),
    deviceTimeNanosVar(optionalVariable(parameters[DEVICE_TIME_NANOS]))
{ }


END_NAMESPACE_MW
