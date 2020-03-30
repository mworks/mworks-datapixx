//
//  DATAPixxDigitalOutputChannel.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDigitalOutputChannel.hpp"


BEGIN_NAMESPACE_MW


const std::string DATAPixxDigitalOutputChannel::USE_INPUT_PORT("use_input_port");


void DATAPixxDigitalOutputChannel::describeComponent(ComponentInfo &info) {
    DATAPixxDigitalChannel::describeComponent(info);
    info.addParameter(USE_INPUT_PORT, "NO");
}


DATAPixxDigitalOutputChannel::DATAPixxDigitalOutputChannel(const ParameterValueMap &parameters) :
    DATAPixxDigitalChannel(parameters),
    useInputPort(parameters[USE_INPUT_PORT])
{ }


END_NAMESPACE_MW
