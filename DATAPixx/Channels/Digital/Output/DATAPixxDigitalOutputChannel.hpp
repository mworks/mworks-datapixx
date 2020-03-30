//
//  DATAPixxDigitalOutputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxDigitalOutputChannel_hpp
#define DATAPixxDigitalOutputChannel_hpp

#include "DATAPixxDigitalChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxDigitalOutputChannel : public DATAPixxDigitalChannel {
    
public:
    static const std::string USE_INPUT_PORT;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxDigitalOutputChannel(const ParameterValueMap &parameters);
    
    bool shouldUseInputPort() const { return useInputPort; }
    virtual int getBitValue() const = 0;
    
    using DATAPixxDigitalChannel::addNewValueNotification;
    using DATAPixxDigitalChannel::setDeviceTimeNanos;
    
private:
    const bool useInputPort;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDigitalOutputChannel_hpp */
