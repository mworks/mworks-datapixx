//
//  DATAPixxAnalogOutputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/18/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxAnalogOutputChannel_hpp
#define DATAPixxAnalogOutputChannel_hpp

#include "DATAPixxAnalogChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxAnalogOutputChannel : public DATAPixxAnalogChannel {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using DATAPixxAnalogChannel::DATAPixxAnalogChannel;
    
    int getValue() const;
    
    using DATAPixxAnalogChannel::addNewValueNotification;
    using DATAPixxAnalogChannel::setDeviceTimeNanos;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxAnalogOutputChannel_hpp */
