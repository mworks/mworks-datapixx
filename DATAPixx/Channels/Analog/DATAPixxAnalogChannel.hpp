//
//  DATAPixxAnalogChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/18/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxAnalogChannel_hpp
#define DATAPixxAnalogChannel_hpp

#include "DATAPixxChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxAnalogChannel : public DATAPixxChannel {
    
public:
    static const std::string CHANNEL_NUMBER;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxAnalogChannel(const ParameterValueMap &parameters);
    
    int getChannelNumber() const { return channelNumber; }
    
private:
    const int channelNumber;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxAnalogChannel_hpp */
