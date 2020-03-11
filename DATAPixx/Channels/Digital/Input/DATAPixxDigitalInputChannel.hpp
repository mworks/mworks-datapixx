//
//  DATAPixxDigitalInputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/5/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxDigitalInputChannel_hpp
#define DATAPixxDigitalInputChannel_hpp

#include "DATAPixxDigitalChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxDigitalInputChannel : public DATAPixxDigitalChannel {
    
public:
    using DATAPixxDigitalChannel::DATAPixxDigitalChannel;
    
    virtual void setBitValue(int bitValue, MWTime deviceTimeNanos, MWTime time) = 0;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDigitalInputChannel_hpp */
