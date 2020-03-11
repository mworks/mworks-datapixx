//
//  DATAPixxBitInputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/5/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxBitInputChannel_hpp
#define DATAPixxBitInputChannel_hpp

#include "DATAPixxDigitalInputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxBitInputChannel : public DATAPixxDigitalInputChannel {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxBitInputChannel(const ParameterValueMap &parameters);
    
    void setBitValue(int bitValue, MWTime deviceTimeNanos, MWTime time) override;
    
private:
    const int bitNumber;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxBitInputChannel_hpp */
