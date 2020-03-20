//
//  DATAPixxAnalogInputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/20/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxAnalogInputChannel_hpp
#define DATAPixxAnalogInputChannel_hpp

#include "DATAPixxAnalogChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxAnalogInputChannel : public DATAPixxAnalogChannel {
    
public:
    enum class ReferenceSource { GND, DIFF, REF0, REF1 };
    
    static const std::string REFERENCE_SOURCE;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxAnalogInputChannel(const ParameterValueMap &parameters);
    
    ReferenceSource getReferenceSource() const { return referenceSource; }
    
    void setValue(int value, MWTime deviceTimeNanos, MWTime time) const;
    
private:
    const ReferenceSource referenceSource;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxAnalogInputChannel_hpp */
