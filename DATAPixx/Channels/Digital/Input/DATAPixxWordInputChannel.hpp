//
//  DATAPixxWordInputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/11/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxWordInputChannel_hpp
#define DATAPixxWordInputChannel_hpp

#include "DATAPixxDigitalInputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxWordInputChannel : public DATAPixxDigitalInputChannel {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxWordInputChannel(const ParameterValueMap &parameters);
    
    std::set<int> getBitNumbers() const override;
    void setBitValue(int bitValue, MWTime deviceTimeNanos, MWTime time) const override;
    
private:
    std::vector<int> bitNumbers;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxWordInputChannel_hpp */
