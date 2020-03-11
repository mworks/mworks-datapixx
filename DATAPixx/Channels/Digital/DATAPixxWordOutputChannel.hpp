//
//  DATAPixxWordOutputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/11/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxWordOutputChannel_hpp
#define DATAPixxWordOutputChannel_hpp

#include "DATAPixxDigitalOutputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxWordOutputChannel : public DATAPixxDigitalOutputChannel {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxWordOutputChannel(const ParameterValueMap &parameters);
    
    int getBitMask() const override;
    int getBitValue() const override;
    
private:
    std::vector<int> bitNumbers;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxWordOutputChannel_hpp */
