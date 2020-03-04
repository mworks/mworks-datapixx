//
//  DATAPixxBitOutputChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxBitOutputChannel_hpp
#define DATAPixxBitOutputChannel_hpp

#include "DATAPixxDigitalOutputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxBitOutputChannel : public DATAPixxDigitalOutputChannel {
    
public:
    static const std::string BIT_NUMBER;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxBitOutputChannel(const ParameterValueMap &parameters);
    
    int getBitMask() const override;
    int getBitValue() const override;
    
private:
    const int bitNumber;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxBitOutputChannel_hpp */
