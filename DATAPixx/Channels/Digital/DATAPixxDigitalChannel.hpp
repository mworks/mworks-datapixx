//
//  DATAPixxDigitalChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxDigitalChannel_hpp
#define DATAPixxDigitalChannel_hpp

#include "DATAPixxChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxDigitalChannel : public DATAPixxChannel {
    
public:
    static const std::string BIT_NUMBER;
    static const std::string BIT_NUMBERS;
    
    using DATAPixxChannel::DATAPixxChannel;
    
protected:
    static constexpr int bitNumberMin = 0;
    static constexpr int bitNumberMax = 23;
    
    static void validateBitNumber(int bitNumber);
    static void evaluateBitNumbers(const std::string &bitNumbersExpr, std::vector<int> &bitNumbers);
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDigitalChannel_hpp */
