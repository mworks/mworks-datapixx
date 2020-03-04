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
    using DATAPixxDigitalChannel::DATAPixxDigitalChannel;
    
    void addNewValueNotification(const boost::shared_ptr<VariableNotification> &notification) {
        valueVar->addNotification(notification);
    }
    
    virtual int getBitMask() const = 0;
    virtual int getBitValue() const = 0;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDigitalOutputChannel_hpp */
