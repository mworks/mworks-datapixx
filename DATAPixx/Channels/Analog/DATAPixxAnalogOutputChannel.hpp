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
    
    explicit DATAPixxAnalogOutputChannel(const ParameterValueMap &parameters);
    
    void addNewValueNotification(const boost::shared_ptr<VariableNotification> &notification) {
        valueVar->addNotification(notification);
    }
    
    void setValueRange(double min, double max) {
        valueMin = min;
        valueMax = max;
    }
    
    double getValue() const;
    
    using DATAPixxAnalogChannel::setDeviceTimeNanos;
    
private:
    double valueMin;
    double valueMax;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxAnalogOutputChannel_hpp */
