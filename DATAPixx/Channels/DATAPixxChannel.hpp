//
//  DATAPixxChannel.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 3/4/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxChannel_hpp
#define DATAPixxChannel_hpp


BEGIN_NAMESPACE_MW


class DATAPixxChannel : public Component {
    
public:
    static const std::string VALUE;
    static const std::string DEVICE_TIME_NANOS;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxChannel(const ParameterValueMap &parameters);
    
protected:
    void setDeviceTimeNanos(MWTime deviceTimeNanos, MWTime time) {
        if (deviceTimeNanosVar) {
            deviceTimeNanosVar->setValue(Datum(deviceTimeNanos), time);
        }
    }
    
    const VariablePtr valueVar;
    
private:
    const VariablePtr deviceTimeNanosVar;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxChannel_hpp */
