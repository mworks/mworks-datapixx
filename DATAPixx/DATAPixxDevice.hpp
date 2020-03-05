//
//  DATAPixxDevice.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/26/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxDevice_hpp
#define DATAPixxDevice_hpp

#include "DATAPixxDigitalInputChannel.hpp"
#include "DATAPixxDigitalOutputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxDevice : public IODevice, boost::noncopyable {
    
public:
    static const std::string UPDATE_INTERVAL;
    static const std::string ENABLE_DOUT_DIN_LOOPBACK;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxDevice(const ParameterValueMap &parameters);
    ~DATAPixxDevice();
    
    void addChild(std::map<std::string, std::string> parameters,
                  ComponentRegistryPtr reg,
                  boost::shared_ptr<Component> child) override;
    
    bool initialize() override;
    bool startDeviceIO() override;
    bool stopDeviceIO() override;
    
private:
    bool haveDigitalInputs() const { return !(digitalInputChannels.empty()); }
    bool configureDigitalInputs();
    bool startDigitalInputs();
    
    bool haveDigitalOutputs() const { return !(digitalOutputChannels.empty()); }
    bool configureDigitalOutputs();
    bool startDigitalOutputs();
    bool stopDigitalOutputs();
    
    bool haveInputs() const { return haveDigitalInputs(); }
    void startReadInputsTask();
    void stopReadInputsTask();
    void readInputs();
    
    static std::atomic_flag deviceExists;
    
    const MWTime updateInterval;
    const VariablePtr enableDigitalLoopback;
    
    using lock_guard = std::lock_guard<std::recursive_mutex>;
    lock_guard::mutex_type mutex;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalInputChannel>> digitalInputChannels;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalOutputChannel>> digitalOutputChannels;
    int digitalOutputBitMask;
    
    boost::shared_ptr<ScheduleTask> readInputsTask;
    
    bool running;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDevice_hpp */
