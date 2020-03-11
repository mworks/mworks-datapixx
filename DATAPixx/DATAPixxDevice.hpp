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
    static const std::string CLOCK_OFFSET_NANOS;
    static const std::string ENABLE_DOUT_PIXEL_MODE;
    static const std::string ENABLE_DOUT_VSYNC_MODE;
    static const std::string ENABLE_DIN_STABILIZE;
    static const std::string ENABLE_DIN_DEBOUNCE;
    static const std::string ENABLE_DOUT_DIN_LOOPBACK;
    static const std::string DIN_EVENT_BUFFER_SIZE;
    
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
    struct DigitalInputEvent {
        boost::endian::little_uint64_t deviceTimeNanos;
        boost::endian::little_uint16_t bitValue;  // Bits 0-15 only
    };
    
    static constexpr int digitalOutputVSYNCBitNumber = 23;
    static constexpr MWTime clockSyncUpdateInterval = 1000000;  // One second
    
    bool configureDevice();
    
    bool haveDigitalInputs() const { return !(digitalInputChannels.empty()); }
    bool configureDigitalInputs();
    bool startDigitalInputs();
    void updateDigitalInputs(int bitValue, MWTime deviceTimeNanos, MWTime currentTime);
    
    bool haveDigitalOutputs() const { return !(digitalOutputChannels.empty()); }
    bool configureDigitalOutputs();
    bool startDigitalOutputs();
    bool stopDigitalOutputs();
    
    bool haveInputs() const { return haveDigitalInputs(); }
    void initializeInputs();
    void startReadInputsTask();
    void stopReadInputsTask();
    void readInputs();
    void readDigitalInputs(MWTime currentTime);
    
    void updateClockSync(MWTime currentTime);
    
    static std::atomic_flag deviceExists;
    
    const MWTime updateInterval;
    const VariablePtr clockOffsetNanosVar;
    const bool enableDigitalOutputPixelMode;
    const bool enableDigitalOutputVSYNCMode;
    const VariablePtr enableDigitalInputStabilize;
    const VariablePtr enableDigitalInputDebounce;
    const VariablePtr enableDigitalLoopback;
    const int digitalInputEventBufferMaxEvents;
    
    const boost::shared_ptr<Clock> clock;
    
    using lock_guard = std::lock_guard<std::recursive_mutex>;
    lock_guard::mutex_type mutex;
    
    unsigned int deviceRAMSize;
    unsigned int nextAvailableRAMAddress;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalInputChannel>> digitalInputChannels;
    unsigned int digitalInputEventBufferRAMAddress;
    unsigned int digitalInputEventBufferRAMSize;
    unsigned int nextDigitalInputEventBufferReadAddress;
    std::vector<DigitalInputEvent> digitalInputEvents;
    int lastCompleteDigitalInputBitValue;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalOutputChannel>> digitalOutputChannels;
    int digitalOutputBitMask;
    
    boost::shared_ptr<ScheduleTask> readInputsTask;
    
    MWTime currentClockOffsetNanos;
    MWTime lastClockSyncUpdateTime;
    
    bool running;
    
};


END_NAMESPACE_MW


#endif /* DATAPixxDevice_hpp */
