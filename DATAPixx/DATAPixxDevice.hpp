//
//  DATAPixxDevice.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/26/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#ifndef DATAPixxDevice_hpp
#define DATAPixxDevice_hpp

#include "DATAPixxAnalogInputChannel.hpp"
#include "DATAPixxAnalogOutputChannel.hpp"
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
    static const std::string ANALOG_INPUT_DATA_INTERVAL;
    static const std::string ENABLE_DAC_ADC_LOOPBACK;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit DATAPixxDevice(const ParameterValueMap &parameters);
    ~DATAPixxDevice();
    
    void addChild(std::map<std::string, std::string> parameters,
                  ComponentRegistryPtr reg,
                  boost::shared_ptr<Component> child) override;
    
    bool initialize() override;
    bool startDeviceIO() override;
    bool stopDeviceIO() override;
    
protected:
    using lock_guard = std::lock_guard<std::recursive_mutex>;
    
    static bool logError(const char *msg);
    static bool logConfigurationFailure(bool updateLocalCache = true);
    static MWTime getDeviceTimeNanos();
    
    MWTime getUpdateInterval() const { return updateInterval; }
    lock_guard acquireLock() { return lock_guard(mutex); }
    bool allocateDeviceRAM(unsigned int size, unsigned int &address);
    MWTime applyClockOffset(MWTime deviceTimeNanos, MWTime currentTime) const;
    bool isRunning() const { return running; }
    
    virtual bool initializeDevice();
    virtual bool configureDevice();
    virtual bool startDevice();
    virtual bool stopDevice();
    virtual void readDeviceInputs(MWTime currentDeviceTimeNanos, MWTime currentTime);
    
private:
    struct UniqueDeviceGuard : boost::noncopyable {
        UniqueDeviceGuard();
        ~UniqueDeviceGuard();
    private:
        static std::atomic_flag deviceExists;
    };
    
    using DeviceTimestamp = boost::endian::little_uint64_t;
    using AnalogInputSampleValue = boost::endian::little_int16_t;  // Signed
    
    struct DigitalInputEvent {
        DeviceTimestamp deviceTimeNanos;
        boost::endian::little_uint16_t bitValue;  // Bits 0-15 only
    };
    
    static constexpr int digitalOutputVSYNCBitNumber = 23;
    static constexpr MWTime clockSyncUpdateInterval = 1000000;  // One second
    
    bool haveAnalogInputs() const { return !(analogInputChannels.empty()); }
    bool configureAnalogInputs();
    bool startAnalogInputs();
    bool stopAnalogInputs();
    void updateAnalogInputs(AnalogInputSampleValue *valuePtr, MWTime deviceTimeNanos, MWTime currentTime);
    
    bool haveAnalogOutputs() const { return !(analogOutputChannels.empty()); }
    bool configureAnalogOutputs();
    bool startAnalogOutputs();
    bool stopAnalogOutputs();
    
    bool haveDigitalInputs() const { return !(digitalInputChannels.empty()); }
    bool configureDigitalInputs();
    bool startDigitalInputs();
    bool stopDigitalInputs();
    void updateDigitalInputs(int bitValue, MWTime deviceTimeNanos, MWTime currentTime);
    
    bool haveDigitalOutputs() const { return !(digitalOutputChannels.empty()); }
    bool configureDigitalOutputs();
    bool startDigitalOutputs();
    bool stopDigitalOutputs();
    
    bool haveDigitalOutputsOnInputPort() const { return !(digitalOutputChannelsOnInputPort.empty()); }
    bool configureDigitalOutputsOnInputPort();
    bool startDigitalOutputsOnInputPort();
    bool stopDigitalOutputsOnInputPort();
    
    void initializeOutputs(MWTime currentDeviceTimeNanos, MWTime currentTime);
    
    void initializeInputs(MWTime currentDeviceTimeNanos, MWTime currentTime);
    void startReadInputsTask();
    void stopReadInputsTask();
    void readInputs();
    void readAnalogInputs(MWTime currentDeviceTimeNanos, MWTime currentTime);
    void readDigitalInputs(MWTime currentDeviceTimeNanos, MWTime currentTime);
    
    void updateClockSync(MWTime currentTime);
    
    const UniqueDeviceGuard uniqueDeviceGuard;
    const MWTime updateInterval;
    const VariablePtr clockOffsetNanosVar;
    const bool enableDigitalOutputPixelMode;
    const bool enableDigitalOutputVSYNCMode;
    const VariablePtr enableDigitalInputStabilize;
    const VariablePtr enableDigitalInputDebounce;
    const VariablePtr enableDigitalLoopback;
    const int digitalInputEventBufferMaxEvents;
    const MWTime analogInputDataInterval;
    const VariablePtr enableAnalogLoopback;
    
    const boost::shared_ptr<Clock> clock;
    
    lock_guard::mutex_type mutex;
    
    unsigned int deviceRAMSize;
    unsigned int nextAvailableRAMAddress;
    
    std::map<int, boost::shared_ptr<DATAPixxAnalogInputChannel>> analogInputChannels;
    std::size_t analogInputSampleSize;
    unsigned int analogInputSampleBufferRAMAddress;
    unsigned int analogInputSampleBufferRAMSize;
    unsigned int nextAnalogInputSampleBufferReadAddress;
    
    std::map<int, boost::shared_ptr<DATAPixxAnalogOutputChannel>> analogOutputChannels;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalInputChannel>> digitalInputChannels;
    std::set<int> usedDigitalInputBitNumbers;
    unsigned int digitalInputEventBufferRAMAddress;
    unsigned int digitalInputEventBufferRAMSize;
    unsigned int nextDigitalInputEventBufferReadAddress;
    int lastCompleteDigitalInputBitValue;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalOutputChannel>> digitalOutputChannels;
    int digitalOutputBitMask;
    
    std::vector<boost::shared_ptr<DATAPixxDigitalOutputChannel>> digitalOutputChannelsOnInputPort;
    int digitalOutputOnInputPortBitValue;
    
    boost::shared_ptr<ScheduleTask> readInputsTask;
    std::vector<std::uint8_t> readBuffer;
    
    MWTime currentClockOffsetNanos;
    MWTime lastClockSyncUpdateTime;
    
    bool running;
    
};


inline bool DATAPixxDevice::logError(const char *msg) {
    const auto error = DPxGetError();
    if (error == DPX_SUCCESS) {
        return false;
    }
    merror(M_IODEVICE_MESSAGE_DOMAIN, "%s: %s (error = %d)", msg, DPxGetErrorString(), error);
    DPxClearError();
    return true;
}


inline bool DATAPixxDevice::logConfigurationFailure(bool updateLocalCache) {
    if (updateLocalCache) {
        DPxUpdateRegCache();
    } else {
        DPxWriteRegCache();
    }
    return logError("Cannot update DATAPixx configuration");
}


inline MWTime DATAPixxDevice::getDeviceTimeNanos() {
    unsigned int nanoHigh32, nanoLow32;
    DPxGetNanoTime(&nanoHigh32, &nanoLow32);
    if (logError("Cannot retrieve current DATAPixx device time")) {
        return 0;
    }
    return MWTime((std::uint64_t(nanoHigh32) << 32) | std::uint64_t(nanoLow32));
}


inline bool DATAPixxDevice::allocateDeviceRAM(unsigned int size, unsigned int &address) {
    address = nextAvailableRAMAddress;
    nextAvailableRAMAddress += size;
    return (nextAvailableRAMAddress <= deviceRAMSize);
}


inline MWTime DATAPixxDevice::applyClockOffset(MWTime deviceTimeNanos, MWTime currentTime) const {
    if (0 == currentClockOffsetNanos || 0 == deviceTimeNanos) {
        return currentTime;
    }
    return (deviceTimeNanos + currentClockOffsetNanos) / 1000;  // ns to us
}


END_NAMESPACE_MW


#endif /* DATAPixxDevice_hpp */
