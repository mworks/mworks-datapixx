//
//  DATAPixxDevice.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/26/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDevice.hpp"

#include "libdpx.h"


BEGIN_NAMESPACE_MW


BEGIN_NAMESPACE()


inline bool logError(const char *msg) {
    const auto error = DPxGetError();
    if (error == DPX_SUCCESS) {
        return false;
    }
    merror(M_IODEVICE_MESSAGE_DOMAIN, "%s: %s (error = %d)", msg, DPxGetErrorString(), error);
    DPxClearError();
    return true;
}


inline bool logConfigurationFailure() {
    DPxUpdateRegCache();
    return logError("Cannot update DATAPixx configuration");
}


int findFirstCommonBit(int mask1, int mask2) {
    auto commonBits = mask1 & mask2;
    if (!commonBits) {
        return -1;
    }
    int index = 0;
    while (!(commonBits & 1)) {
        commonBits >>= 1;
        index++;
    }
    return index;
}


inline MWTime getDeviceTimeNanos() {
    unsigned int nanoHigh32, nanoLow32;
    DPxGetNanoTime(&nanoHigh32, &nanoLow32);
    if (logError("Cannot retrieve current DATAPixx device time")) {
        return 0;
    }
    return MWTime((std::uint64_t(nanoHigh32) << 32) | std::uint64_t(nanoLow32));
}


END_NAMESPACE()


const std::string DATAPixxDevice::UPDATE_INTERVAL("update_interval");
const std::string DATAPixxDevice::CLOCK_OFFSET_NANOS("clock_offset_nanos");
const std::string DATAPixxDevice::ENABLE_DOUT_PIXEL_MODE("enable_dout_pixel_mode");
const std::string DATAPixxDevice::ENABLE_DOUT_VSYNC_MODE("enable_dout_vsync_mode");
const std::string DATAPixxDevice::ENABLE_DIN_STABILIZE("enable_din_stabilize");
const std::string DATAPixxDevice::ENABLE_DIN_DEBOUNCE("enable_din_debounce");
const std::string DATAPixxDevice::ENABLE_DOUT_DIN_LOOPBACK("enable_dout_din_loopback");
const std::string DATAPixxDevice::DIN_EVENT_BUFFER_SIZE("din_event_buffer_size");
const std::string DATAPixxDevice::ANALOG_INPUT_DATA_INTERVAL("analog_input_data_interval");
const std::string DATAPixxDevice::ENABLE_DAC_ADC_LOOPBACK("enable_dac_adc_loopback");


void DATAPixxDevice::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/datapixx");
    
    info.addParameter(UPDATE_INTERVAL, true);
    info.addParameter(CLOCK_OFFSET_NANOS, false);
    info.addParameter(ENABLE_DOUT_PIXEL_MODE, "NO");
    info.addParameter(ENABLE_DOUT_VSYNC_MODE, "NO");
    info.addParameter(ENABLE_DIN_STABILIZE, "NO");
    info.addParameter(ENABLE_DIN_DEBOUNCE, "NO");
    info.addParameter(ENABLE_DOUT_DIN_LOOPBACK, "NO");
    info.addParameter(DIN_EVENT_BUFFER_SIZE, "100");
    info.addParameter(ANALOG_INPUT_DATA_INTERVAL, "0");
    info.addParameter(ENABLE_DAC_ADC_LOOPBACK, "NO");
}


DATAPixxDevice::DATAPixxDevice(const ParameterValueMap &parameters) :
    IODevice(parameters),
    updateInterval(parameters[UPDATE_INTERVAL]),
    clockOffsetNanosVar(optionalVariable(parameters[CLOCK_OFFSET_NANOS])),
    enableDigitalOutputPixelMode(parameters[ENABLE_DOUT_PIXEL_MODE]),
    enableDigitalOutputVSYNCMode(parameters[ENABLE_DOUT_VSYNC_MODE]),
    enableDigitalInputStabilize(parameters[ENABLE_DIN_STABILIZE]),
    enableDigitalInputDebounce(parameters[ENABLE_DIN_DEBOUNCE]),
    enableDigitalLoopback(parameters[ENABLE_DOUT_DIN_LOOPBACK]),
    digitalInputEventBufferMaxEvents(parameters[DIN_EVENT_BUFFER_SIZE]),
    analogInputDataInterval(parameters[ANALOG_INPUT_DATA_INTERVAL]),
    enableAnalogLoopback(parameters[ENABLE_DAC_ADC_LOOPBACK]),
    clock(Clock::instance()),
    deviceRAMSize(0),
    nextAvailableRAMAddress(0),
    analogInputSampleSize(0),
    analogInputSampleBufferRAMAddress(0),
    analogInputSampleBufferRAMSize(0),
    nextAnalogInputSampleBufferReadAddress(0),
    digitalInputEventBufferRAMAddress(0),
    digitalInputEventBufferRAMSize(0),
    nextDigitalInputEventBufferReadAddress(0),
    lastCompleteDigitalInputBitValue(0),
    digitalOutputBitMask(0),
    currentClockOffsetNanos(0),
    lastClockSyncUpdateTime(0),
    running(false)
{
    if (updateInterval <= 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid update interval");
    }
    if (enableDigitalOutputPixelMode && enableDigitalOutputVSYNCMode) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Pixel mode and VSYNC mode cannot be enabled simultaneously");
    }
    if (digitalInputEventBufferMaxEvents <= 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid digital input event buffer size");
    }
}


DATAPixxDevice::~DATAPixxDevice() {
    DPxClose();
    logError("Cannot close DATAPixx device");
}


void DATAPixxDevice::addChild(std::map<std::string, std::string> parameters,
                      ComponentRegistryPtr reg,
                      boost::shared_ptr<Component> child)
{
    if (auto channel = boost::dynamic_pointer_cast<DATAPixxAnalogInputChannel>(child)) {
        const auto channelNumber = channel->getChannelNumber();
        if (!(analogInputChannels.insert(std::make_pair(channelNumber, channel)).second)) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Analog input channel %d is already in use") % channelNumber);
        }
        return;
    }
    if (auto channel = boost::dynamic_pointer_cast<DATAPixxAnalogOutputChannel>(child)) {
        const auto channelNumber = channel->getChannelNumber();
        if (!(analogOutputChannels.insert(std::make_pair(channelNumber, channel)).second)) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Analog output channel %d is already in use") % channelNumber);
        }
        return;
    }
    if (auto channel = boost::dynamic_pointer_cast<DATAPixxDigitalInputChannel>(child)) {
        digitalInputChannels.push_back(channel);
        return;
    }
    if (auto channel = boost::dynamic_pointer_cast<DATAPixxDigitalOutputChannel>(child)) {
        digitalOutputChannels.push_back(channel);
        return;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid channel type for DATAPixx device");
}


bool DATAPixxDevice::initialize() {
    lock_guard lock(mutex);
    
    if ((DPxOpen(), logError("Cannot open DATAPixx device"))) {
        return false;
    }
    
    mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Opened DATAPixx device (ID = %d)", DPxGetID());
    
    if ((DPxStopAllScheds(), logError("Cannot stop existing DATAPixx schedules")) ||
        (DPxDisableAdcFreeRun(), logError("Cannot disable DATAPixx ADC free run")) ||
        (DPxDisableDinLogEvents(), logError("Cannot disable DATAPixx digital input event logging")) ||
        logConfigurationFailure())
    {
        return false;
    }
    
    if (!configureDevice()) {
        return false;
    }
    if (haveAnalogInputs() && !configureAnalogInputs()) {
        return false;
    }
    if (haveAnalogOutputs() && !configureAnalogOutputs()) {
        return false;
    }
    if (haveDigitalInputs() && !configureDigitalInputs()) {
        return false;
    }
    if (haveDigitalOutputs() && !configureDigitalOutputs()) {
        return false;
    }
    
    // Commit all configuration changes
    if (logConfigurationFailure()) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDeviceIO() {
    lock_guard lock(mutex);
    
    if (!running) {
        if (haveAnalogOutputs() && !startAnalogOutputs()) {
            return false;
        }
        if (haveAnalogInputs() && !startAnalogInputs()) {
            return false;
        }
        if (haveDigitalOutputs() && !startDigitalOutputs()) {
            return false;
        }
        if (haveDigitalInputs() && !startDigitalInputs()) {
            return false;
        }
        
        if (logConfigurationFailure()) {
            return false;
        }
        
        const auto currentTime = clock->getCurrentTimeUS();
        const auto currentDeviceTimeNanos = getDeviceTimeNanos();
        
        if (haveOutputs()) {
            initializeOutputs(currentDeviceTimeNanos, currentTime);
        }
        if (haveInputs()) {
            initializeInputs(currentDeviceTimeNanos, currentTime);
            if (!readInputsTask) {
                startReadInputsTask();
            }
        }
        
        running = true;
    }
    
    return true;
}


bool DATAPixxDevice::stopDeviceIO() {
    lock_guard lock(mutex);
    
    if (running) {
        if (readInputsTask) {
            stopReadInputsTask();
        }
        
        if (haveDigitalOutputs() && !stopDigitalOutputs()) {
            return false;
        }
        if (haveAnalogInputs() && !stopAnalogInputs()) {
            return false;
        }
        if (haveAnalogOutputs() && !stopAnalogOutputs()) {
            return false;
        }
        
        if (logConfigurationFailure()) {
            return false;
        }
        
        running = false;
    }
    
    return true;
}


bool DATAPixxDevice::configureDevice() {
    if (enableDigitalOutputPixelMode) {
        if (haveDigitalOutputs()) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  "Digital outputs cannot be used when pixel mode is enabled");
        }
        DPxEnableDoutPixelMode();
    } else {
        DPxDisableDoutPixelMode();
    }
    if (logError("Cannot configure DATAPixx digital output pixel mode")) {
        return false;
    }
    
    if (enableDigitalOutputVSYNCMode) {
        DPxEnableDoutVsyncMode();
    } else {
        DPxDisableDoutVsyncMode();
    }
    if (logError("Cannot configure DATAPixx digital output VSYNC mode")) {
        return false;
    }
    
    deviceRAMSize = DPxGetRamSize();
    if (logError("Cannot determine DATAPixx device RAM size")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::configureAnalogInputs() {
    if (analogInputDataInterval <= 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid analog input data interval");
    }
    if (updateInterval % analogInputDataInterval != 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                              "Update interval must be an integer multiple of analog input data interval");
    }
    
    const auto channelNumberMax = DPxGetAdcNumChans() - 1;
    if (logError("Cannot determine number of ADC channels in DATAPixx device")) {
        return false;
    }
    
    if ((DPxDisableAdcCalibRaw(), logError("Cannot disable DATAPixx ADC raw mode")) ||
        (DPxDisableAdcBuffAllChans(), logError("Cannot disable DATAPixx ADC RAM buffering")))
    {
        return false;
    }
    
    for (auto &item : analogInputChannels) {
        const auto channelNumber = item.first;
        auto &channel = item.second;
        
        if (channelNumber > channelNumberMax) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Invalid analog input channel number: %d") % channelNumber);
        }
        
        double voltageMin, voltageMax;
        DPxGetAdcRange(channelNumber, &voltageMin, &voltageMax);
        if (logError("Cannot determine range of DATAPixx ADC channel")) {
            return false;
        }
        channel->setVoltageRange(voltageMin, voltageMax);
        
        int referenceSource;
        switch (channel->getReferenceSource()) {
            case DATAPixxAnalogInputChannel::ReferenceSource::GND:
                referenceSource = DPXREG_ADC_CHANREF_GND;
                break;
            case DATAPixxAnalogInputChannel::ReferenceSource::DIFF:
                referenceSource = DPXREG_ADC_CHANREF_DIFF;
                break;
            case DATAPixxAnalogInputChannel::ReferenceSource::REF0:
                referenceSource = DPXREG_ADC_CHANREF_REF0;
                break;
            case DATAPixxAnalogInputChannel::ReferenceSource::REF1:
                referenceSource = DPXREG_ADC_CHANREF_REF1;
                break;
        }
        DPxSetAdcBuffChanRef(channelNumber, referenceSource);
        if (logError("Cannot set differential reference source of DATAPixx ADC channel")) {
            return false;
        }
        
        DPxEnableAdcBuffChan(channelNumber);
        if (logError("Cannot enable RAM buffering on DATAPixx ADC channel")) {
            return false;
        }
    }
    
    // Allocate enough buffer space for two full update intervals worth of samples
    analogInputSampleSize = sizeof(DeviceTimestamp) + analogInputChannels.size() * sizeof(AnalogInputSampleValue);
    analogInputSampleBufferRAMAddress = nextAvailableRAMAddress;
    analogInputSampleBufferRAMSize = 2 * (updateInterval / analogInputDataInterval) * analogInputSampleSize;
    nextAvailableRAMAddress += analogInputSampleBufferRAMSize;
    if (nextAvailableRAMAddress > deviceRAMSize) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Required analog input sample buffer size is too large for available DATAPixx device RAM");
        return false;
    }
    readBuffer.reserve(analogInputSampleBufferRAMSize);
    
    DPxSetAdcBuff(analogInputSampleBufferRAMAddress, analogInputSampleBufferRAMSize);
    if (logError("Cannot configure DATAPixx analog input sample buffer")) {
        return false;
    }
    
    if (DPxEnableAdcLogTimetags(), logError("Cannot enable DATAPixx analog input sample time tagging")) {
        return false;
    }
    
    const auto dataIntervalNanos = analogInputDataInterval * 1000;  // us to ns
    DPxSetAdcSched(dataIntervalNanos, dataIntervalNanos, DPXREG_SCHED_CTRL_RATE_NANO, 0);
    if (logError("Cannot configure DATAPixx analog input schedule")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startAnalogInputs() {
    if (enableAnalogLoopback->getValue().getBool()) {
        DPxEnableDacAdcLoopback();
    } else {
        DPxDisableDacAdcLoopback();
    }
    if (logError("Cannot configure DATAPixx analog loopback")) {
        return false;
    }
    
    // Discard any existing samples in the buffer
    DPxSetAdcBuffWriteAddr(analogInputSampleBufferRAMAddress);
    if (logError("Cannot clear DATAPixx analog input sample buffer")) {
        return false;
    }
    nextAnalogInputSampleBufferReadAddress = analogInputSampleBufferRAMAddress;
    
    if (DPxStartAdcSched(), logError("Cannot start DATAPixx analog input schedule")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::stopAnalogInputs() {
    if (DPxStopAdcSched(), logError("Cannot stop DATAPixx analog input schedule")) {
        return false;
    }
    return true;
}


void DATAPixxDevice::updateAnalogInputs(AnalogInputSampleValue *valuePtr, MWTime deviceTimeNanos, MWTime currentTime) {
    const auto time = applyClockOffset(deviceTimeNanos, currentTime);
    for (auto &item : analogInputChannels) {
        auto &channel = item.second;
        channel->setValue(*valuePtr, deviceTimeNanos, time);
        valuePtr++;
    }
}


bool DATAPixxDevice::configureAnalogOutputs() {
    const auto channelNumberMax = DPxGetDacNumChans() - 1;
    if (logError("Cannot determine number of DAC channels in DATAPixx device")) {
        return false;
    }
    
    if ((DPxDisableDacCalibRaw(), logError("Cannot disable DATAPixx DAC raw mode")) ||
        (DPxDisableDacBuffAllChans(), logError("Cannot disable DATAPixx DAC RAM buffering")))
    {
        return false;
    }
    
    boost::weak_ptr<DATAPixxDevice> weakThis(component_shared_from_this<DATAPixxDevice>());
    
    for (auto &item : analogOutputChannels) {
        const auto channelNumber = item.first;
        auto &channel = item.second;
        
        if (channelNumber > channelNumberMax) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Invalid analog output channel number: %d") % channelNumber);
        }
        
        double voltageMin, voltageMax;
        DPxGetDacRange(channelNumber, &voltageMin, &voltageMax);
        if (logError("Cannot determine range of DATAPixx DAC channel")) {
            return false;
        }
        channel->setVoltageRange(voltageMin, voltageMax);
        
        // It's OK to capture channel by reference, because it will remain alive (in
        // analogOutputChannels) for as long as the device is alive
        auto callback = [weakThis, &channel, channelNumber](const Datum &data, MWTime time) {
            if (auto sharedThis = weakThis.lock()) {
                lock_guard lock(sharedThis->mutex);
                if (sharedThis->running) {
                    auto value = channel->getValue();
                    DPxSetDacValue(value, channelNumber);
                    if (!logError("Cannot set DATAPixx analog output") &&
                        !logConfigurationFailure())
                    {
                        channel->setDeviceTimeNanos(getDeviceTimeNanos(), time);
                    }
                }
            }
        };
        channel->addNewValueNotification(boost::make_shared<VariableCallbackNotification>(callback));
    }
    
    // Initialize all configured analog output channels to zero
    if (!stopAnalogOutputs()) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startAnalogOutputs() {
    for (auto &item : analogOutputChannels) {
        const auto channelNumber = item.first;
        auto &channel = item.second;
        DPxSetDacValue(channel->getValue(), channelNumber);
        if (logError("Cannot initialize DATAPixx analog output")) {
            return false;
        }
    }
    return true;
}


bool DATAPixxDevice::stopAnalogOutputs() {
    for (auto &item : analogOutputChannels) {
        const auto channelNumber = item.first;
        DPxSetDacValue(0, channelNumber);
        if (logError("Cannot reset DATAPixx analog output")) {
            return false;
        }
    }
    return true;
}


bool DATAPixxDevice::configureDigitalInputs() {
    // Reset all bits to input mode
    if (DPxSetDinDataDir(0), logError("Cannot configure DATAPixx digital inputs")) {
        return false;
    }
    
    digitalInputEventBufferRAMAddress = nextAvailableRAMAddress;
    digitalInputEventBufferRAMSize = digitalInputEventBufferMaxEvents * sizeof(DigitalInputEvent);
    nextAvailableRAMAddress += digitalInputEventBufferRAMSize;
    if (nextAvailableRAMAddress > deviceRAMSize) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Requested digital input event buffer size is too large for available DATAPixx device RAM");
        return false;
    }
    readBuffer.reserve(digitalInputEventBufferRAMSize);
    
    DPxSetDinBuff(digitalInputEventBufferRAMAddress, digitalInputEventBufferRAMSize);
    if (logError("Cannot configure DATAPixx digital input event buffer")) {
        return false;
    }
    
    if (DPxEnableDinLogTimetags(), logError("Cannot enable DATAPixx digital input event time tagging")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDigitalInputs() {
    if (enableDigitalInputStabilize->getValue().getBool()) {
        DPxEnableDinStabilize();
    } else {
        DPxDisableDinStabilize();
    }
    if (logError("Cannot configure DATAPixx digital input stabilization")) {
        return false;
    }
    
    if (enableDigitalInputDebounce->getValue().getBool()) {
        DPxEnableDinDebounce();
    } else {
        DPxDisableDinDebounce();
    }
    if (logError("Cannot configure DATAPixx digital input debouncing")) {
        return false;
    }
    
    if (enableDigitalLoopback->getValue().getBool()) {
        DPxEnableDoutDinLoopback();
    } else {
        DPxDisableDoutDinLoopback();
    }
    if (logError("Cannot configure DATAPixx digital loopback")) {
        return false;
    }
    
    // Discard any existing events in the buffer
    DPxSetDinBuffWriteAddr(digitalInputEventBufferRAMAddress);
    if (logError("Cannot clear DATAPixx digital input event buffer")) {
        return false;
    }
    nextDigitalInputEventBufferReadAddress = digitalInputEventBufferRAMAddress;
    
    if (DPxEnableDinLogEvents(), logError("Cannot enable DATAPixx digital input event logging")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::stopDigitalInputs() {
    if (DPxDisableDinLogEvents(), logError("Cannot disable DATAPixx digital input event logging")) {
        return false;
    }
    return true;
}


void DATAPixxDevice::updateDigitalInputs(int bitValue, MWTime deviceTimeNanos, MWTime currentTime) {
    const auto time = applyClockOffset(deviceTimeNanos, currentTime);
    for (auto &channel : digitalInputChannels) {
        channel->setBitValue(bitValue, deviceTimeNanos, time);
    }
}


bool DATAPixxDevice::configureDigitalOutputs() {
    if (DPxDisableDoutButtonSchedules(), logError("Cannot disable DATAPixx DOUT button schedules")) {
        return false;
    }
    
    boost::weak_ptr<DATAPixxDevice> weakThis(component_shared_from_this<DATAPixxDevice>());
    
    for (auto &channel : digitalOutputChannels) {
        const auto bitMask = channel->getBitMask();
        const auto duplicateBitNumber = findFirstCommonBit(digitalOutputBitMask, bitMask);
        if (-1 != duplicateBitNumber) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Digital output bit %d is already in use") % duplicateBitNumber);
        }
        if (enableDigitalOutputVSYNCMode && (bitMask & (1 << digitalOutputVSYNCBitNumber))) {
            throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                                  boost::format("Digital output bit %d cannot be used when VSYNC mode is enabled") %
                                  digitalOutputVSYNCBitNumber);
        }
        digitalOutputBitMask |= bitMask;
        
        // It's OK to capture channel by reference, because it will remain alive (in
        // digitalOutputChannels) for as long as the device is alive
        auto callback = [weakThis, &channel, bitMask](const Datum &data, MWTime time) {
            if (auto sharedThis = weakThis.lock()) {
                lock_guard lock(sharedThis->mutex);
                if (sharedThis->running) {
                    auto bitValue = channel->getBitValue();
                    DPxSetDoutValue(bitValue, bitMask);
                    if (!logError("Cannot set DATAPixx digital output") &&
                        !logConfigurationFailure())
                    {
                        channel->setDeviceTimeNanos(getDeviceTimeNanos(), time);
                    }
                }
            }
        };
        channel->addNewValueNotification(boost::make_shared<VariableCallbackNotification>(callback));
    }
    
    // Initialize all configured digital output bits to zero
    if (!stopDigitalOutputs()) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::startDigitalOutputs() {
    int bitValue = 0;
    
    for (auto &channel : digitalOutputChannels) {
        bitValue |= channel->getBitValue();
    }
    
    if (DPxSetDoutValue(bitValue, digitalOutputBitMask), logError("Cannot initialize DATAPixx digital outputs")) {
        return false;
    }
    
    return true;
}


bool DATAPixxDevice::stopDigitalOutputs() {
    if (DPxSetDoutValue(0, digitalOutputBitMask), logError("Cannot reset DATAPixx digital outputs")) {
        return false;
    }
    return true;
}


void DATAPixxDevice::initializeOutputs(MWTime currentDeviceTimeNanos, MWTime currentTime) {
    if (haveAnalogOutputs()) {
        for (auto &item : analogOutputChannels) {
            auto &channel = item.second;
            channel->setDeviceTimeNanos(currentDeviceTimeNanos, currentTime);
        }
    }
    
    if (haveDigitalOutputs()) {
        for (auto &channel : digitalOutputChannels) {
            channel->setDeviceTimeNanos(currentDeviceTimeNanos, currentTime);
        }
    }
}


void DATAPixxDevice::initializeInputs(MWTime currentDeviceTimeNanos, MWTime currentTime) {
    std::vector<AnalogInputSampleValue> analogInputValues;
    int digitalInputBitValue = 0;
    bool didReadDigitalInputs = false;
    
    for (auto &item : analogInputChannels) {
        const auto channelNumber = item.first;
        const auto value = DPxGetAdcValue(channelNumber);
        if (logError("Cannot read DATAPixx analog input")) {
            break;
        }
        analogInputValues.push_back(value);
    }
    
    // We need to acquire the initial state of digital inputs *before* we compute the clock
    // offset.  Otherwise, the state we get may reflect events logged by the device while
    // updateClockSync was running.
    if (haveDigitalInputs()) {
        digitalInputBitValue = DPxGetDinValue();
        if (logError("Cannot read DATAPixx digital inputs")) {
            lastCompleteDigitalInputBitValue = 0;
        } else {
            lastCompleteDigitalInputBitValue = digitalInputBitValue;
            didReadDigitalInputs = true;
        }
    }
    
    updateClockSync(currentTime);
    
    if (analogInputValues.size() == analogInputChannels.size()) {
        updateAnalogInputs(analogInputValues.data(), currentDeviceTimeNanos, currentTime);
    }
    
    if (didReadDigitalInputs) {
        updateDigitalInputs(digitalInputBitValue, currentDeviceTimeNanos, currentTime);
    }
}


void DATAPixxDevice::startReadInputsTask() {
    boost::weak_ptr<DATAPixxDevice> weakThis(component_shared_from_this<DATAPixxDevice>());
    auto action = [weakThis]() {
        if (auto sharedThis = weakThis.lock()) {
            lock_guard lock(sharedThis->mutex);
            sharedThis->readInputs();
        }
        return nullptr;
    };
    readInputsTask = Scheduler::instance()->scheduleUS(FILELINE,
                                                       updateInterval,
                                                       updateInterval,
                                                       M_REPEAT_INDEFINITELY,
                                                       action,
                                                       M_DEFAULT_IODEVICE_PRIORITY,
                                                       M_DEFAULT_IODEVICE_WARN_SLOP_US,
                                                       M_DEFAULT_IODEVICE_FAIL_SLOP_US,
                                                       M_MISSED_EXECUTION_DROP);
}


void DATAPixxDevice::stopReadInputsTask() {
    readInputsTask->cancel();
    readInputsTask.reset();
}


void DATAPixxDevice::readInputs() {
    if (!readInputsTask) {
        // If we've already been canceled, don't try to read more data
        return;
    }
    
    // Retrieve current register values
    if (DPxUpdateRegCache(), logError("Cannot update DATAPixx register cache")) {
        return;
    }
    
    const auto currentTime = clock->getCurrentTimeUS();
    const auto currentDeviceTimeNanos = getDeviceTimeNanos();
    
    if (haveAnalogInputs()) {
        readAnalogInputs(currentDeviceTimeNanos, currentTime);
    }
    if (haveDigitalInputs()) {
        readDigitalInputs(currentDeviceTimeNanos, currentTime);
    }
    
    if (currentTime - lastClockSyncUpdateTime >= clockSyncUpdateInterval) {
        updateClockSync(currentTime);
    }
}


void DATAPixxDevice::readAnalogInputs(MWTime currentDeviceTimeNanos, MWTime currentTime) {
    const auto writeAddress = DPxGetAdcBuffWriteAddr();
    if (logError("Cannot retrieve DATAPixx analog input sample buffer write address")) {
        return;
    }
    const auto endAddress = analogInputSampleBufferRAMAddress + analogInputSampleBufferRAMSize;
    bool didWrap = true;
    do {
        unsigned int bytesAvailable = 0;
        if (nextAnalogInputSampleBufferReadAddress > writeAddress) {
            // Samples wrapped around the end of the buffer.  Read samples up to
            // the end of the buffer now, then read the rest on the next pass through
            // the loop.
            bytesAvailable = endAddress - nextAnalogInputSampleBufferReadAddress;
        } else {
            // No wrapping.  This is the final iteration of the loop.
            bytesAvailable = writeAddress - nextAnalogInputSampleBufferReadAddress;
            didWrap = false;
        }
        bytesAvailable -= bytesAvailable % analogInputSampleSize;  // Read only complete samples
        if (!bytesAvailable) {
            break;
        }
        
        readBuffer.resize(bytesAvailable);
        DPxReadRam(nextAnalogInputSampleBufferReadAddress, bytesAvailable, readBuffer.data());
        if (logError("Cannot read DATAPixx analog input sample buffer")) {
            return;
        }
        
        nextAnalogInputSampleBufferReadAddress += bytesAvailable;
        if (nextAnalogInputSampleBufferReadAddress == endAddress) {
            nextAnalogInputSampleBufferReadAddress = analogInputSampleBufferRAMAddress;
        }
        
        for (auto iter = readBuffer.begin(); iter != readBuffer.end(); iter += analogInputSampleSize) {
            const auto deviceTimeNanos = *reinterpret_cast<DeviceTimestamp *>(&(*iter));
            auto valuePtr = reinterpret_cast<AnalogInputSampleValue *>(&(*(iter + sizeof(DeviceTimestamp))));
            updateAnalogInputs(valuePtr, deviceTimeNanos, currentTime);
        }
    } while (didWrap);
}


void DATAPixxDevice::readDigitalInputs(MWTime currentDeviceTimeNanos, MWTime currentTime) {
    const auto writeAddress = DPxGetDinBuffWriteAddr();
    if (logError("Cannot retrieve DATAPixx digital input event buffer write address")) {
        return;
    }
    const auto endAddress = digitalInputEventBufferRAMAddress + digitalInputEventBufferRAMSize;
    bool didWrap = true;
    do {
        unsigned int bytesAvailable = 0;
        if (nextDigitalInputEventBufferReadAddress > writeAddress) {
            // Event log wrapped around the end of the buffer.  Read events up to
            // the end of the buffer now, then read the rest on the next pass through
            // the loop.
            bytesAvailable = endAddress - nextDigitalInputEventBufferReadAddress;
        } else {
            // No wrapping.  This is the final iteration of the loop.
            bytesAvailable = writeAddress - nextDigitalInputEventBufferReadAddress;
            didWrap = false;
        }
        bytesAvailable -= bytesAvailable % sizeof(DigitalInputEvent);  // Read only complete events
        if (!bytesAvailable) {
            break;
        }
        
        readBuffer.resize(bytesAvailable);
        DPxReadRam(nextDigitalInputEventBufferReadAddress, bytesAvailable, readBuffer.data());
        if (logError("Cannot read DATAPixx digital input event buffer")) {
            return;
        }
        
        nextDigitalInputEventBufferReadAddress += bytesAvailable;
        if (nextDigitalInputEventBufferReadAddress == endAddress) {
            nextDigitalInputEventBufferReadAddress = digitalInputEventBufferRAMAddress;
        }
        
        for (auto iter = readBuffer.begin(); iter != readBuffer.end(); iter += sizeof(DigitalInputEvent)) {
            auto &event = *reinterpret_cast<DigitalInputEvent *>(&(*iter));
            int bitValue = event.bitValue;
            // OR in the most recent state of bits 16-23, which aren't logged
            bitValue |= (lastCompleteDigitalInputBitValue & 0xFF0000);
            updateDigitalInputs(bitValue, event.deviceTimeNanos, currentTime);
        }
    } while (didWrap);
    
    // Read the current values to update the state of bits 16-23
    const auto bitValue = DPxGetDinValue();
    if (!logError("Cannot read DATAPixx digital inputs")) {
        updateDigitalInputs(bitValue, currentDeviceTimeNanos, currentTime);
        lastCompleteDigitalInputBitValue = bitValue;
    }
}


void DATAPixxDevice::updateClockSync(MWTime currentTime) {
    // Reset stored clock offset to zero, so that we know not to use it if the
    // update fails
    currentClockOffsetNanos = 0;
    
    std::array<std::tuple<MWTime, MWTime, std::uint64_t>, 5> samples;
    
    for (auto &sample : samples) {
        auto beforeNS = clock->getCurrentTimeNS();
        DPxUpdateRegCache();
        auto afterNS = clock->getCurrentTimeNS();
        if (logError("Cannot update DATAPixx register cache")) {
            return;
        }
        
        auto deviceTimeNanos = getDeviceTimeNanos();
        if (0 == deviceTimeNanos) {
            return;
        }
        
        sample = std::make_tuple(beforeNS, afterNS - beforeNS, deviceTimeNanos);
    }
    
    // Sort by afterNS-beforeNS and extract median (to exclude outliers)
    std::sort(samples.begin(), samples.end(), [](const auto &first, const auto &second) {
        return std::get<1>(first) < std::get<1>(second);
    });
    const auto &median = samples.at(samples.size() / 2);
    
    currentClockOffsetNanos = (std::get<0>(median) + std::get<1>(median) / 2) - std::get<2>(median);
    if (clockOffsetNanosVar) {
        clockOffsetNanosVar->setValue(Datum(currentClockOffsetNanos));
    }
    
    lastClockSyncUpdateTime = currentTime;
}


MWTime DATAPixxDevice::applyClockOffset(MWTime deviceTimeNanos, MWTime currentTime) const {
    if (0 == currentClockOffsetNanos || 0 == deviceTimeNanos) {
        return currentTime;
    }
    return (deviceTimeNanos + currentClockOffsetNanos) / 1000;  // ns to us
}


DATAPixxDevice::UniqueDeviceGuard::UniqueDeviceGuard() {
    if (deviceExists.test_and_set()) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Experiment can contain at most one DATAPixx device");
    }
}


DATAPixxDevice::UniqueDeviceGuard::~UniqueDeviceGuard() {
    deviceExists.clear();
}


std::atomic_flag DATAPixxDevice::UniqueDeviceGuard::deviceExists = ATOMIC_FLAG_INIT;


END_NAMESPACE_MW
