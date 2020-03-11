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
    clock(Clock::instance()),
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
    // Need to do this last.  Otherwise, if a subsequent check throws, the destructor will never run,
    // and the flag will never be cleared.
    if (deviceExists.test_and_set()) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Experiment can contain at most one DATAPixx device");
    }
}


DATAPixxDevice::~DATAPixxDevice() {
    DPxClose();
    logError("Cannot close DATAPixx device");
    deviceExists.clear();
}


void DATAPixxDevice::addChild(std::map<std::string, std::string> parameters,
                      ComponentRegistryPtr reg,
                      boost::shared_ptr<Component> child)
{
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
        logConfigurationFailure())
    {
        return false;
    }
    
    if (!configureDevice()) {
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
        if (haveDigitalOutputs() && !startDigitalOutputs()) {
            return false;
        }
        if (haveDigitalInputs() && !startDigitalInputs()) {
            return false;
        }
        if (logConfigurationFailure()) {
            return false;
        }
        
        if (haveInputs()) {
            updateClockSync(clock->getCurrentTimeUS());
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
    
    return true;
}


bool DATAPixxDevice::configureDigitalInputs() {
    // Reset all bits to input mode
    if (DPxSetDinDataDir(0), logError("Cannot configure DATAPixx digital inputs")) {
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
    
    return true;
}


bool DATAPixxDevice::configureDigitalOutputs() {
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
    const auto deviceTimeNanos = getDeviceTimeNanos();
    
    MWTime time;
    if (0 == currentClockOffsetNanos || 0 == deviceTimeNanos) {
        time = currentTime;
    } else {
        time = (deviceTimeNanos + currentClockOffsetNanos) / 1000;  // ns to us
    }
    
    if (haveDigitalInputs()) {
        const auto bitValue = DPxGetDinValue();
        if (!logError("Cannot read DATAPixx digital inputs")) {
            for (auto &channel : digitalInputChannels) {
                channel->setBitValue(bitValue, deviceTimeNanos, time);
            }
        }
    }
    
    if (currentTime - lastClockSyncUpdateTime >= clockSyncUpdateInterval) {
        updateClockSync(currentTime);
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


std::atomic_flag DATAPixxDevice::deviceExists = ATOMIC_FLAG_INIT;


END_NAMESPACE_MW
