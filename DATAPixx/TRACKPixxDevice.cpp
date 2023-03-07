//
//  TRACKPixxDevice.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 6/22/22.
//  Copyright © 2022 The MWorks Project. All rights reserved.
//

#include "TRACKPixxDevice.hpp"


BEGIN_NAMESPACE_MW


BEGIN_NAMESPACE()


unsigned int lensTypeFromName(const std::string &name) {
    if (name == "25mm") {
        return 0;
    } else if (name == "50mm") {
        return 1;
    } else if (name == "75mm") {
        return 2;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid TRACKPixx lens type", name);
}


unsigned char speciesFromName(const std::string &name) {
    if (name == "human") {
        return 0;
    } else if (name == "nhp") {
        return 1;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid TRACKPixx species", name);
}


unsigned char trackingModeFromName(const std::string &name) {
    if (name == "standard") {
        return 0;
    } else if (name == "headfixed_nhp") {
        return 1;
    }
    throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, "Invalid TRACKPixx tracking mode", name);
}


template<typename IntType>
bool getPositiveInteger(const Datum &data, const char *description, IntType &val) {
    constexpr IntType maxVal = std::numeric_limits<IntType>::max();
    const auto doubleVal = data.getFloat();
    if (doubleVal <= 0.0 || std::floor(doubleVal) != doubleVal) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "TRACKPixx %s must be a positive integer", description);
        return false;
    }
    if (doubleVal > double(maxVal)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "TRACKPixx %s must be less than or equal to %" PRIuMAX,
               description,
               std::uintmax_t(maxVal));
        return false;
    }
    val = IntType(doubleVal);
    return true;
}


template<typename IntType>
bool getIntegerInRange(const Datum &data, const char *description, int minVal, int maxVal, IntType &val) {
    const auto doubleVal = data.getFloat();
    if (doubleVal < double(minVal) || doubleVal > double(maxVal) || std::floor(doubleVal) != doubleVal) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "TRACKPixx %s must be an integer between %d and %d (inclusive)",
               description,
               minVal,
               maxVal);
        return false;
    }
    val = IntType(doubleVal);
    return true;
}


inline void assignValue(const boost::shared_ptr<Variable> &var, double value, MWTime time) {
    if (var && std::isfinite(value)) {
        var->setValue(Datum(value), time);
    }
}


inline void assignEyeState(const boost::shared_ptr<Variable> &var, double value, MWTime time) {
    if (var && std::isfinite(value)) {
        const auto state = bool(value);
        if (var->getValue().getBool() != state) {
            var->setValue(Datum(state), time);
        }
    }
}


END_NAMESPACE()


const std::string TRACKPixxDevice::SHOW_OVERLAY("show_overlay");
const std::string TRACKPixxDevice::DISTANCE("distance");
const std::string TRACKPixxDevice::LENS_TYPE("lens_type");
const std::string TRACKPixxDevice::TRACKER_WINDOW_BRIGHTNESS("tracker_window_brightness");
const std::string TRACKPixxDevice::IR_ILLUMINATOR_INTENSITY("ir_illuminator_intensity");
const std::string TRACKPixxDevice::IRIS_EXPECTED_SIZE("iris_expected_size");
const std::string TRACKPixxDevice::FIXATION_SPEED("fixation_speed");
const std::string TRACKPixxDevice::FIXATION_SAMPLES("fixation_samples");
const std::string TRACKPixxDevice::SACCADE_SPEED("saccade_speed");
const std::string TRACKPixxDevice::SACCADE_SAMPLES("saccade_samples");
const std::string TRACKPixxDevice::SPECIES("species");
const std::string TRACKPixxDevice::TRACKING_MODE("tracking_mode");
const std::string TRACKPixxDevice::TRACKER_TIME_SECONDS("tracker_time_seconds");
const std::string TRACKPixxDevice::SCREEN_LX("screen_lx");
const std::string TRACKPixxDevice::SCREEN_LY("screen_ly");
const std::string TRACKPixxDevice::PUPIL_SIZE_L("pupil_size_l");
const std::string TRACKPixxDevice::SCREEN_RX("screen_rx");
const std::string TRACKPixxDevice::SCREEN_RY("screen_ry");
const std::string TRACKPixxDevice::PUPIL_SIZE_R("pupil_size_r");
const std::string TRACKPixxDevice::BLINK_L("blink_l");
const std::string TRACKPixxDevice::BLINK_R("blink_r");
const std::string TRACKPixxDevice::FIXATION_L("fixation_l");
const std::string TRACKPixxDevice::FIXATION_R("fixation_r");
const std::string TRACKPixxDevice::SACCADE_L("saccade_l");
const std::string TRACKPixxDevice::SACCADE_R("saccade_r");
const std::string TRACKPixxDevice::RAW_LX("raw_lx");
const std::string TRACKPixxDevice::RAW_LY("raw_ly");
const std::string TRACKPixxDevice::RAW_RX("raw_rx");
const std::string TRACKPixxDevice::RAW_RY("raw_ry");


void TRACKPixxDevice::describeComponent(ComponentInfo &info) {
    DATAPixxDevice::describeComponent(info);
    
    info.setSignature("iodevice/trackpixx");
    
    info.addParameter(SHOW_OVERLAY, "NO");
    info.addParameter(DISTANCE);
    info.addParameter(LENS_TYPE);
    info.addParameter(TRACKER_WINDOW_BRIGHTNESS, "200");
    info.addParameter(IR_ILLUMINATOR_INTENSITY, "8");
    info.addParameter(IRIS_EXPECTED_SIZE);
    info.addParameter(FIXATION_SPEED, "20");
    info.addParameter(FIXATION_SAMPLES, "25");
    info.addParameter(SACCADE_SPEED, "60");
    info.addParameter(SACCADE_SAMPLES, "10");
    info.addParameter(SPECIES, "human");
    info.addParameter(TRACKING_MODE, "standard");
    info.addParameter(TRACKER_TIME_SECONDS, false);
    info.addParameter(SCREEN_LX, false);
    info.addParameter(SCREEN_LY, false);
    info.addParameter(PUPIL_SIZE_L, false);
    info.addParameter(SCREEN_RX, false);
    info.addParameter(SCREEN_RY, false);
    info.addParameter(PUPIL_SIZE_R, false);
    info.addParameter(BLINK_L, false);
    info.addParameter(BLINK_R, false);
    info.addParameter(FIXATION_L, false);
    info.addParameter(FIXATION_R, false);
    info.addParameter(SACCADE_L, false);
    info.addParameter(SACCADE_R, false);
    info.addParameter(RAW_LX, false);
    info.addParameter(RAW_LY, false);
    info.addParameter(RAW_RX, false);
    info.addParameter(RAW_RY, false);
}


TRACKPixxDevice::TRACKPixxDevice(const ParameterValueMap &parameters) :
    DATAPixxDevice(parameters),
    showOverlay(parameters[SHOW_OVERLAY]),
    distance(parameters[DISTANCE]),
    lensType(lensTypeFromName(parameters[LENS_TYPE].str())),
    trackerWindowBrightness(parameters[TRACKER_WINDOW_BRIGHTNESS]),
    ledIntensity(parameters[IR_ILLUMINATOR_INTENSITY]),
    irisExpectedSize(parameters[IRIS_EXPECTED_SIZE]),
    fixationSpeed(parameters[FIXATION_SPEED]),
    fixationSamples(parameters[FIXATION_SAMPLES]),
    saccadeSpeed(parameters[SACCADE_SPEED]),
    saccadeSamples(parameters[SACCADE_SAMPLES]),
    species(speciesFromName(parameters[SPECIES].str())),
    trackingMode(trackingModeFromName(parameters[TRACKING_MODE].str())),
    trackerTimeSeconds(optionalVariable(parameters[TRACKER_TIME_SECONDS])),
    screenLeftX(optionalVariable(parameters[SCREEN_LX])),
    screenLeftY(optionalVariable(parameters[SCREEN_LY])),
    pupilSizeLeft(optionalVariable(parameters[PUPIL_SIZE_L])),
    screenRightX(optionalVariable(parameters[SCREEN_RX])),
    screenRightY(optionalVariable(parameters[SCREEN_RY])),
    pupilSizeRight(optionalVariable(parameters[PUPIL_SIZE_R])),
    blinkLeft(optionalVariable(parameters[BLINK_L])),
    blinkRight(optionalVariable(parameters[BLINK_R])),
    fixationLeft(optionalVariable(parameters[FIXATION_L])),
    fixationRight(optionalVariable(parameters[FIXATION_R])),
    saccadeLeft(optionalVariable(parameters[SACCADE_L])),
    saccadeRight(optionalVariable(parameters[SACCADE_R])),
    rawLeftX(optionalVariable(parameters[RAW_LX])),
    rawLeftY(optionalVariable(parameters[RAW_LY])),
    rawRightX(optionalVariable(parameters[RAW_RX])),
    rawRightY(optionalVariable(parameters[RAW_RY])),
    didInitialize(false),
    trackerScheduleFrameSize(0),
    trackerScheduleFrameBufferRAMAddress(0),
    trackerScheduleFrameBufferRAMSize(0),
    nextTrackerScheduleFrameBufferReadAddress(0)
{
    if (getUpdateInterval() % trackerSamplingInterval != 0) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                              boost::format("Update interval must be an integer multiple of TRACKPixx sampling "
                                            "interval (%1% us)") % trackerSamplingInterval);
    }
}


TRACKPixxDevice::~TRACKPixxDevice() {
    if (didInitialize) {
        TPxHideOverlay(), logError("Cannot hide DATAPixx eye image overlay");
        TPxSetSleepPictureRequest(), logError("Cannot put TRACKPixx to sleep");
        TPxUninitialize(), logError("Cannot uninitialize TRACKPixx device");
        logConfigurationFailure(false);
    }
}


void TRACKPixxDevice::beginCalibration() {
    auto lock = acquireLock();
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot calibrate TRACKPixx while device is running");
        return;
    }
    TPxClearDeviceCalibration();
    if (!logError("Cannot clear TRACKPixx calibration")) {
        logConfigurationFailure(false);
    }
}


void TRACKPixxDevice::takeCalibrationSample(double screenX, double screenY, bool leftEye, bool rightEye) {
    auto lock = acquireLock();
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot take TRACKPixx calibration sample while device is running");
        return;
    }
    const auto eyeToVerify = (int(leftEye) << 0) | (int(rightEye) << 1);
    TPxGetEyePositionDuringCalib(screenX, screenY, eyeToVerify);
    logError("Cannot take TRACKPixx calibration sample");
    // We don't need to call DPxUpdateRegCache (via logConfigurationFailure) here
}


void TRACKPixxDevice::endCalibration() {
    auto lock = acquireLock();
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot calibrate TRACKPixx while device is running");
        return;
    }
    TPxBestPolyFinishCalibration();
    if (!logError("Cannot finish TRACKPixx calibration") &&
        !logConfigurationFailure())
    {
        if (!TPxIsDeviceCalibrated()) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "TRACKPixx calibration failed");
        } else if (!(TPxSaveCalibration(), logError("Cannot save TRACKPixx calibration to device"))) {
            logConfigurationFailure(false);
        }
    }
}


void TRACKPixxDevice::loadCalibration() {
    auto lock = acquireLock();
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot load TRACKPixx calibration while device is running");
        return;
    }
    if (!(TPxLoadCalibration(), logError("Cannot load TRACKPixx calibration from device"))) {
        logConfigurationFailure(false);
    }
}


void TRACKPixxDevice::beginPupilSizeCalibration() {
    auto lock = acquireLock();
    
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot calibrate TRACKPixx pupil size while device is running");
        return;
    }
    
    if (!(TPxPpSizeCalibrationClear(), logError("Cannot clear TRACKPixx pupil size calibration")) &&
        // It's not clear whether we need to call DPxUpdateRegCache (via logConfigurationFailure)
        // between the calls to TPxPpSizeCalibrationClear and TPxPpSizeCalibrationGetEyeData, so
        // we do it just to be safe
        !logConfigurationFailure(false) &&
        !(TPxPpSizeCalibrationGetEyeData(), logError("Cannot begin TRACKPixx pupil size calibration")))
    {
        logConfigurationFailure(false);
    }
}


void TRACKPixxDevice::endPupilSizeCalibration() {
    auto lock = acquireLock();
    
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot calibrate TRACKPixx pupil size while device is running");
        return;
    }
    
    if (!(TPxPpSizeCalibrationDoneGatheringData(), logError("Cannot end TRACKPixx pupil size calibration")) &&
        !(TPxPpSizeCalibrationLinearRegression(), logError("Cannot calibrate TRACKPixx pupil size")) &&
        !logConfigurationFailure())
    {
        if (!TPxIsPpSizeCalibrated()) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "TRACKPixx pupil size calibration failed");
        }
    }
}


void TRACKPixxDevice::loadPupilSizeCalibration() {
    auto lock = acquireLock();
    if (isRunning()) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Cannot load TRACKPixx pupil size calibration while device is running");
        return;
    }
    if (!(TPxPpSizeCalibrationSet(), logError("Cannot load TRACKPixx pupil size calibration from device"))) {
        logConfigurationFailure(false);
    }
}


bool TRACKPixxDevice::initializeDevice() {
    if (!DATAPixxDevice::initializeDevice()) {
        return false;
    }
    
    if ((TPxInitialize(0, const_cast<char *>("")), logError("Cannot initialize TRACKPixx device"))) {
        return false;
    }
    didInitialize = true;
    
    if ((TPxDisableFreeRun(), logError("Cannot disable TRACKPixx free run")) ||
        (TPxSetAllDACOutput(0), logError("Cannot disable TRACKPixx analog output")) ||
        (TPxSetAwakePictureRequest(), logError("Cannot wake TRACKPixx")))
    {
        return false;
    }
    
    return true;
}


bool TRACKPixxDevice::configureDevice() {
    if (!DATAPixxDevice::configureDevice()) {
        return false;
    }
    
    if ((TPxDisableMedianFilter(), logError("Cannot disable TRACKPixx median filter")) ||
        (TPxDisableHDR(), logError("Cannot disable TRACKPixx high dynamic range mode")) ||
        (TPxSetLens(lensType), logError("Cannot set TRACKPixx lens type")) ||
        (TPxSpecies(1, species), logError("Cannot set TRACKPixx species")) ||
        (TPxTrackingMode(1, trackingMode), logError("Cannot set TRACKPixx tracking mode")) ||
        (TPxDisableSearchLimits(), logError("Cannot disable TRACKPixx search limits")))
    {
        return false;
    }
    
    // Register configuration variable callbacks
    {
        auto sharedThis = component_shared_from_this<TRACKPixxDevice>();
        if (!ConfigurationNotification::create(sharedThis, showOverlay, &TRACKPixxDevice::setShowOverlay) ||
            !ConfigurationNotification::create(sharedThis, distance, &TRACKPixxDevice::setDistance) ||
            !ConfigurationNotification::create(sharedThis, trackerWindowBrightness, &TRACKPixxDevice::setTrackerWindowBrightness) ||
            !ConfigurationNotification::create(sharedThis, ledIntensity, &TRACKPixxDevice::setLEDIntensity) ||
            !ConfigurationNotification::create(sharedThis, irisExpectedSize, &TRACKPixxDevice::setIrisExpectedSize) ||
            !ConfigurationNotification::create(sharedThis, fixationSpeed, &TRACKPixxDevice::setFixationSpeed) ||
            !ConfigurationNotification::create(sharedThis, fixationSamples, &TRACKPixxDevice::setFixationSamples) ||
            !ConfigurationNotification::create(sharedThis, saccadeSpeed, &TRACKPixxDevice::setSaccadeSpeed) ||
            !ConfigurationNotification::create(sharedThis, saccadeSamples, &TRACKPixxDevice::setSaccadeSamples))
        {
            return false;
        }
    }
    
    trackerScheduleFrameSize = TPxGetTrackerScheduleFrameLength(0 /*This argument seems to be ignored*/);
    if (logError("Cannot determine TRACKPixx schedule frame size")) {
        return false;
    }
    constexpr decltype(trackerScheduleFrameSize) expectedScheduleFrameSize = 64;
    if (trackerScheduleFrameSize != expectedScheduleFrameSize) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Invalid TRACKPixx schedule frame size: expected %u, got %u",
               expectedScheduleFrameSize,
               trackerScheduleFrameSize);
        return false;
    }
    
    // Allocate enough buffer space for two full update intervals worth of samples
    trackerScheduleFrameBufferRAMSize = 2 * (getUpdateInterval() / trackerSamplingInterval) * trackerScheduleFrameSize;
    if (!allocateDeviceRAM(trackerScheduleFrameBufferRAMSize, trackerScheduleFrameBufferRAMAddress)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "Required TRACKPixx schedule frame buffer size is too large for available DATAPixx device RAM");
        return false;
    }
    trackerScheduleFrames.reserve(trackerScheduleFrameBufferRAMSize / trackerScheduleFrameSize);
    
    TPxSetBuff(trackerScheduleFrameBufferRAMAddress, trackerScheduleFrameBufferRAMSize);
    if (logError("Cannot configure TRACKPixx schedule frame buffer")) {
        return false;
    }
    
    if (TPxEnableLogTimetags(), logError("Cannot enable TRACKPixx schedule frame time tagging")) {
        return false;
    }
    
    return true;
}


bool TRACKPixxDevice::startDevice() {
    if (!DATAPixxDevice::startDevice()) {
        return false;
    }
    
    // Discard any existing frames in the buffer
    TPxSetBuffWriteAddr(trackerScheduleFrameBufferRAMAddress);
    if (logError("Cannot clear TRACKPixx schedule frame buffer")) {
        return false;
    }
    nextTrackerScheduleFrameBufferReadAddress = trackerScheduleFrameBufferRAMAddress;
    
    if (TPxEnableFreeRun(), logError("Cannot start TRACKPixx schedule")) {
        return false;
    }
    
    return true;
}


bool TRACKPixxDevice::stopDevice() {
    if (TPxDisableFreeRun(), logError("Cannot stop TRACKPixx schedule")) {
        return false;
    }
    return DATAPixxDevice::stopDevice();
}


void TRACKPixxDevice::readDeviceInputs(MWTime currentDeviceTimeNanos, MWTime currentTime) {
    DATAPixxDevice::readDeviceInputs(currentDeviceTimeNanos, currentTime);
    
    const auto writeAddress = TPxGetBuffWriteAddr();
    if (logError("Cannot retrieve TRACKPixx schedule frame buffer write address")) {
        return;
    }
    const auto endAddress = trackerScheduleFrameBufferRAMAddress + trackerScheduleFrameBufferRAMSize;
    if (writeAddress >= endAddress) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,
               "TRACKPixx returned out-of-bounds schedule frame buffer write address (%u >= %u); "
               "discarding all frames in buffer",
               writeAddress,
               endAddress);
        TPxSetBuffWriteAddr(trackerScheduleFrameBufferRAMAddress);
        if (!logError("Cannot clear TRACKPixx schedule frame buffer")) {
            logConfigurationFailure(false);
        }
        return;
    }
    bool didWrap = true;
    do {
        unsigned int bytesAvailable = 0;
        if (nextTrackerScheduleFrameBufferReadAddress > writeAddress) {
            // Frames wrapped around the end of the buffer.  Read frames up to the
            // end of the buffer now, then read the rest on the next pass through
            // the loop.
            bytesAvailable = endAddress - nextTrackerScheduleFrameBufferReadAddress;
        } else {
            // No wrapping.  This is the final iteration of the loop.
            bytesAvailable = writeAddress - nextTrackerScheduleFrameBufferReadAddress;
            didWrap = false;
        }
        bytesAvailable -= bytesAvailable % trackerScheduleFrameSize;  // Read only complete frames
        if (!bytesAvailable) {
            break;
        }
        
        const auto framesAvailable = bytesAvailable / trackerScheduleFrameSize;
        trackerScheduleFrames.resize(framesAvailable);
        TPxReadTPxData(nextTrackerScheduleFrameBufferReadAddress,
                       reinterpret_cast<double *>(trackerScheduleFrames.data()),
                       framesAvailable);
        if (logError("Cannot read TRACKPixx schedule frame buffer")) {
            return;
        }
        
        nextTrackerScheduleFrameBufferReadAddress += bytesAvailable;
        if (nextTrackerScheduleFrameBufferReadAddress == endAddress) {
            nextTrackerScheduleFrameBufferReadAddress = trackerScheduleFrameBufferRAMAddress;
        }
        
        for (auto &frame : trackerScheduleFrames) {
            handleFrame(frame, currentDeviceTimeNanos, currentTime);
        }
    } while (didWrap);
}


bool TRACKPixxDevice::setShowOverlay(const Datum &data) {
    if (data.getBool()) {
        TPxShowOverlay();
    } else {
        TPxHideOverlay();
    }
    if (logError("Cannot configure DATAPixx eye image overlay")) {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setDistance(const Datum &data) {
    unsigned int value = 0;
    if (!getPositiveInteger(data, "distance", value) ||
        (TPxSetDistance(value), logError("Cannot set TRACKPixx distance")))
    {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setTrackerWindowBrightness(const Datum &data) {
    int value = 0;
    if (!getIntegerInRange(data, "tracker window brightness", 0, 255, value) ||
        (TPxSetTrackerWindowBrightness(value), logError("Cannot set TRACKPixx tracker window brightness")))
    {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setLEDIntensity(const Datum &data) {
    unsigned int value = 0;
    if (!getIntegerInRange(data, "IR illuminator intensity", 0, 8, value) ||
        (TPxSetLEDIntensity(value), logError("Cannot set TRACKPixx IR illuminator intensity")))
    {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setIrisExpectedSize(const Datum &data) {
    int value = 0;
    if (!getPositiveInteger(data, "iris expected size", value) ||
        (TPxSetIrisExpectedSize(value), logError("Cannot set TRACKPixx iris expected size")))
    {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setFixationSpeed(const Datum &data) {
    const auto value = data.getFloat();
    if (value <= 0.0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "TRACKPixx fixation speed must be greater than zero");
        return false;
    }
    if ((TPxSetFixationSpeed(value), logError("Cannot set TRACKPixx fixation speed"))) {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setFixationSamples(const Datum &data) {
    unsigned int value = 0;
    if (!getPositiveInteger(data, "fixation samples", value) ||
        (TPxSetFixationSamples(value), logError("Cannot set TRACKPixx fixation samples")))
    {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setSaccadeSpeed(const Datum &data) {
    const auto value = data.getFloat();
    if (value <= 0.0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "TRACKPixx saccade speed must be greater than zero");
        return false;
    }
    if ((TPxSetSaccadeSpeed(value), logError("Cannot set TRACKPixx saccade speed"))) {
        return false;
    }
    return true;
}


bool TRACKPixxDevice::setSaccadeSamples(const Datum &data) {
    unsigned int value = 0;
    if (!getPositiveInteger(data, "saccade samples", value) ||
        (TPxSetSaccadeSamples(value), logError("Cannot set TRACKPixx saccade samples")))
    {
        return false;
    }
    return true;
}


void TRACKPixxDevice::handleFrame(const TrackerScheduleFrame &frame,
                                  MWTime currentDeviceTimeNanos,
                                  MWTime currentTime) const
{
    const auto timeTagNanos = frame.timeTag * 1e9;
    MWTime frameDeviceTimeNanos;
    if (std::isfinite(timeTagNanos) &&
        timeTagNanos > 0.0 &&
        timeTagNanos <= double(std::numeric_limits<MWTime>::max()))
    {
        frameDeviceTimeNanos = MWTime(std::round(timeTagNanos));
    } else {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN,
                 "Ignoring TRACKPixx schedule frame time tag with unexpected value (%g)",
                 frame.timeTag);
        frameDeviceTimeNanos = currentDeviceTimeNanos;
    }
    const auto frameTime = applyClockOffset(frameDeviceTimeNanos, currentTime);
    
    assignValue(trackerTimeSeconds, frame.timeTag, frameTime);
    assignValue(screenLeftX, frame.leftScrPosX, frameTime);
    assignValue(screenLeftY, frame.leftScrPosY, frameTime);
    assignValue(pupilSizeLeft, frame.leftPPSize, frameTime);
    assignValue(screenRightX, frame.rightScrPosX, frameTime);
    assignValue(screenRightY, frame.rightScrPosY, frameTime);
    assignValue(pupilSizeRight, frame.rightPPSize, frameTime);
    assignEyeState(blinkLeft, frame.leftBlink, frameTime);
    assignEyeState(blinkRight, frame.rightBlink, frameTime);
    assignEyeState(fixationLeft, frame.leftFixation, frameTime);
    assignEyeState(fixationRight, frame.rightFixation, frameTime);
    assignEyeState(saccadeLeft, frame.leftSaccade, frameTime);
    assignEyeState(saccadeRight, frame.rightSaccade, frameTime);
    assignValue(rawLeftX, frame.leftRawX, frameTime);
    assignValue(rawLeftY, frame.leftRawY, frameTime);
    assignValue(rawRightX, frame.rightRawX, frameTime);
    assignValue(rawRightY, frame.rightRawY, frameTime);
}


bool TRACKPixxDevice::ConfigurationNotification::create(const DevicePtr &device,
                                                        const VariablePtr &var,
                                                        Callback callback)
{
    var->addNotification(boost::make_shared<ConfigurationNotification>(device, callback));
    // Invoke the callback once to establish the initial configuration
    return call(device, callback, var->getValue());
}


void TRACKPixxDevice::ConfigurationNotification::notify(const Datum &data, MWTime time) {
    if (auto sharedDevice = weakDevice.lock()) {
        auto lock = sharedDevice->acquireLock();
        if (call(sharedDevice, callback, data)) {
            logConfigurationFailure(false);
        }
    }
}


END_NAMESPACE_MW
