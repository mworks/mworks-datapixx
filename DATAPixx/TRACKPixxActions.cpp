//
//  TRACKPixxActions.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 7/6/22.
//  Copyright © 2022 The MWorks Project. All rights reserved.
//

#include "TRACKPixxActions.hpp"


BEGIN_NAMESPACE_MW


const std::string TRACKPixxAction::DEVICE("device");


void TRACKPixxAction::describeComponent(ComponentInfo &info) {
    Action::describeComponent(info);
    info.addParameter(DEVICE);
}


TRACKPixxAction::TRACKPixxAction(const ParameterValueMap &parameters) :
    Action(parameters),
    weakDevice(parameters[DEVICE].getRegistry()->getObject<TRACKPixxDevice>(parameters[DEVICE].str()))
{
    if (weakDevice.expired()) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                              "Device is not a TRACKPixx interface",
                              parameters[DEVICE].str());
    }
}


bool TRACKPixxAction::execute() {
    if (auto sharedDevice = weakDevice.lock()) {
        performAction(*sharedDevice);
    }
    return true;
}


void TRACKPixxBeginCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_begin_calibration");
}


void TRACKPixxBeginCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.beginCalibration();
}


const std::string TRACKPixxTakeCalibrationSampleAction::SCREEN_X("screen_x");
const std::string TRACKPixxTakeCalibrationSampleAction::SCREEN_Y("screen_y");
const std::string TRACKPixxTakeCalibrationSampleAction::LEFT_EYE("left_eye");
const std::string TRACKPixxTakeCalibrationSampleAction::RIGHT_EYE("right_eye");


void TRACKPixxTakeCalibrationSampleAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    
    info.setSignature("action/trackpixx_take_calibration_sample");
    
    info.addParameter(SCREEN_X);
    info.addParameter(SCREEN_Y);
    info.addParameter(LEFT_EYE, "YES");
    info.addParameter(RIGHT_EYE, "YES");
}


TRACKPixxTakeCalibrationSampleAction::TRACKPixxTakeCalibrationSampleAction(const ParameterValueMap &parameters) :
    TRACKPixxAction(parameters),
    screenX(parameters[SCREEN_X]),
    screenY(parameters[SCREEN_Y]),
    leftEye(parameters[LEFT_EYE]),
    rightEye(parameters[RIGHT_EYE])
{ }


void TRACKPixxTakeCalibrationSampleAction::performAction(TRACKPixxDevice &device) {
    device.takeCalibrationSample(screenX->getValue().getFloat(),
                                 screenY->getValue().getFloat(),
                                 leftEye->getValue().getBool(),
                                 rightEye->getValue().getBool());
}


void TRACKPixxEndCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_end_calibration");
}


void TRACKPixxEndCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.endCalibration();
}


void TRACKPixxLoadCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_load_calibration");
}


void TRACKPixxLoadCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.loadCalibration();
}


void TRACKPixxBeginPupilSizeCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_begin_pupil_size_calibration");
}


void TRACKPixxBeginPupilSizeCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.beginPupilSizeCalibration();
}


void TRACKPixxEndPupilSizeCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_end_pupil_size_calibration");
}


void TRACKPixxEndPupilSizeCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.endPupilSizeCalibration();
}


void TRACKPixxLoadPupilSizeCalibrationAction::describeComponent(ComponentInfo &info) {
    TRACKPixxAction::describeComponent(info);
    info.setSignature("action/trackpixx_load_pupil_size_calibration");
}


void TRACKPixxLoadPupilSizeCalibrationAction::performAction(TRACKPixxDevice &device) {
    device.loadPupilSizeCalibration();
}


END_NAMESPACE_MW
