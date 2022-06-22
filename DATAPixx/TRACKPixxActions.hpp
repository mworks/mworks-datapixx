//
//  TRACKPixxActions.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 7/6/22.
//  Copyright © 2022 The MWorks Project. All rights reserved.
//

#ifndef TRACKPixxActions_hpp
#define TRACKPixxActions_hpp

#include "TRACKPixxDevice.hpp"


BEGIN_NAMESPACE_MW


class TRACKPixxAction : public Action {
    
public:
    static const std::string DEVICE;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit TRACKPixxAction(const ParameterValueMap &parameters);
    
    bool execute() override;
    
private:
    virtual void performAction(TRACKPixxDevice &device) = 0;
    
    const boost::weak_ptr<TRACKPixxDevice> weakDevice;
    
};


class TRACKPixxBeginCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


class TRACKPixxTakeCalibrationSampleAction : public TRACKPixxAction {
    
public:
    static const std::string SCREEN_X;
    static const std::string SCREEN_Y;
    static const std::string LEFT_EYE;
    static const std::string RIGHT_EYE;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit TRACKPixxTakeCalibrationSampleAction(const ParameterValueMap &parameters);
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
    const VariablePtr screenX;
    const VariablePtr screenY;
    const VariablePtr leftEye;
    const VariablePtr rightEye;
    
};


class TRACKPixxEndCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


class TRACKPixxLoadCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


class TRACKPixxBeginPupilSizeCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


class TRACKPixxEndPupilSizeCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


class TRACKPixxLoadPupilSizeCalibrationAction : public TRACKPixxAction {
    
public:
    static void describeComponent(ComponentInfo &info);
    
    using TRACKPixxAction::TRACKPixxAction;
    
private:
    void performAction(TRACKPixxDevice &device) override;
    
};


END_NAMESPACE_MW


#endif /* TRACKPixxActions_hpp */
