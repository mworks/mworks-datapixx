//
//  TRACKPixxDevice.hpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 6/22/22.
//  Copyright © 2022 The MWorks Project. All rights reserved.
//

#ifndef TRACKPixxDevice_hpp
#define TRACKPixxDevice_hpp

#include "DATAPixxDevice.hpp"


BEGIN_NAMESPACE_MW


class TRACKPixxDevice : public DATAPixxDevice {
    
public:
    static const std::string SHOW_OVERLAY;
    static const std::string DISTANCE;
    static const std::string LENS_TYPE;
    static const std::string TRACKER_WINDOW_BRIGHTNESS;
    static const std::string IR_ILLUMINATOR_INTENSITY;
    static const std::string IRIS_EXPECTED_SIZE;
    static const std::string FIXATION_SPEED;
    static const std::string FIXATION_SAMPLES;
    static const std::string SACCADE_SPEED;
    static const std::string SACCADE_SAMPLES;
    static const std::string SPECIES;
    static const std::string TRACKING_MODE;
    static const std::string TRACKER_TIME_SECONDS;
    static const std::string SCREEN_LX;
    static const std::string SCREEN_LY;
    static const std::string PUPIL_SIZE_L;
    static const std::string SCREEN_RX;
    static const std::string SCREEN_RY;
    static const std::string PUPIL_SIZE_R;
    static const std::string BLINK_L;
    static const std::string BLINK_R;
    static const std::string FIXATION_L;
    static const std::string FIXATION_R;
    static const std::string SACCADE_L;
    static const std::string SACCADE_R;
    static const std::string RAW_LX;
    static const std::string RAW_LY;
    static const std::string RAW_RX;
    static const std::string RAW_RY;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit TRACKPixxDevice(const ParameterValueMap &parameters);
    ~TRACKPixxDevice();
    
    void beginCalibration();
    void takeCalibrationSample(double screenX, double screenY, bool leftEye, bool rightEye);
    void endCalibration();
    void loadCalibration();
    
    void beginPupilSizeCalibration();
    void endPupilSizeCalibration();
    void loadPupilSizeCalibration();
    
protected:
    bool initializeDevice() override;
    bool configureDevice() override;
    bool startDevice() override;
    bool stopDevice() override;
    void readDeviceInputs(MWTime currentDeviceTimeNanos, MWTime currentTime) override;
    
private:
    struct TrackerScheduleFrame {
        double timeTag;        // (float)  Time stamp of DATAPixx time in seconds
        double leftScrPosX;    // (float)  Cartesian screen coordinate for x component of left eye in pixels
        double leftScrPosY;    // (float)  Cartesian screen coordinate for y component of left eye in pixels
        double leftPPSize;     // (float)  Diameter of left eye pupil in pixels
        double rightScrPosX;   // (float)  Cartesian screen coordinate for x component of right eye in pixels
        double rightScrPosY;   // (float)  Cartesian screen coordinate for y component of right eye in pixels
        double rightPPSize;    // (float)  Diameter of right eye pupil in pixels
        double digitalInput;   // (int)    Digital input state in base10
        double leftBlink;      // (bool)   Is left eye blinking?
        double rightBlink;     // (bool)   Is right eye blinking?
        double digitalOutput;  // (int)    Digital output state in base10
        double leftFixation;   // (bool)   Is left eye fixated?
        double rightFixation;  // (bool)   Is right eye fixated?
        double leftSaccade;    // (bool)   Is left eye saccading?
        double rightSaccade;   // (bool)   Is right eye saccading?
        double messageC;       // (int)    Integer event code sent to DPx (not yet implemented)
        double leftRawX;       // (float)  Pupil-to-corneal reflection vector x component for left eye
        double leftRawY;       // (float)  Pupil-to-corneal reflection vector y component for left eye
        double rightRawX;      // (float)  Pupil-to-corneal reflection vector x component for right eye
        double rightRawY;      // (float)  Pupil-to-corneal reflection vector y component for right eye
        double unused1;
        double unused2;
    };
    // Each frame has 22 elements, with the final 2 being undocumented and/or unused
    static_assert(sizeof(TrackerScheduleFrame) == sizeof(std::array<double, 22>));
    
    static constexpr MWTime trackerSamplingInterval = 500;  // TRACKPixx3 samples at 2kHz => 500us/sample
    
    bool setShowOverlay(const Datum &data);
    bool setDistance(const Datum &data);
    bool setTrackerWindowBrightness(const Datum &data);
    bool setLEDIntensity(const Datum &data);
    bool setIrisExpectedSize(const Datum &data);
    bool setFixationSpeed(const Datum &data);
    bool setFixationSamples(const Datum &data);
    bool setSaccadeSpeed(const Datum &data);
    bool setSaccadeSamples(const Datum &data);
    
    void handleFrame(const TrackerScheduleFrame &frame, MWTime currentDeviceTimeNanos, MWTime currentTime) const;
    
    const VariablePtr showOverlay;
    const VariablePtr distance;
    const unsigned int lensType;
    const VariablePtr trackerWindowBrightness;
    const VariablePtr ledIntensity;
    const VariablePtr irisExpectedSize;
    const VariablePtr fixationSpeed;
    const VariablePtr fixationSamples;
    const VariablePtr saccadeSpeed;
    const VariablePtr saccadeSamples;
    const unsigned char species;
    const unsigned char trackingMode;
    const VariablePtr trackerTimeSeconds;
    const VariablePtr screenLeftX;
    const VariablePtr screenLeftY;
    const VariablePtr pupilSizeLeft;
    const VariablePtr screenRightX;
    const VariablePtr screenRightY;
    const VariablePtr pupilSizeRight;
    const VariablePtr blinkLeft;
    const VariablePtr blinkRight;
    const VariablePtr fixationLeft;
    const VariablePtr fixationRight;
    const VariablePtr saccadeLeft;
    const VariablePtr saccadeRight;
    const VariablePtr rawLeftX;
    const VariablePtr rawLeftY;
    const VariablePtr rawRightX;
    const VariablePtr rawRightY;
    
    bool didInitialize;
    
    unsigned int trackerScheduleFrameSize;
    unsigned int trackerScheduleFrameBufferRAMAddress;
    unsigned int trackerScheduleFrameBufferRAMSize;
    unsigned int nextTrackerScheduleFrameBufferReadAddress;
    
    std::vector<TrackerScheduleFrame> trackerScheduleFrames;
    
    class ConfigurationNotification : public VariableNotification {
    public:
        using DevicePtr = boost::shared_ptr<TRACKPixxDevice>;
        using Callback = bool(TRACKPixxDevice::*)(const Datum &);
        
        static bool create(const DevicePtr &device, const VariablePtr &var, Callback callback);
        
        ConfigurationNotification(const DevicePtr &device, Callback callback) :
            weakDevice(device),
            callback(callback)
        { }
        
        void notify(const Datum &data, MWTime time) override;
        
    private:
        static bool call(const DevicePtr &device, Callback callback, const Datum &data) {
            return (device.get()->*callback)(data);
        }
        
        const boost::weak_ptr<TRACKPixxDevice> weakDevice;
        const Callback callback;
    };
    
};


END_NAMESPACE_MW


#endif /* TRACKPixxDevice_hpp */
