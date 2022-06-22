//
//  DATAPixxPlugin.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/25/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDevice.hpp"
#include "DATAPixxAnalogInputChannel.hpp"
#include "DATAPixxAnalogOutputChannel.hpp"
#include "DATAPixxBitInputChannel.hpp"
#include "DATAPixxBitOutputChannel.hpp"
#include "DATAPixxWordInputChannel.hpp"
#include "DATAPixxWordOutputChannel.hpp"
#include "TRACKPixxDevice.hpp"
#include "TRACKPixxActions.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxPlugin : public Plugin {
    void registerComponents(boost::shared_ptr<ComponentRegistry> registry) override {
        registry->registerFactory<StandardComponentFactory, DATAPixxDevice>();
        registry->registerFactory<StandardComponentFactory, DATAPixxAnalogInputChannel>();
        registry->registerFactory<StandardComponentFactory, DATAPixxAnalogOutputChannel>();
        registry->registerFactory<StandardComponentFactory, DATAPixxBitInputChannel>();
        registry->registerFactory<StandardComponentFactory, DATAPixxBitOutputChannel>();
        registry->registerFactory<StandardComponentFactory, DATAPixxWordInputChannel>();
        registry->registerFactory<StandardComponentFactory, DATAPixxWordOutputChannel>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxDevice>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxBeginCalibrationAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxTakeCalibrationSampleAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxEndCalibrationAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxLoadCalibrationAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxBeginPupilSizeCalibrationAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxEndPupilSizeCalibrationAction>();
        registry->registerFactory<StandardComponentFactory, TRACKPixxLoadPupilSizeCalibrationAction>();
    }
};


extern "C" Plugin* getPlugin() {
    return new DATAPixxPlugin();
}


END_NAMESPACE_MW
