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
    }
};


extern "C" Plugin* getPlugin() {
    return new DATAPixxPlugin();
}


END_NAMESPACE_MW
