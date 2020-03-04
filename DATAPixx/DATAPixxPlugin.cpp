//
//  DATAPixxPlugin.cpp
//  DATAPixx
//
//  Created by Christopher Stawarz on 2/25/20.
//  Copyright © 2020 The MWorks Project. All rights reserved.
//

#include "DATAPixxDevice.hpp"
#include "DATAPixxBitOutputChannel.hpp"


BEGIN_NAMESPACE_MW


class DATAPixxPlugin : public Plugin {
    void registerComponents(boost::shared_ptr<ComponentRegistry> registry) override {
        registry->registerFactory<StandardComponentFactory, DATAPixxDevice>();
        registry->registerFactory<StandardComponentFactory, DATAPixxBitOutputChannel>();
    }
};


extern "C" Plugin* getPlugin() {
    return new DATAPixxPlugin();
}


END_NAMESPACE_MW
