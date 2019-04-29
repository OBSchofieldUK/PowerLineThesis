#ifndef READ_SETTINGS_HPP_
#define READ_SETTINGS_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <streambuf>
#include <cstdio>

#include "SettingStructs.hpp"

#include <rapidjson/document.h>
#include "rapidjson/filewritestream.h"
#include <rapidjson/writer.h>


#include <ros/package.h>


namespace settings{
    void read(AirSimRecording &dst);
    void read(PLineD &dst);
    void read(Image_processing_node &dst);

    rapidjson::Document readFile(void);
    void saveFile(rapidjson::Document &doc);
}

#endif
