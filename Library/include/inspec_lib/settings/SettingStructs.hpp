#ifndef SETTING_STRUCTS_HPP_
#define SETTING_STRUCTS_HPP_

#include <string>
#include <iostream>

namespace settings{
    struct AirSimRecording{
        std::string Recording_folder;
        double SlowDownFactor;
    };
    const AirSimRecording AirSimRecordingDefault = {" ",1.0};
    
    struct PLineD{

    };
}


#endif
