#include "inspec_lib/settings/ReadSettings.hpp"
#include "inspec_lib/settings/SettingStructs.hpp"

namespace{
    std::string SplitFilename (const std::string& str)
    {
        std::size_t found = str.find_last_of("/\\");
        std::string path = str.substr(0,found);
        return path;
    }
    std::string getSettingsPath(void){
        std::string holder = ros::package::getPath("inspec_lib");
        return SplitFilename(holder) + "/settings.json";
    }
}


namespace settings{
    rapidjson::Document readFile(void){
        std::string path = getSettingsPath();
        std::ifstream file(path);
        if(!file){
            std::cout << "Unable to open file at: " << path  << std::endl;
            std::ofstream myfile(path);
            myfile << "{\n" << "    " << '"' << "Version" << '"' << ": 1\n" << "}";
            myfile.close();
            file.open(path);
            if(!file){
                std::cout << "Somthing Went terrably wrong" << std::endl;
                throw "Somthing Went terrably wrong";
            }
            std::cout << "FileCreated" << std::endl;
        }
        std::string str((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
        rapidjson::Document d;
        d.Parse(str.c_str());
        file.close();
        return d;
    }
    void saveFile(rapidjson::Document &doc){
        FILE* fp = fopen(getSettingsPath().c_str(), "w");
        char writeBuffer[65536];
        rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
        rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
        doc.Accept(writer);
        fclose(fp);
    }

    void read(AirSimRecording &dst){
        rapidjson::Document doc = readFile();
        if(doc.HasMember("AirSimRecording")){
            const rapidjson::Value& ASR = doc["AirSimRecording"];
            if(ASR.HasMember("Recording_folder")){
                dst.Recording_folder = ASR["Recording_folder"].GetString();
            }else{
                dst.Recording_folder = AirSimRecordingDefault.Recording_folder;
            }
            if(ASR.HasMember("SlowDownFactor")){
                dst.SlowDownFactor = ASR["SlowDownFactor"].GetDouble();
            }else{
                dst.SlowDownFactor = AirSimRecordingDefault.SlowDownFactor;
            }
        }else{
            rapidjson::Value AirSimRecordingObject(rapidjson::kObjectType);

            rapidjson::Value rec_folder(rapidjson::StringRef(AirSimRecordingDefault.Recording_folder.c_str()));
            AirSimRecordingObject.AddMember("Recording_folder",rec_folder,doc.GetAllocator());
            
            rapidjson::Value Slow_Factor(AirSimRecordingDefault.SlowDownFactor);
            AirSimRecordingObject.AddMember("SlowDownFactor",Slow_Factor,doc.GetAllocator());

            doc.AddMember("AirSimRecording",AirSimRecordingObject,doc.GetAllocator());

            dst = AirSimRecordingDefault;
            saveFile(doc); 
        }
    }
}