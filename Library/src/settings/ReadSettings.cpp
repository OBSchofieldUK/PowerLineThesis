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
    std::string indentSpace(uint in){
        std::string result = "\n";
        if(in != 0){
            for(uint i = 0; i < in; i++){
                result += "    ";
            }
        }
        return result;
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

        std::string path = getSettingsPath();
        std::ifstream file(path);
        std::string str((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
        file.close();
        uint indent = 0;
        str+= " ";
        for(uint i = 0; i < str.length()-1; i++){
            if(str[i]=='{' && str[i+1] != '\n'){
                indent++;
                str.replace(i,1,"{"+indentSpace(indent));
            }else if(str[i]==',' && str[i+1] != '\n'){
                str.replace(i,1,","+indentSpace(indent));
            }else if(str[i]=='}' && str[i+1] != '\n'){
                indent--;
                if(str[i+1] == ',' || str[i+1]== '}'){
                    str.replace(i,1,indentSpace(indent)+"}"); 
                }else{
                    str.replace(i,1,indentSpace(indent)+"}"+indentSpace(indent));
                    
                }
                i += indentSpace(indent).length();    
            }
        }
        std::ofstream myfile(path);
        myfile << str;
        myfile.close();
        std::cout << "Done Saving" << std::endl;
    }

    void read(AirSimRecording &dst){
        rapidjson::Document doc = readFile();
        dst = AirSimRecordingDefault;
        if(doc.HasMember("AirSimRecording")){
            const rapidjson::Value& ASR = doc["AirSimRecording"];
            if(ASR.HasMember("Recording_folder")){
                dst.Recording_folder = ASR["Recording_folder"].GetString();
            }
            if(ASR.HasMember("SlowDownFactor")){
                dst.SlowDownFactor = ASR["SlowDownFactor"].GetDouble();
            }
        }else{
            rapidjson::Value AirSimRecordingObject(rapidjson::kObjectType);

            rapidjson::Value rec_folder(rapidjson::StringRef(AirSimRecordingDefault.Recording_folder.c_str()));
            AirSimRecordingObject.AddMember("Recording_folder",rec_folder,doc.GetAllocator());
            
            rapidjson::Value Slow_Factor(AirSimRecordingDefault.SlowDownFactor);
            AirSimRecordingObject.AddMember("SlowDownFactor",Slow_Factor,doc.GetAllocator());

            doc.AddMember("AirSimRecording",AirSimRecordingObject,doc.GetAllocator());

            
            saveFile(doc); 
        }
    }

    void read(Image_processing_node &dst){
        rapidjson::Document doc = readFile();
        dst = Image_processing_node_Default;
        if(doc.HasMember("Image Processing Node")){
            const rapidjson::Value& obj = doc["Image Processing Node"];
            if(obj.HasMember("Debug")){
                dst.debug = obj["Debug"].GetBool();
            }
            if(obj.HasMember("Show Incoming Image")){

                dst.show_incomming_image = obj["Show Incoming Image"].GetBool();
                
            }
            if(obj.HasMember("Show Final Image")){
                dst.show_final_image = obj["Show Final Image"].GetBool();
            }
            if(obj.HasMember("Press to continue")){
                dst.press_to_continue = obj["Press to continue"].GetBool();
            }
        }else{
            rapidjson::Value Object(rapidjson::kObjectType);

            rapidjson::Value debug(Image_processing_node_Default.debug);
            Object.AddMember("Debug",debug,doc.GetAllocator());

            rapidjson::Value show_incoming_img(Image_processing_node_Default.show_incomming_image);
            Object.AddMember("Show Incoming Image",show_incoming_img,doc.GetAllocator());

            rapidjson::Value show_final_img(Image_processing_node_Default.show_final_image);
            Object.AddMember("Show Final Image",show_final_img,doc.GetAllocator());

            rapidjson::Value ptc(Image_processing_node_Default.press_to_continue);
            Object.AddMember("Press to continue",ptc,doc.GetAllocator());

            doc.AddMember("Image Processing Node",Object,doc.GetAllocator());

            saveFile(doc);
        }
    }

    void read(PLineD &dst){
        rapidjson::Document doc = readFile();
        dst= PLineDDefault;
        if(doc.HasMember("PLineD")){
            const rapidjson::Value& obj = doc["PLineD"];
            if(obj.HasMember("Canny")){
                const rapidjson::Value& obj2 = obj["Canny"];
                if(obj2.HasMember("Filter size")){
                    dst.canny_filter_size = obj2["Filter size"].GetUint();
                }
                if(obj2.HasMember("Treshold Low")){
                    dst.canny_treshold_low = obj2["Treshold Low"].GetUint();
                }
                if(obj2.HasMember("Treshold High")){
                    dst.canny_treshold_high = obj2["Treshold High"].GetUint();
                }
            }
            if(obj.HasMember("Segment Cut")){
                const rapidjson::Value& obj2 = obj["Segment Cut"];
                if(obj2.HasMember("Min length")){
                    dst.segcut_min_length = obj2["Min length"].GetUint();
                }
                if(obj2.HasMember("Max angle")){
                    dst.segcut_max_angle = obj2["Max angle"].GetDouble();
                }
                if(obj2.HasMember("Step size")){
                    dst.segcut_step_size = obj2["Step size"].GetUint();
                }
            }
            if(obj.HasMember("Covariance ratio")){
                dst.covariance_ratio = obj["Covariance ratio"].GetDouble();
            }
            if(obj.HasMember("Segment grouping")){     
                const rapidjson::Value& obj2 = obj["Segment grouping"];
                if(obj2.HasMember("Min length finnished")){
                    dst.group_min_end_length = obj2["Min length finnished"].GetUint();
                }
                if(obj2.HasMember("Min length start")){
                    dst.group_min_start_length = obj2["Min length start"].GetUint();
                }
                if(obj2.HasMember("Max angle difference")){
                    dst.group_max_angle_dif = obj2["Max angle difference"].GetDouble();
                }
                if(obj2.HasMember("Max line distance")){
                    dst.group_max_line_dist = obj2["Max line distance"].GetUint();
                }
            }
            if(obj.HasMember("Parrallel line filter")){
                const rapidjson::Value& obj2 = obj["Parrallel line filter"];
                if(obj2.HasMember("Use filter")){
                    dst.Parrallel_active = obj2["Use filter"].GetBool();
                }
                if(obj2.HasMember("Max angle")){
                    dst.Parralel_max_angle = obj2["Max angle"].GetDouble();
                }
            }
            if(obj.HasMember("Debug")){
                dst.debug = obj["Debug"].GetBool();
            }
        }else{
            rapidjson::Value Object(rapidjson::kObjectType);


            // CANNY
            rapidjson::Value CObject(rapidjson::kObjectType);

            rapidjson::Value cfs(PLineDDefault.canny_filter_size);
            CObject.AddMember("Filter size",cfs,doc.GetAllocator());

            rapidjson::Value ctl(PLineDDefault.canny_treshold_low);
            CObject.AddMember("Treshold low",ctl,doc.GetAllocator());

            rapidjson::Value cth(PLineDDefault.canny_treshold_high);
            CObject.AddMember("Treshold high",cth,doc.GetAllocator());

            Object.AddMember("Canny",CObject,doc.GetAllocator());

            // COV
            rapidjson::Value CovR(PLineDDefault.covariance_ratio);
            Object.AddMember("Covariance ratio",CovR,doc.GetAllocator());

            // Group
            rapidjson::Value GObject(rapidjson::kObjectType);

            rapidjson::Value Gmlf(PLineDDefault.group_min_end_length);
            rapidjson::Value Gmls(PLineDDefault.group_min_start_length);
            rapidjson::Value Gmad(PLineDDefault.group_max_angle_dif);
            rapidjson::Value Gmld(PLineDDefault.group_max_line_dist);

            GObject.AddMember("Min length finnished",Gmlf,doc.GetAllocator());
            GObject.AddMember("Min length start",Gmls,doc.GetAllocator());
            GObject.AddMember("Max angle difference",Gmad,doc.GetAllocator());
            GObject.AddMember("Max line distance",Gmld,doc.GetAllocator());

            Object.AddMember("Segment grouping",GObject,doc.GetAllocator());
            
            // Parrallel

            rapidjson::Value PObject(rapidjson::kObjectType);
            rapidjson::Value Pa(PLineDDefault.Parrallel_active);
            rapidjson::Value Pma(PLineDDefault.Parralel_max_angle);

            PObject.AddMember("Use filter",Pa,doc.GetAllocator());
            PObject.AddMember("Max angle",Pma, doc.GetAllocator());

            Object.AddMember("Parrallel line filter", PObject,doc.GetAllocator());

            //Other
            rapidjson::Value debug(PLineDDefault.debug);
            Object.AddMember("Debug",debug,doc.GetAllocator());

            doc.AddMember("PLineD",Object,doc.GetAllocator());

            saveFile(doc);
        }
    }
}