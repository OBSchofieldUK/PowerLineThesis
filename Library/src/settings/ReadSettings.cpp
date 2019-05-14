#include "inspec_lib/settings/ReadSettings.hpp"
#include "inspec_lib/settings/SettingStructs.hpp"

namespace{
    std::string SplitFilename (const std::string& str){
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
            if(ASR.HasMember("Start at image num")){
                dst.start_at_img_num = ASR["Start at image num"].GetUint();
            }
            if(ASR.HasMember("End at image num")){
                dst.end_at_img_num = ASR["End at image num"].GetUint();
            }
        }else{
            rapidjson::Value AirSimRecordingObject(rapidjson::kObjectType);

            rapidjson::Value rec_folder(rapidjson::StringRef(AirSimRecordingDefault.Recording_folder.c_str()));
            AirSimRecordingObject.AddMember("Recording_folder",rec_folder,doc.GetAllocator());
            
            rapidjson::Value Slow_Factor(AirSimRecordingDefault.SlowDownFactor);
            AirSimRecordingObject.AddMember("SlowDownFactor",Slow_Factor,doc.GetAllocator());

            rapidjson::Value StartImg(AirSimRecordingDefault.start_at_img_num);
            AirSimRecordingObject.AddMember("Start at image num",StartImg,doc.GetAllocator());

            rapidjson::Value EndImg(AirSimRecordingDefault.end_at_img_num);
            AirSimRecordingObject.AddMember("End at image num",EndImg,doc.GetAllocator());

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
            if(obj.HasMember("Image topic")){
                dst.Image_topic = obj["Image topic"].GetString();
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
            
            rapidjson::Value top(rapidjson::StringRef(Image_processing_node_Default.Image_topic.c_str()));
            Object.AddMember("Image topic",top,doc.GetAllocator());

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
                    if(dst.canny_filter_size % 2 == 0) dst.canny_filter_size++;
                }
                if(obj2.HasMember("Treshold low")){
                    dst.canny_treshold_low = obj2["Treshold low"].GetUint();
                }
                if(obj2.HasMember("Treshold high")){
                    dst.canny_treshold_high = obj2["Treshold high"].GetUint();
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

            // Segment Cut
            rapidjson::Value SCObject(rapidjson::kObjectType);

            rapidjson::Value scml(PLineDDefault.segcut_min_length);
            rapidjson::Value scss(PLineDDefault.segcut_step_size);
            rapidjson::Value scma(PLineDDefault.segcut_max_angle);

            SCObject.AddMember("Min length",scml,doc.GetAllocator());
            SCObject.AddMember("Max angle",scma,doc.GetAllocator());
            SCObject.AddMember("Step size",scss,doc.GetAllocator());

            Object.AddMember("Segment Cut",SCObject,doc.GetAllocator());


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
    void read(Proximity_Filter &dst){   
    }
    void read(Camera &dst){
        rapidjson::Document doc = readFile();
        dst = Camera_Default;
        if(doc.HasMember("Camera")){
            const rapidjson::Value& obj = doc["Camera"];
            if(obj.HasMember("Pixel width")){
                dst.pixel_width = obj["Pixel width"].GetUint();
            }
            if(obj.HasMember("Pixel height")){
                dst.pixel_height = obj["Pixel height"].GetUint();
            }
            if(obj.HasMember("Chip size(mm)")){
                dst.Chip_size_mm = obj["Chip size(mm)"].GetDouble();
                double angle = atan(dst.pixel_height/double(dst.pixel_width));
                dst.Chip_size_x = cos(angle)*dst.Chip_size_mm/1000.0;
                dst.Chip_size_y = sin(angle)*dst.Chip_size_mm/1000.0;
            }
            if(obj.HasMember("Focal length(mm)")){
                dst.focal_length_mm = obj["Focal length(mm)"].GetDouble();
                dst.d = dst.focal_length_mm/1000.0;
            }
            if(obj.HasMember("FOV")){
                dst.FOV = obj["FOV"].GetDouble(); 
            }
            if(obj.HasMember("Camera Position")){
                const rapidjson::Value& obj2 = obj["Camera Position"];
                if(obj2.HasMember("X")){
                    dst.XYZ_camTdrone[0] = obj2["X"].GetDouble();
                }
                if(obj2.HasMember("Y")){
                    dst.XYZ_camTdrone[1] = obj2["Y"].GetDouble();
                }
                if(obj2.HasMember("Z")){
                    dst.XYZ_camTdrone[2] = obj2["Z"].GetDouble();
                }
                if(obj2.HasMember("Roll")){
                    dst.RPY_camTdrone[0] = obj2["Roll"].GetDouble();
                }
                if(obj2.HasMember("Pitch")){
                    dst.RPY_camTdrone[1] = obj2["Pitch"].GetDouble();
                }
                if(obj2.HasMember("Yaw")){
                    dst.RPY_camTdrone[2] = obj2["Yaw"].GetDouble();
                }
            }
        }else{
            rapidjson::Value Object(rapidjson::kObjectType);

            rapidjson::Value pw(Camera_Default.pixel_width);
            rapidjson::Value ph(Camera_Default.pixel_height);
            rapidjson::Value fov(Camera_Default.FOV);
            rapidjson::Value cs(Camera_Default.Chip_size_mm);
            rapidjson::Value fl(Camera_Default.focal_length_mm);

            Object.AddMember("Pixel width",pw,doc.GetAllocator());
            Object.AddMember("Pixel height",ph,doc.GetAllocator());
            Object.AddMember("Chip size(mm)",cs,doc.GetAllocator());
            Object.AddMember("Focal length(mm)",fl,doc.GetAllocator());
            Object.AddMember("FOV",fov,doc.GetAllocator());

            rapidjson::Value pos(rapidjson::kObjectType);
            rapidjson::Value x(Camera_Default.XYZ_camTdrone[0]);
            rapidjson::Value y(Camera_Default.XYZ_camTdrone[1]);
            rapidjson::Value z(Camera_Default.XYZ_camTdrone[2]);
            rapidjson::Value roll(Camera_Default.RPY_camTdrone[0]);
            rapidjson::Value pitch(Camera_Default.RPY_camTdrone[1]);
            rapidjson::Value yaw(Camera_Default.RPY_camTdrone[2]);

            pos.AddMember("X",x,doc.GetAllocator());
            pos.AddMember("Y",y,doc.GetAllocator());
            pos.AddMember("Z",z,doc.GetAllocator());

            pos.AddMember("Roll",roll,doc.GetAllocator());
            pos.AddMember("Pitch",pitch,doc.GetAllocator());
            pos.AddMember("Yaw",yaw,doc.GetAllocator());
            Object.AddMember("Camera Position",pos,doc.GetAllocator());

            doc.AddMember("Camera",Object,doc.GetAllocator());
            saveFile(doc);

        }
        if(dst.Chip_size_mm <1 && dst.focal_length_mm < 1 && dst.FOV < 1){ // ALL missing
            std::cerr << "Not Enough Camera Settings plese specify eiterh FOV or ChipSize and focal Length" << std::endl;
            throw "ERROR";
        }else if(dst.Chip_size_mm < 1 && dst.focal_length_mm < 1 && dst.FOV > 1){ // Only FOV
            dst.focal_length_mm = 21;
            dst.d = dst.focal_length_mm/1000;
            dst.Chip_size_x = tan(M_PI*(dst.FOV/2)/180)*dst.d*2;
            dst.Chip_size_y = dst.Chip_size_x*(dst.pixel_height/double(dst.pixel_width));

        }else if(dst.Chip_size_mm > 1 && dst.focal_length_mm < 1 && dst.FOV > 1){ // FOV and chip size
            dst.d = dst.Chip_size_x/( tan(M_PI*(dst.FOV/2)/180) * 2);

        }else if(dst.Chip_size_mm < 1 && dst.focal_length_mm > 1 && dst.FOV > 1){ // FOV and Focal length
            dst.Chip_size_x = tan(M_PI*(dst.FOV/2)/180)*dst.d *2;
            dst.Chip_size_y = dst.Chip_size_x*(dst.pixel_height/dst.pixel_width);

        }else if(dst.Chip_size_mm > 1 && dst.focal_length_mm > 1 && dst.FOV < 1){ // Chip size and Focal length
            dst.FOV = (atan(dst.Chip_size_x/(dst.d*2.0))/M_PI)*2*180;
        }else if(dst.Chip_size_mm > 1 && dst.focal_length_mm > 1 && dst.FOV > 1){ // GOT Everything
        }else{
            std::cerr << "Not Sure what went wrong with the camera settings" << std::endl;
            throw "ERROR";
        }
        
    }
    void read(Kalman_LineXmove &dst){
        dst = Kalman_LineXmove_Default;
    }
    void read(Lidar &dst){
        rapidjson::Document doc = readFile();
        dst = Lidar_matcher_Default;
        if(doc.HasMember("Lidar")){
            const rapidjson::Value& obj = doc["Lidar"];
            if(obj.HasMember("Number of Segments")){
                dst.number_of_segments = obj["Number of Segments"].GetUint();
            }
            if(obj.HasMember("FOV vertical")){
                dst.segment_V_angle = obj["FOV vertical"].GetDouble();
            }
            if(obj.HasMember("FOV horizontal")){
                dst.segment_H_angle = obj["FOV horizontal"].GetDouble();
            }
            if(obj.HasMember("Lidar Position")){
                const rapidjson::Value& obj2 = obj["Lidar Position"];
                if(obj2.HasMember("X")){
                    dst.XYZ_lidarTcam[0] = obj2["X"].GetDouble();
                }
                if(obj2.HasMember("Y")){
                    dst.XYZ_lidarTcam[1] = obj2["Y"].GetDouble();
                }
                if(obj2.HasMember("Z")){
                    dst.XYZ_lidarTcam[2] = obj2["Z"].GetDouble();
                }
                if(obj2.HasMember("Roll")){
                    dst.RPY_lidarTcam[0] = obj2["Roll"].GetDouble();
                }
                if(obj2.HasMember("Pitch")){
                    dst.RPY_lidarTcam[1] = obj2["Pitch"].GetDouble();
                }
                if(obj2.HasMember("Yaw")){
                    dst.RPY_lidarTcam[2] = obj2["Yaw"].GetDouble();
                }
            }

        }else{
            rapidjson::Value Object(rapidjson::kObjectType);
            
            rapidjson::Value nos(Lidar_matcher_Default.number_of_segments);
            rapidjson::Value fovv(Lidar_matcher_Default.segment_V_angle);
            rapidjson::Value fovh(Lidar_matcher_Default.segment_H_angle);

            rapidjson::Value pos(rapidjson::kObjectType);
            rapidjson::Value x(Lidar_matcher_Default.XYZ_lidarTcam[0]);
            rapidjson::Value y(Lidar_matcher_Default.XYZ_lidarTcam[1]);
            rapidjson::Value z(Lidar_matcher_Default.XYZ_lidarTcam[2]);
            rapidjson::Value roll(Lidar_matcher_Default.RPY_lidarTcam[0]);
            rapidjson::Value pitch(Lidar_matcher_Default.RPY_lidarTcam[1]);
            rapidjson::Value yaw(Lidar_matcher_Default.RPY_lidarTcam[2]);

            pos.AddMember("X",x,doc.GetAllocator());
            pos.AddMember("Y",y,doc.GetAllocator());
            pos.AddMember("Z",z,doc.GetAllocator());

            pos.AddMember("Roll",roll,doc.GetAllocator());
            pos.AddMember("Pitch",pitch,doc.GetAllocator());
            pos.AddMember("Yaw",yaw,doc.GetAllocator());

            Object.AddMember("Number of Segments",nos,doc.GetAllocator());
            Object.AddMember("FOV vertical",fovv,doc.GetAllocator());
            Object.AddMember("FOV horizontal",fovh,doc.GetAllocator());
            Object.AddMember("Lidar Position",pos,doc.GetAllocator());
            
            doc.AddMember("Lidar",Object,doc.GetAllocator());
            saveFile(doc);
        }

        double min_angle = -dst.segment_H_angle*dst.number_of_segments/2 + dst.segment_H_angle/2;
        for(uint i = 0; i < dst.number_of_segments; i ++){
            dst.seg_angle.push_back(min_angle+dst.segment_H_angle*i);
        }

    }


    std::ostream& operator<<(std::ostream& os, const Camera& cam){
        os << "###### Camera ######" << std::endl;
        os << "Pixel: " << cam.pixel_width << " x " << cam.pixel_height << std::endl;
        os << "Chip:  " << cam.Chip_size_x << " x " << cam.Chip_size_y << " - size: " << cam.Chip_size_mm << std::endl;
        os << "d:     " << cam.d << std::endl;
        os << "FOV:   " << cam.FOV << std::endl;

    }
}