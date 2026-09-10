#include "HMAPBiman.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> split(const std::string& line, char delimiter);
std::string trim(const std::string& str);
rai::String findSourceObj(rai::Configuration& C, arr source_pos);
void generateActuatedConfig(rai::Configuration& C, rai::Configuration& C2, rai::String source_obj);

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    std::string filename = "../../output/model_output.csv";

    std::ifstream file(filename);

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> fields = split(line, ',');
        std::string file_name = trim(fields[0]);

        arr target_pose;
        std::vector<std::string> array7d_elements = split(trim(fields[1]), ';');
        for (const std::string& elem : array7d_elements) {
            target_pose.append(std::stod(trim(elem))); // Trim and convert to double
        }

        arr source_pos;
        std::vector<std::string> array3d_elements = split(trim(fields[2]), ';');
        for (const std::string& elem : array3d_elements) {
            source_pos.append(std::stod(trim(elem))); // Trim and convert to double
        }

        // Initialize Configuration and BotOp
        rai::Configuration C;
        cout << file_name << endl;
        C.addFile(("../../output/"+file_name).c_str());  

        rai::String source_obj = findSourceObj(C, source_pos);
        rai::Configuration C2;
        generateActuatedConfig(C, C2, source_obj);

        std::string target = static_cast<const char*>(source_obj);
        std::string interacted_target = target;
        std::vector<std::string> tool_list = {};
        std::vector<std::string> gripper_list = {"l_l_gripper"};
        double filter = 1;
        std::string video_path = "video/config";
        int total_obstacle_count = 0;
        int waypoint_factor = 2;
        arr qF = target_pose;
        arr q_obs = {};
        C2.setJointState(C.getFrame(source_obj)->getPose());
        cout << target_pose << endl;
        HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, true);
        hmap_biman.is_model_aval = false;
        hmap_biman.run();
        hmap_biman.displaySolution();
    }


    file.close();
    return 0;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------FUNCTIONS------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

std::vector<std::string> split(const std::string& line, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }
    return result;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    size_t last = str.find_last_not_of(" \t\n\r");
    return (first == std::string::npos || last == std::string::npos) ? "" : str.substr(first, last - first + 1);
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

rai::String findSourceObj(rai::Configuration& C, arr source_pos) {
    
    StringA all_frames = C.getFrameNames();
    for (const auto& frame : all_frames) {
        arr cand_obj_pos = C.getFrame(frame)->getPosition();
        double dist = std::sqrt(std::pow(cand_obj_pos(0) - source_pos(0), 2) +
                                std::pow(cand_obj_pos(1) - source_pos(1), 2) +
                                std::pow(cand_obj_pos(2) - source_pos(2), 2));
        if (dist < 0.1) {
            return frame;
        }
    }
    return "";
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void generateActuatedConfig(rai::Configuration& C, rai::Configuration& C2, rai::String source_obj){
    C2.copy(C);

    StringA all_frames = C2.getFrameNames();
     for (const auto& frame : all_frames) {
        if (frame.contains(rai::String("l_"))){
            delete C2.getFrame(frame);
        }
    }

    rai::Frame* source = C2.getFrame(source_obj);
    source->setJoint(rai::JointType::JT_free, {-10, 10, -10, 10, 0.4, 0.8,-4,4,-4,4,-4,4,-4,4});

    source->parent = C.getFrame("world");
    source->setContact(1);
    source->setPosition({0, 0, 0});

}