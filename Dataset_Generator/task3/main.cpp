#include "DL.h"
#include "HMAPBiman.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

/*
The randomizations that we need to do for this task:
-- Randomize the scale of the objects, ensuring the object specified in the instruction can be placed in one of the spots. -> DONE
-- Randomize the number/height of the table -> DONE
-- Randomize the orientation/position etc. as done in the task-01  -> DONE
-- Randomize the prompts -> DONE
*/

rai::String findSourceObj(rai::Configuration& C, arr source_pos);
void generateActuatedConfig(rai::Configuration& C, rai::Configuration& C2, rai::String source_obj);

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {

    std::unordered_map<int, arr> color;
    color[1]   = {1, 0, 0};
    color[2]   = {0, 1, 0};
    color[3]   = {0, 0, 1};
    color[4]   = {0, 1, 1};
    color[5]   = {1, 1, 0};
    color[6]   = {1, 0, 1};
    color[7]   = {1, 1, 1};

    std::unordered_map<int, std::string> color_name;
    color_name[1]    = "red";
    color_name[2]    = "green";
    color_name[3]    = "blue";
    color_name[4]    = "cyan";
    color_name[5]    = "yellow";
    color_name[6]    = "purple";
    color_name[7]    = "white";

    std::unordered_map<int, std::string> object_name;
    object_name[1] = "cylinder";    
    object_name[2] = "triangle";
    object_name[3] = "cube";

    int count = 4;
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3", "cam_frame_4"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;
    std::vector<std::string> configurations;

    std::string json_file = "../data/data.json";
    

    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task3/DL_task3_conf.g");
        
        cout << "Sample: " << i << endl;

        // Change the colors of the objects 
        int tbc  = rnd.num(1, 7);
        int cc  = rnd.num(1, 7);
        int tc = rnd.num(1, 7);
        int pc = rnd.num(1, 7);
        C.getFrame("table_top")->setColor(color[tc]);
        C.getFrame("table_leg_1")->setColor(color[tc]);
        C.getFrame("table_leg_2")->setColor(color[tc]);
        C.getFrame("table_leg_3")->setColor(color[tc]);
        C.getFrame("table_leg_4")->setColor(color[tc]);

        // Randomize the scene
        // Randomize objects
        int goal_obj = rnd.num(1,3);
        rai::Frame* toy_body = C.addFrame("toy_body", "table_top", "joint: rigid, contact: 1, mass: 1, visual: True "); double toy_scale = rnd.uni(2, 3); 
        toy_body->setShape_F(rai::ST_mesh, {toy_scale, toy_scale, toy_scale}, "../../../LIRA_models/simple/toy_body.stl");
        C.addFrame("tb_triangle", "toy_body")->setRelativePosition({-0.075*toy_scale, 0, 0}); C.addFrame("tb_cylinder", "toy_body")->setRelativePosition({0, 0, 0}); C.addFrame("tb_prism", "toy_body")->setRelativePosition({0.075*toy_scale, 0, 0});
        toy_body->setRelativePosition({rnd.uni(-0.3, 0.3), rnd.uni(-0.15, -0.1), 0.1});toy_body->setQuaternion({0.7, 0.7, 0, 0}); toy_body->setContact(1);
        rai::Frame* cylinder = C.addFrame("1", "table_top", "joint: rigid, contact: 1, mass: 1, visual: True ");double obj_scale = rnd.uni(2, 3); 
        cylinder->setShape_F(rai::ST_mesh, {obj_scale, obj_scale, obj_scale}, "../../../LIRA_models/simple/cylinder.stl");cylinder->setRelativePosition({rnd.uni(-0.4, -0.2), rnd.uni(-0.05, 0.15), 0.13});cylinder->setQuaternion({0.7, 0.7, 0, 0});cylinder->setContact(1);
        rai::Frame* triangle = C.addFrame("2", "table_top", "joint: rigid, contact: 1, mass: 1, visual: True "); 
        triangle->setShape_F(rai::ST_mesh, {obj_scale, obj_scale, obj_scale}, "../../../LIRA_models/simple/triangle.stl");triangle->setRelativePosition({rnd.uni(-0.2, 0.2), rnd.uni(-0.05, 0.15), 0.13});triangle->setQuaternion({0.7, 0.7, 0, 0});triangle->setContact(1);
        rai::Frame* prism = C.addFrame("3", "table_top", "joint: rigid, contact: 1, mass: 1, visual: True ");
        prism->setShape_F(rai::ST_mesh, {obj_scale, obj_scale, obj_scale}, "../../../LIRA_models/simple/cube.stl");prism->setRelativePosition({rnd.uni(0.2, 0.4), rnd.uni(-0.05, 0.15), 0.13});prism->setQuaternion({0.7, 0.7, 0, 0});prism->setContact(1);
        toy_body->setColor({color[tbc]}); triangle->setColor(color[tc]); prism->setColor(color[pc]); cylinder->setColor(color[cc]);
        arr s = {0, 0, 0.1}; 

        // Randomize table
        int tab_conf = rnd.num(1,4);
        rai::Frame* table = C.getFrame("table_top");
        switch(tab_conf){
            case 1: {
                double ang = rnd.uni(0.55, 0.85); table->setQuaternion({ang, 0, 0, ang});
                table->setPosition({rnd.uni(0.45, 0.55), rnd.uni(0.15, 0.45), rnd.uni(0.3, 0.6)});      
                break;}
            case 2: {
                double ang2 = rnd.uni(0.55, 0.85); table->setQuaternion({-ang2, 0, 0, -ang2});
                table->setPosition({rnd.uni(0.45, 0.55)*-1, rnd.uni(0.15, 0.45), rnd.uni(0.3, 0.6)});      
                break;}
            case 3: {
                double ang3 = rnd.uni(-0.15, 0.15); table->setQuaternion({0.99, 0, 0, ang3});
                table->setPosition({rnd.uni(-0.3, 0.3), rnd.uni(0.7, 0.8), rnd.uni(0.3, 0.6)});      
                break;}      
            case 4: {
                double ang4 = rnd.uni(-0.3, 0.3); table->setQuaternion({0.99, 0, 0, ang4});  
                cout << table->getPose() << endl; 
                table->setPosition({rnd.uni(-0.3, 0.3), rnd.uni(-0.3, -0.5), rnd.uni(0.3, 0.6)});
                break;}      
        }

        // Randomize robot
        // double qr = rnd.uni(-1.57, 1.57);
        // C.getFrame("l_panda_link0")->setQuaternion({qr, 0, 0, qr});
        
        // Randomize prompt
        int rnd_prompt = rnd.num(1,10);
        std::string goal_str = object_name[goal_obj];
        std::string prompt;
        switch (rnd_prompt) {
            case 1:
                prompt = "<point>\nPlace the " + goal_str + " in the appropriate slot within the shape sorter toy.";
                break;
            case 2: 
                prompt = "<point>\nInsert the " + goal_str + " into the right position inside the shape sorter toy.";
                break;
            case 3: 
                prompt = "<point>\nFit the " + goal_str + " into its proper spot in the shape sorter toy.";
                break;
            case 4: 
                prompt = "<point>\nDrop the " + goal_str + " into the matching hole inside the shape sorter toy.";
                break;
            case 5: 
                prompt = "<point>\nMove the " + goal_str + " into the correct opening within the shape sorter toy.";
                break;
            case 6: 
                prompt = "<point>\nPosition the " + goal_str + " in the right compartment of the shape sorter toy.";
                break;
            case 7: 
                prompt = "<point>\nPut the " + goal_str + " into the proper location inside the shape sorter toy.";
                break;
            case 8: 
                prompt = "<point>\nAlign and place the " + goal_str + " in its designated spot within the shape sorter toy.";
                break;
            case 9: 
                prompt = "<point>\nPut the " + goal_str + " into the matching section of the shape sorter toy.";
                break;
            case 10: 
                prompt = "<point>\nInsert the " + goal_str + " into the appropriate opening of the shape sorter toy.";
                break;
        }

        // Calculate source and goal pos
        arr goal_pose;
        if (goal_str == "triangle"){
            goal_pose = C.getFrame("tb_triangle")->getPose();
        } else if (goal_str == "cube"){
            goal_pose = C.getFrame("tb_prism")->getPose();
        } else if (goal_str == "cylinder"){
            goal_pose = C.getFrame("tb_cylinder")->getPose();
        }
        goal_pose += {0, 0, 0.1, 0, 0, 0, 0};
        arr source_pose = C.getFrame(std::to_string(goal_obj).c_str())->getPose();

        // Get the camera view
        DL dl(C, 0);
        std::vector<byteA> img_all;
        arr pts_all;
        std::string config_name = "task3_config_file_" + std::to_string(i) + ".g";
        dl.getCameraView(cam_names, img_all, pts_all, 0.6);

        dl.serialize_csv(pts_all, PATH + "task3_point_cloud_" + std::to_string(i) + ".csv");
        dl.serialize_config(C, PATH + config_name);

        int img_count = 0;
        for (const byteA img : img_all) {
            dl.serialize_img(img, PATH + "task3_img_" + std::to_string(i) + "_" + std::to_string(img_count) + ".png");
            img_count++;
        }


        prompts.push_back(prompt);
        pointCloudFiles.push_back("task3_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pose(0), source_pose(1), source_pose(2), source_pose(3), source_pose(4), source_pose(5), source_pose(6)});
        configurations.push_back(config_name);
        
        // Run the HMAP
        /*
        arr source_pos = {source_pose(0), source_pose(1), source_pose(2)};
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
        int waypoint_factor = 3;
        arr qF = goal_pose;
        arr q_obs = {};
        C2.setJointState(C.getFrame(source_obj)->getPose());
        HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, "cam_hmap", 0);
        hmap_biman.run();
        hmap_biman.displaySolution();
        */
    }
    
    nlohmann::json dataset;

    // Check if the file exists
    if (std::filesystem::exists(json_file)) {
        std::ifstream inputFile(json_file);
        if (inputFile.is_open()) {
            try {
                inputFile >> dataset;
                inputFile.close();
            } catch (const std::exception& e) {
                std::cerr << "Error parsing JSON file: " << e.what() << std::endl;
                return 1;
            }

            // Ensure dataset is an array
            if (!dataset.is_array()) {
                std::cerr << "Error: Existing JSON is not an array." << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error opening existing file for reading." << std::endl;
            return 1;
        }
    } else {
        // File does not exist, create a new array
        dataset = nlohmann::json::array();
    }

    // Append new entries to the dataset
    for (size_t i = 0; i < pointCloudFiles.size(); ++i) {
        nlohmann::json entry;
        entry["point_cloud"] = pointCloudFiles[i];
        entry["prompt"] = prompts[i];
        entry["source_pose"] = source_poses[i];
        entry["goal_pose"] = goal_poses[i];
        entry["configuration"] = configurations[i];
        dataset.push_back(entry);
    }

    // Write the updated JSON array back to the file
    std::ofstream outputFile(json_file);
    if (outputFile.is_open()) {
        outputFile << dataset.dump(4); // Pretty print with 4 spaces
        outputFile.close();
        std::cout << "JSON file updated successfully!" << std::endl;
    } else {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }
    
    return 0;
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
    source->setJoint(rai::JointType::JT_free, {-10, 10, -10, 10, -10, 10,-4,4,-4,4,-4,4,-4,4});
    source->parent->children.remove(source->parent->children.findValue(source));
    source->parent = 0;
    source->setParent(C.getFrame("world"));
    source->setContact(1);
    source->setPosition({0, 0, 0});

}