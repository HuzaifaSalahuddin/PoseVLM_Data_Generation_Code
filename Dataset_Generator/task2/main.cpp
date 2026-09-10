#include "DL.h"

/*
The randomizations that we need to do for this task:
-- Include the cases where the object can be placed in both smaller and larger bins, but there is already an object there in either the larger or the smaller bin. So, placement should be done in the bin with appropriate volume! -> DONE
-- Randomize the scale of both bins and the objects, ensuring the object specified in the instruction can be placed in one of the bins. -> DONE
-- Randomize the number/height of the tables -> DONE
-- Randomize the orientation/position etc. as done in the task-01  -> DONE
-- Randomize the prompts -> DONE
*/

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
    object_name[1] = "sphere";
    object_name[2] = "prism";
    object_name[3] = "pyramid";

    int count = 1;
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3", "cam_frame_4"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;
    std::vector<std::string> configurations;

    std::string json_file = "../data/data.json";
    
    double mesh_scale = 0.001;

    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task2/DL_task2_conf.g");
        
        cout << "Sample: " << i << endl;

        // Change the colors of the objects 
        int sc  = rnd.num(1, 7);
        int pc  = rnd.num(1, 7);
        int pyc = rnd.num(1, 7);
        int blc = rnd.num(1, 7);
        int bsc = rnd.num(1, 7);
        int tc  = rnd.num(1, 7);
        C.getFrame("table_leg_1")->setColor(color[tc]);
        C.getFrame("table_top_1")->setColor(color[tc]);
        C.getFrame("table_leg_2")->setColor(color[tc]);
        C.getFrame("table_top_2")->setColor(color[tc]);
        C.getFrame("table_leg_3")->setColor(color[tc]);
        C.getFrame("table_top_3")->setColor(color[tc]);

        // Randomize the scene
        // Randomize objects
        int goal_obj = rnd.num(1,3);
        int t1 = rnd.num(1, 3); int t2 = rnd.num(1, 3); int t3 = rnd.num(1, 3);
        rai::Frame* sphere = C.addFrame("1", ("table_top_" + std::to_string(t1)).c_str(), "joint: rigid, contact: 1, mass: 1");
        sphere->setRelativePosition({rnd.uni(-0.1, 0), rnd.uni(-0.13, 0), 0.07}); sphere->setColor(color[sc]); sphere->setShape(rai::ST_sphere, {rnd.uni(0.1, 0.2), 0.05});
        rai::Frame* prism = C.addFrame("2", ("table_top_" + std::to_string(t2)).c_str(), "joint: rigid, contact: 1, mass: 1");
        prism->setRelativePosition({rnd.uni(0.05, 0.13), rnd.uni(0.05, 0.13), 0.04}); prism->setColor(color[pc]); prism->setShape(rai::ST_box, {rnd.uni(0.1, 0.2), 0.05, 0.05});
        rai::Frame* pyramid = C.addFrame("3", ("table_top_" + std::to_string(t3)).c_str(), "joint: rigid, contact: 1, mass: 1, visual: True ");
        pyramid->setRelativePosition({rnd.uni(-0.13, 0), rnd.uni(0.05, 0.13), 0.02}); pyramid->setColor(color[pyc]); pyramid->setShape_F(rai::ST_mesh, {rnd.uni(1*mesh_scale, 1.5*mesh_scale), rnd.uni(1*mesh_scale, 1.5*mesh_scale), rnd.uni(1*mesh_scale, 1.5*mesh_scale)}, "../../../LIRA_models/simple/pyramid_10_10.stl"); pyramid->setQuaternion({0.7, 0.7, 0, 0});
        arr s = {0, 0, 0.1};

        // Randomize table
        for(int tab = 1; tab <= 3; tab++){
            if(tab == t1 || tab == t2 || tab == t3){
                 arr pos_tab = {rnd.uni(-0.1, 0.1), (rnd.uni(-0.1, 0.1)+ rnd.num(0, 1)), rnd.uni(-0.15, 0.15)}; double quat = rnd.uni(0, 1.57); arr quat_tab = {quat, 0, 0, quat};
                C.getFrame(("table_leg_" + std::to_string(tab)).c_str())->setPosition(C.getFrame(("table_leg_" + std::to_string(tab)).c_str())->getPosition() + pos_tab);
                C.getFrame(("table_leg_" + std::to_string(tab)).c_str())->setQuaternion(quat_tab);
            } else if(rnd.uni(0, 1) < 0.8){
                delete C.getFrame(("table_top_" + std::to_string(tab)).c_str());
                delete C.getFrame(("table_leg_" + std::to_string(tab)).c_str());
            }
        }

        // Randomize bin
        int target_bin = 1;
        double large_bin_scale = rnd.uni(1, 3)*mesh_scale;
        double small_bin_scale = rnd.uni(1, 4)*mesh_scale;
        C.getFrame("bin_1")->setShape_F(rai::ST_mesh, {large_bin_scale, large_bin_scale, large_bin_scale}, "../../../LIRA_models/simple/cylinder_hollow_14_14.stl");
        C.getFrame("bin_2")->setShape_F(rai::ST_mesh, {small_bin_scale, small_bin_scale, small_bin_scale}, "../../../LIRA_models/simple/cylinder_hollow_7_7.stl");
        C.getFrame("bin_1")->setColor(color[blc]); C.getFrame("bin_2")->setColor(color[bsc]);
        arr pos_dif_bin_l = {rnd.uni(-0.2, 0.2), rnd.uni(-0.1, 0), 0};
        arr pos_dif_bin_s = {rnd.uni(-0.2, 0.2), rnd.uni(0, 0.2), 0};
        arr s_l = {-1*static_cast<int>(rnd.num(0, 1)), 0, 0}; arr s_s = {-1*static_cast<int>(rnd.num(0, 1)), 0, 0};
        C.getFrame("bin_1")->setPosition((C.getFrame("bin_1")->getPosition() + pos_dif_bin_l + s_l));
        C.getFrame("bin_2")->setPosition((C.getFrame("bin_2")->getPosition() + pos_dif_bin_s + s_s));
        if(small_bin_scale > 2) {
            int bi = rnd.num(1,2); if(bi == target_bin) target_bin = 2; int ob = rnd.num(1,3); if(ob == goal_obj && goal_obj == 1) goal_obj = rnd.num(2,3); if(ob == goal_obj && goal_obj != 1)  goal_obj = 1;
            C.getFrame(std::to_string(ob).c_str())->setPosition(C.getFrame(("bin_" + std::to_string(bi)).c_str())->getPosition() + s);
        }

        // Randomize robot
        double qr = rnd.uni(-1.57, 1.57);
        C.getFrame("l_panda_link0")->setQuaternion({qr, 0, 0, qr});

        // Randomize prompt
        int rnd_prompt = rnd.num(1,10);
        std::string goal_str = object_name[goal_obj];
        std::string prompt;
        switch (rnd_prompt) {
            case 1:
                prompt = "<point>\nFetch the " + goal_str +  " from the table and place it into one of the bins.";
                break;
            case 2: 
                prompt = "<point>\nRetrieve the " + goal_str  + " from the table and position it inside a bin.";
                break;
            case 3: 
                prompt = "<point>\nTransfer the " + goal_str  + " into a bin.";
                break;
            case 4: 
                prompt = "<point>\nPut the " + goal_str + " below into one of the bins.";
                break;
            case 5: 
                prompt = "<point>\nGrasp the " + goal_str + " from the table and place it into any of the bins.";
                break;
            case 6: 
                prompt = "<point>\nPick up the " + goal_str + " placed on table and drop it into one of the bins.";
                break;
            case 7: 
                prompt = "<point>\nGet the " + goal_str + " and move it inside one of the bins below.";
                break;
            case 8: 
                prompt = "<point>\nPut the " + goal_str + ", into one of the bins below.";
                break;
            case 9: 
                prompt = "<point>\nPlace the " + goal_str  + " into a bin.";
                break;
            case 10: 
                prompt = "<point>\nCould you move this " + goal_str + " into any of the bins below?";
                break;
        }

        
        // Calculate source and goal pos
        arr s2 = {0, 0, 0.1, 0, 0, 0, 0};
        arr goal_pose = C.getFrame(("bin_" + std::to_string(target_bin)).c_str())->getPose() + s2;
        arr source_pose = C.getFrame(std::to_string(goal_obj).c_str())->getPose();

        C.view(true);

        // Get the camera view
        DL dl(C, true);
        std::vector<byteA> img_all;
        arr pts_all;
        std::string config_name = "task2_config_file_" + std::to_string(i) + ".g";
        dl.getCameraView(cam_names, img_all, pts_all, 0.6);
        dl.serialize_csv(pts_all, PATH + "task2_point_cloud_" + std::to_string(i) + ".csv");
        dl.serialize_config(C, PATH + config_name);

        prompts.push_back(prompt);
        pointCloudFiles.push_back("task2_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pose(0), source_pose(1), source_pose(2), source_pose(3), source_pose(4), source_pose(5), source_pose(6)});
        configurations.push_back(config_name);
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



