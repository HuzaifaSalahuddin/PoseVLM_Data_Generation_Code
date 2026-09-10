#include "DL.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
  
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

    int count = 4;
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_1", "cam_frame_0", "cam_frame_2", "cam_frame_3"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;

    std::string json_file = "../data/data.json";
    

    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task2/DL_task2_conf.g"); 
        cout << "Sample: " << i << endl;

        // Change the colors of the objects 
        int sc = rnd.num(1, 7);
        C.getFrame("sphere")->setColor(color[sc]);
        C.getFrame("prism")->setColor(color[rnd.num(1, 7)]);
        C.getFrame("pyramid")->setColor(color[rnd.num(1, 7)]);

        C.getFrame("bin_large")->setColor(color[rnd.num(1, 7)]);
        C.getFrame("bin_small")->setColor(color[rnd.num(1, 7)]);

        // Randomize the scene
        arr table_pose = C.getFrame("table_leg")->getPose();
        arr pos_dif = {rnd.uni(-0.1, 0.1), rnd.uni(-0.1, 0.1), 0};
        arr table_pos = {table_pose(0), table_pose(1), table_pose(2)};
        table_pos += pos_dif;

        double ang = rnd.uni(0, 1.57);
        arr quat_new = {ang, 0, 0, ang};
        quat_new += {table_pose(3), table_pose(4), table_pose(5), table_pose(6)};
        C.getFrame("table_leg")->setPosition(table_pos);
        C.getFrame("table_leg")->setQuaternion(quat_new);

        arr bin_large_pos = C.getFrame("bin_large")->getPosition();
        bin_large_pos += pos_dif;
        C.getFrame("bin_large")->setPosition(bin_large_pos);

        arr bin_small_pos = C.getFrame("bin_small")->getPosition();
        bin_small_pos += pos_dif;
        C.getFrame("bin_small")->setPosition(bin_small_pos);

        arr goal_pose = C.getFrame("bin_large")->getPose();
        goal_pose += {0, 0, 0.08, 0, 0, 0, 0};
        arr source_pose = C.getFrame("sphere")->getPose();

        goal_pose *= 1000;
        source_pose *= 1000;
        C.view(true);
        // Initialize DL
        DL dl(C, true);
        
        // Get the camera view
        std::vector<byteA> img_all;
        arr pts_all;
        dl.getCameraView(cam_names, img_all, pts_all, 0.6);
        dl.serialize_csv(pts_all, PATH + "task2_point_cloud_" + std::to_string(i) + ".csv");
        std::string prompt = "Put the " + color_name[sc] + " sphere into one of the bins bellow.";
        
        prompts.push_back(prompt);
        pointCloudFiles.push_back("task2_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pose(0), source_pose(1), source_pose(2), source_pose(3), source_pose(4), source_pose(5), source_pose(6)});

        //for (int j = 0; j < cam_names.size(); j++){
        //    dl.serialize_img(img_all[j], PATH + std::to_string(i) + "_" + cam_names[j]);
        //}
        
        //dl.serialize_txt(prompt, PATH + "prompt_" + std::to_string(i) + ".txt");
        //dl.serialize_txt(pos, PATH + "pose_" + std::to_string(i)  + ".txt");
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



