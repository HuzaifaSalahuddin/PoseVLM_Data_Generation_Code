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
    color_name[6]    = "pink";
    color_name[7]    = "white";

    int count = 3000; // To get 3000 samples!
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;

    std::string json_file = "../data/data.json";

    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task1/DL_task1_conf.g"); 
        
        cout << "Sample: " << i << endl;
        // Change the colors of the objects
        int bc = rnd.num(1, 7);
        C.getFrame("box")->setColor(color[bc]);
        
        int oc = 0;
        do{
            oc = rnd.num(1, 7);
        } while(oc == bc);
        C.getFrame("obstacle_0")->setColor(color[oc]);

        int boc = 0;
        do{
            boc = rnd.num(1, 7);
        } while(boc == bc || boc == oc);
        StringA all_frames = C.getFrameNames();
        for (const auto& frame : all_frames){
            if (frame.contains("shelf_book_")) {
                rai::Frame* obj = C.getFrame(frame);
                obj->setColor(color[boc]);
            }
        }

        int bc2 = 0;
        do{
            bc2 = rnd.num(1, 7);
        } while(bc2 == bc || bc2 == oc || bc2 == boc);
        C.getFrame("box_1")->setColor(color[bc2]);

        int bc3 = 0;
        do{
            bc3= rnd.num(1, 7);
        } while(bc3 == bc || bc3 == oc || bc3 == boc || bc3 == bc2);
        C.getFrame("box_2")->setColor(color[bc3]);

        // Randomize the box positions
        arr box_pos = C.getFrame("box")->getPosition();
        arr box_shift = {rnd.uni(-0.1, 0.1), rnd.uni(-0.1, 0.1), 0};
        box_pos += box_shift;
        C.getFrame("box")->setPosition(box_pos);

        
        // First remove the target object book position
        int a = rnd.num(2, 14);
        arr foo = C.getFrame(("shelf_book_" + std::to_string(a)).c_str())->getPose();
        arr foo_shift = {0.03, 0, 0, 0, 0, 0, 0};
        arr goal_pose = foo + foo_shift;
        arr source_pose = C.getFrame("box")->getPose();

        C.getFrame(("shelf_book_" + std::to_string(a)).c_str())->setPosition({0, 0, 10});
        C.getFrame(("shelf_book_" + std::to_string(a+1)).c_str())->setPosition({0, 0, 10});

        // Then remove the blocked book position
        int b = 0;
        do{
            b = rnd.num(3, 14);
        } while(abs(a-b) < 3);
        arr p = C.getFrame(("shelf_book_" + std::to_string(b)).c_str())->getPosition();
        arr s = {0.03, 0.15, 0};
        p = p + s;
        C.getFrame(("shelf_book_" + std::to_string(b)).c_str())->setPosition({0, 0, 10});
        C.getFrame(("shelf_book_" + std::to_string(b+1)).c_str())->setPosition({0, 0, 10});
        C.getFrame("obstacle_0")->setPosition(p);
        
        // Lastly remove a single book 
        int c = 0;
        do{
            c = rnd.num(2, 14);
        } while(abs(a-c) < 1 || abs(b-c) < 1);
        C.getFrame(("shelf_book_" + std::to_string(c)).c_str())->setPosition({0, 0, 10});
        goal_pose *= 1000;
        source_pose *= 1000;

        //C.view(true, "Task 1");
        // Initialize DL
        C.view(true);
        DL dl(C, true);
        
        // Get the camera view
        std::vector<byteA> img_all;
        arr pts_all;
        dl.getCameraView(cam_names, img_all, pts_all, 0.6);

        dl.serialize_csv(pts_all, PATH + "task1_point_cloud_" + std::to_string(i) + ".csv");

        std::string prompt = "Put the " + color_name[bc] + " book next to the " + color_name[boc] + " books. ";

        prompts.push_back(prompt);
        pointCloudFiles.push_back("task1_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pose(0), source_pose(1), source_pose(2), source_pose(3), source_pose(4), source_pose(5), source_pose(6)});

        //for (int j = 0; j < cam_names.size(); j++){
        //    dl.serialize_img(img_all[j], PATH + "task1_" + std::to_string(i) + "_" + cam_names[j]);
        //}
        //dl.serialize_txt(prompt, PATH + "prompt_" + std::to_string(i) + ".txt");
        //dl.serialize_txt(pos, PATH + "pose_" + std::to_string(i)  + ".txt");
    }

    nlohmann::json dataset;
    // Check if file exists and load existing data if it does
    if (std::filesystem::exists(json_file)) {
        std::ifstream inputFile(json_file);
        if (inputFile.is_open()) {
            inputFile >> dataset;
            inputFile.close();
            // Ensure dataset is an array to append to
            if (!dataset.is_array()) {
                std::cerr << "Error: Existing JSON is not an array." << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error opening existing file." << std::endl;
            return 1;
        }
    } else {
        // Create a new JSON array if file does not exist
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

    // Write updated JSON array back to the file
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



