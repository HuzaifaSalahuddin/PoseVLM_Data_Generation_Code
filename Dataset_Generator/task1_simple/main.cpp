#include "DL.h"
#include <cstdlib>
#include <string>
#include <iostream> 
#include <array>

using namespace std;

// Removes all frames from the scene that contain a certain sub-string
void deleteObject(rai::Configuration& C, rai::String to_remove)
{
    StringA all_frames = C.getFrameNames();
        for (const auto& frame : all_frames){
            // Delete all the books
            if (frame.contains(to_remove)) {
                rai::Frame* obj = C.getFrame(frame);
                delete obj;
            }
        }

    return;
}

// Randomly Decides to elevate one of the books on floor through introduction of a table in the scene
bool table_Setup(rai::Configuration& C, const std::string& frame)
{
    
    // Decide on whether to have the table or not
    int randomIndex = rnd.num(0,1);
    rai::String table_frame = "table";
    bool is_onTable;
    if (randomIndex==0) {
        deleteObject(C, table_frame);
        is_onTable = false;

    } else {
        arr curr_table_pos = C.getFrame(table_frame)->getPosition();
        arr curr_book_pos  = C.getFrame(frame.c_str())->getPosition();
        double dwnshift = rnd.uni(0, -0.5);
        double height_adj = 0.12; // This is obtained from .g file (z book size)
        double height = 0.6 + dwnshift + height_adj;
        arr desired_tablepos = {curr_book_pos(0), curr_book_pos(1), dwnshift};
        arr desired_bookpos = {curr_book_pos(0), curr_book_pos(1), height};
        C.getFrame(table_frame)->setPosition(desired_tablepos); 
        C.getFrame(frame.c_str())->setPosition(desired_bookpos);
        is_onTable = true;
    }

    return is_onTable;
}

std::string randomizePrompts(std::string book, bool is_onTable) // Change this string arg. later on, if need arises
{
        // Randomize prompt
        int rnd_prompt = rnd.num(1,10);
        std::string prompt;
        switch (rnd_prompt) {
        case 1:
            if(is_onTable) {
            prompt = "<point>\nGet the book located on table and place it centrally on top of the shelf.";
            } else {
                return randomizePrompts(book, is_onTable);
            }
            break;
        case 2:
            prompt = "<point>\nPick up the " + book +  " and place it on top of the shelf.";
            break;
        case 3:
            if(is_onTable) {
            prompt = "<point>\nPick up the " + book + " placed on table and then place it on top of the shelf such that it is positioned at shelf's center.";
            } else {
                return randomizePrompts(book, is_onTable);
            }
            break;
        case 4:
            prompt = "<point>\nGrasp the " + book + " and position it at the center spot on the shelf.";
            break;
        case 5:
            prompt = "<point>\nKindly move the " + book + " to a position on top of the shelf.";
            break;
        case 6:
            prompt = "<point>\nPlease arrange the " + book + " so that it is centered on the shelf.";
            break;
        case 7:
            if(is_onTable) {
            prompt = "<point>\nRetrieve the book from the table and move it to the shelf’s midpoint.";
            } else {
                return randomizePrompts(book, is_onTable);
            }
            break;
        case 8:
            prompt = "<point>\nPlace the " + book + " carefully on top of shelf at its middle section.";
            break;
        case 9:
            prompt = "<point>\nMove the " + book + " so that it sits at the shelf’s exact center.";
            break;
        case 10:
            if(is_onTable) {
            prompt = "<point>\nGrab the " + book +  " from table and position it precisely in the middle section of the shelf.";
            } else {
                return randomizePrompts(book, is_onTable);
            }
            break;
        }

        return prompt;
}
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

    int count = 2000; // To get 2000 samples!
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3", "cam_frame_4", "cam_frame_5" , "cam_frame_6", "cam_frame_7", "cam_frame_8"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;
    std::vector<std::string> configurations;
    std::string json_file = "../data/data.json";

    // Loop to generate samples
    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task1_simple/DL_task1_simple_conf.g"); 
        cout << "Sample: " << i << endl;

        // Change the colors of the objects 
        int sc  = rnd.num(1, 7);
        int bc  = rnd.num(1, 7);
        
        C.getFrame("box")->setColor(color[bc]);
        C.getFrame("shelf_center_l")->setColor(color[sc]);
        C.getFrame("shelf_back")->setColor(color[sc]);
        C.getFrame("shelf_front")->setColor(color[sc]);
        

        // Randomize the shelf
        rai::Frame* shelf = C.getFrame("shelf");
        arr posD = {rnd.uni(-0.2, 0.2), rnd.uni(-0.2, 0.2), rnd.uni(0, -0.3)}; arr shelf_pos = shelf->getPosition();
        shelf_pos += posD; shelf->setPosition(shelf_pos);
        arr shelf_quat = DL::eulerToQuaternionDeg(0, 0.0, rnd.num(0, 60));shelf->setQuaternion(shelf_quat);

        // Randomize the box
        rai::Frame* box = C.getFrame("box");
        arr box_pos = box->getRelativePosition();
        
        // Modify the relative position
        arr size = {0.06, rnd.uni(0.15, 0.25), rnd.uni(0.20, 0.27), 0.5};
        box->setShape(rai::ST_box, size);
        box_pos(0) += rnd.uni(0, 0.5); // Randomize X
        box_pos(1) += rnd.uni(0, 0.5); // Randomize Y
        box_pos(2) = size(2)*0.5 - shelf_pos(2); // Set Z explicitly to make sure it is not below the ground

// -------------- ALWAYS USE () Brackets to access the elements of rai::Double Stuff!!

        box->setRelativePosition(box_pos);
        arr box_quat = DL::eulerToQuaternionDeg(0, 0.0, rnd.num(0, 60)); box->setQuaternion(box_quat);

        // Sometimes put the book on table
        bool is_onTable;
        is_onTable = table_Setup(C, "box");
        
        arr goal_pose = C.getFrame("goal")->getPose();
        arr source_pos = C.getFrame("box")->getPose();

        arr final_pos = box->getPosition();

        std::string goal_str = color_name[bc] + " book";
        std::string prompt;
        prompt = randomizePrompts(goal_str, is_onTable);
        // Initialize DL
        //C.view(false);
        DL dl(C, false);
        
        // Get the camera view
        double filter = 0.6;
        std::vector<byteA> img_all;
        arr pts_all;
        std::string config_name = "task1_simple_config_file_" + std::to_string(i) + ".g";
        dl.getCameraView(cam_names, img_all, pts_all, filter);
        dl.serialize_csv(pts_all, PATH + "task1_simple_point_cloud_" + std::to_string(i) + ".csv");
        dl.serialize_config(C, PATH + config_name);

        prompts.push_back(prompt);
        pointCloudFiles.push_back("task1_simple_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pos(0), source_pos(1), source_pos(2)});
        configurations.push_back(config_name);
        
    }

    // Save the data in JSON file after sample generation is fully done
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
        entry["configuration"] = configurations[i];
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
