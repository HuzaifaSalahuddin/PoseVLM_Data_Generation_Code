#include "DL.h"
#include <cstdlib>
#include <string>
#include <iostream> 
#include <array>

using namespace std;


/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/


// In this file, the goal is to apply more randomization to positions, shapes etc. for the objects present in the scene

/*

WHAT IF INSTEAD OF ONE POINT, WE PRODUCE A CLUSTER OF POINTS THROUGH MODEL? AND THEN USE IT AS A GUIDANACE FOR FURTHER PREDICTION!

Here is the list of randomizations in my mind. But before that, let us try to add some more shelves on the table yaa'ani!
Okey tamam, I guess I am done with that Insha Allah. So, let us move forward!

--- For all the following randomizations, my advice to you would be to create functions!
(1) Randomize the position of entire setup (Books + Shelf_Table) -> Done Alhamdulillah!
(2) Randomize the position of books to be picked -> They shouldn't be always in one line! -> Done Alhamdulillah!
(3) Randomize the shape of objects to be picked and also change the object-to-pick from always being "box" -> Done being "box" thingy Alhamdulillah!
(4) Randomize the shape/scale/no. of shelves etc. of book shelf -> Done Alhamdulillah!
(5) Randomize the prompts -> Done Alhamdulillah!
(6) Deal with this plane thing! -> Done Alhamdulillah!

*/

/* Some Useful Functions */ 

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

 // Rotates the entire shelf-book setup around panda
int rotateShelf(rai::Configuration& C, const std::string& frame) {

    arr q;
    arr pos;
    arr shift;

    // Randomly decide on left, right, forward and backward orientations
    int randomIndex = rnd.num(1, 4);

    pos = C.getFrame(frame.c_str())->getPosition();
    q   = C.getFrame(frame.c_str())->getQuaternion();

    if (randomIndex == 1) {
         q = {-0.7071068, 0, 0, -0.7071068};
    } else if (randomIndex == 2) {
         q = {-1, 0, 0, 0};
    } else if (randomIndex == 3) {
         q = {-0.7071068, 0, 0, 0.7071068};
    } else if (randomIndex == 4) {
         q = {0, 0, 0, -1};
         shift = {0, 0.1, 0}; // To move it a lil' away from the robot
         pos+= shift;
    }
    
    C.getFrame(frame.c_str())->setPosition(pos);
    C.getFrame(frame.c_str())->setQuaternion(q);

    return randomIndex; // Return idx which contains orientation info
}

// Randomizes the no. of shelves of a book-shelf, returns the retained shelf/shelves
rai::String shelf_Setup(rai::Configuration& C)
{
    // Decide on the three options: (1) Remove top shelf (2) Remove lower shelf (3) Keep both of them (lil' more priority)
    int randomIndex = rnd.num(1, 4);
    rai::String to_remove;
    rai::String to_keep;
    if (randomIndex==1) {
        to_remove = "shelf_book_h";
        rai::Frame* obj = C.getFrame("shelf_center_h"); 
        to_keep = "shelf_book_l";
        delete obj;
    } else if (randomIndex==2) {
        // Incase of upper shelf only, the pos+size of forward wall of book shelf needs to be adjusted as well
        to_remove = "shelf_book_l";
        rai::Frame* obj = C.getFrame("shelf_center_l");
        arr pos = {0, -0.39, 0.41};
        arr size = {0.9, 0.03, 0.81, 0.5};
        rai::ShapeType shape = rai::ST_box;
        C.getFrame("shelf_front") -> setShape(shape, size).setRelativePosition(pos); // Adjust the height of shelf front
        to_keep = "shelf_book_h";
        delete obj;
    } else {
        // Randomly select one of the two shelves for creating hollow space to place book
        std::array<rai::String, 2> shelves = {"shelf_book_l", "shelf_book_h"};
        int to_keep = rnd.num(0, 1);
        return shelves[to_keep];
    }

    // Remove the selected shelf (if any)
    deleteObject(C, to_remove);

    return to_keep; 
}

// Randomizes the positions of to-pick-books placed on floor
arr randomize2Pick(arr& box_pos, int setup_ind, double lower_bound, double upper_bound) 
{
        arr box_shift;
        arr panda_pos = {0, 0.35, 0.05}; // panda pos obtained from .g file
        arr curr_pred;

        // Get the position shift to add, based on orientation info of overall shelf setup
        if (setup_ind == 1) {
            box_shift = {rnd.uni(0, -0.5), rnd.uni(-0.3, 0.3), 0};
        } else if (setup_ind == 2) {
            box_shift = {rnd.uni(-0.3, 0.3), rnd.uni(0, 0.5), 0};
        } else if (setup_ind == 3) {
            box_shift = {rnd.uni(0, 0.5), rnd.uni(-0.3, 0.3), 0};
        } else {
            box_shift = {rnd.uni(-0.3, 0.3), rnd.uni(0, -0.5), 0};
        }

        curr_pred = box_pos + box_shift;

        // Additional Check to verify books are not in collision with panda/ too far from panda
        double dist_squared = std::pow(curr_pred(0) - panda_pos(0), 2) + std::pow(curr_pred(1) - panda_pos(1), 2);
        if (dist_squared < std::pow(lower_bound, 2) || dist_squared > std::pow(upper_bound, 2)) { 
            return randomize2Pick(box_pos, setup_ind, lower_bound, upper_bound); // Keep on repeating until the position is within acceptable range!
        }   
    
        return curr_pred;
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

// Generate random prompts based on various templates
std::string randomizePrompts(std::string book, std::string shelf, bool is_onTable) // Change this string arg. later on, if need arises
{

   int num = rnd.num(1,14);
   std::string prompt;

   // For some of the templates, the word "table" is included, provided <is_onTable> is true for that particular sample
   switch (num) {
    case 1:
        prompt = "<point>\nCould you place the " + book + " book next to the " + shelf + " books? ";
        break; 
    case 2:
        prompt = "<point>\nPick up the " + book + " book and move it next to the " + shelf + " books. ";
        break; 
    case 3:
        if(is_onTable) {
            prompt = "<point>\nPick up the " + book + " book from table and place it next to " + shelf + " books. ";
        } else {
            return randomizePrompts(book, shelf, is_onTable);
        }
        break; 
    case 4:
        prompt = "<point>\nInsert the " + book + " book between the " + shelf + " books. ";
        break; 
    case 5:
        prompt = "<point>\nTake the " + book + " book and place it centrally within the row of " + shelf + " books. ";
        break;
    case 6:
        prompt = "<point>\nPosition the " + book + " book in the middle of " + shelf + " books. ";
        break;
    case 7:
        prompt = "<point>\nGrasp the " + book + " book and put it right in the center of the " + shelf + " books. ";
        break;
    case 8:
        prompt = "<point>\nPut the " + book + " book between the row of " + shelf + " books. ";
        break;
    case 9:
        if(is_onTable) {
            prompt = "<point>\nRetrieve the " + book + " colored book from the table and place it in the middle of " + shelf + " books. ";
        } else {
            return randomizePrompts(book, shelf, is_onTable);
        }
        
        break;
    case 10:
        if(is_onTable) {
            prompt = "<point>\nMove the " + book + " colored book on the table to a position right in the center of " + shelf + " books. ";
        } else {
            return randomizePrompts(book, shelf, is_onTable);
        }
        
        break;
    case 11:
        if(is_onTable) {
            prompt = "<point>\nGrasp the " + book + " book placed on table and position it in between the row of " + shelf + " books. ";
        } else {
            return randomizePrompts(book, shelf, is_onTable);
        }
        
        break;
    case 12:
        prompt = "<point>\nMove the " + book + " book so that it is in between the " + shelf + " books. ";
        break;
    case 13:
        if(is_onTable) {
            prompt = "<point>\nGet the " + book + " book from the table and place it next to " + shelf + " books. ";
        } else {
            return randomizePrompts(book, shelf, is_onTable);
        }
        
        break;
    case 14:
        prompt = "<point>\nPlace the " + book + " book next to " + shelf + " books. ";
    default:
        break;
}
    return prompt;

}

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

    int count = 1; // To get 3000 samples!
    std::string PATH = "../data/";
    std::vector<std::string> cam_names = {"cam_frame_0", "cam_frame_1", "cam_frame_2", "cam_frame_3", "cam_frame_4", "cam_frame_5" , "cam_frame_6", "cam_frame_7"};

    std::vector<std::string> pointCloudFiles;
    std::vector<std::string> prompts;
    std::vector<std::vector<double>> goal_poses;
    std::vector<std::vector<double>> source_poses;
    std::vector<std::string> configurations;
    std::string json_file = "../data/data.json";

    // Loop to generate samples
    for(int i= 0; i < count; i++){
        rai::Configuration C;
        C.addFile("../../src/config/task1/DL_task1_conf.g"); 
        
        cout << "Sample: " << i << endl;

        int setup_ind = rotateShelf(C, "shelf"); // Orient the shelf setup randomly

        std::string selected_shelf = std::string(shelf_Setup(C));
        
        // Change the colors of the books in shelves, books on floor etc.
        int bc = rnd.num(1, 7);
        C.getFrame("box")->setColor(color[bc]);
        
        int oc = 0;
        do{
            oc = rnd.num(1, 7);
        } while(oc == bc);
        C.getFrame("obstacle_0")->setColor(color[oc]);

        int lboc = 0;
        do{
            lboc = rnd.num(1, 7);
        } while(lboc == bc || lboc == oc);
        StringA all_frames = C.getFrameNames();
        for (const auto& frame : all_frames){
            if (frame.contains("shelf_book_l")) {
                rai::Frame* obj = C.getFrame(frame);
                obj->setColor(color[lboc]);
            }
        }

        int hboc = 0;
        do{
            hboc = rnd.num(1, 7);
        } while(hboc == bc || hboc == oc || hboc == lboc);
        for (const auto& frame : all_frames){
            if (frame.contains("shelf_book_h")) {
                rai::Frame* obj = C.getFrame(frame);
                obj->setColor(color[hboc]);
            }
        }

        // std::cin.get(); -> Use this to pause the code until a key is pressed
        int bc2 = 0;
        do{
            bc2 = rnd.num(1, 7);
        } while(bc2 == bc || bc2 == oc || bc2 == lboc || bc2 == hboc);
        C.getFrame("box_1")->setColor(color[bc2]);

        int bc3 = 0;
        do{
            bc3= rnd.num(1, 7);
        } while(bc3 == bc || bc3 == oc || bc3 == lboc || bc3 == bc2 || bc3 == hboc);
        C.getFrame("box_2")->setColor(color[bc3]);

        // Get the current non-randomized positions of books on floor
        arr box_pos = C.getFrame("box")->getPosition();
        arr box_1_pos = C.getFrame("box_1")->getPosition();
        arr box_2_pos = C.getFrame("box_2")->getPosition();

        // Get the randomized positions within certain bounds (euclidian dis. from panda) separately for each book, given a particular shelf setup.
        arr box_shift;
        double lower_bound = 0.40; // Maybe these values should be changed later on, these are not the perfect values to get good distr.
        double upper_bound = 0.80;
        
        box_pos = randomize2Pick(box_pos, setup_ind, lower_bound, upper_bound);
        box_1_pos = randomize2Pick(box_1_pos, setup_ind, lower_bound, upper_bound);
        box_2_pos = randomize2Pick(box_2_pos, setup_ind, lower_bound, upper_bound);
    
        // Apply the position-randomization to book frames
        C.getFrame("box")->setPosition(box_pos);
        C.getFrame("box_1")->setPosition(box_1_pos);
        C.getFrame("box_2")->setPosition(box_2_pos);

        // Apply a random table-elevation to one of the books on floor
        int rndnum = rnd.num(0, 2);
        string to_elevate;
        int src_color;
        bool is_onTable;
        if (rndnum==0) {
            to_elevate = "box";
            src_color = bc;
        } else if (rndnum==1) {
            to_elevate = "box_1";
            src_color = bc2;
        } else {
            to_elevate = "box_2";
            src_color = bc3;
        }
        is_onTable = table_Setup(C, to_elevate);

        // Introduce a frame that represents goal pose
        int a = rnd.num(2, 14);
        
        rai::ShapeType shape = rai::ST_marker;
        arr size = {0.0,0.0,0.0};
        rai::Frame* parent = C.getFrame("shelf");
        C.addFrame("goal")->setParent(parent).setPosition(size).setShape(shape, size);

        // Adjust the relative position of goal frame w.r.t base of shelf so that it is right close to the middle of empty space.
        arr spc_pos = C.getFrame((selected_shelf + std::to_string(a)).c_str())->getRelativePosition();
        arr spc_quat = C.getFrame((selected_shelf + std::to_string(a)).c_str())->getRelativeQuaternion();
        arr g = {spc_pos(0)+0.03, spc_pos(1), spc_pos(2)};

        // First make hollow space for target placement
        
        C.getFrame((selected_shelf + std::to_string(a)).c_str())->setPosition({0, 0, 10});
        C.getFrame((selected_shelf + std::to_string(a+1)).c_str())->setPosition({0, 0, 10});



        C.getFrame("goal")->setRelativePosition(g).setRelativeQuaternion(spc_quat);
        
        // Obtain the absolute pose of goal frame after all adjustments are completed
        arr goal_pose = C.getFrame("goal")->getPose();

        // Get the color of goal book row
        int g_clr; 
        if (selected_shelf=="shelf_book_l") {
            g_clr = lboc;
        }   else {
            g_clr = hboc;
        }

        // Randomly select one of the three books for picking
        arr source_pose;

        int randnum = rnd.num(0, 3);
        if (randnum==0) {
            source_pose = C.getFrame("box")->getPose();
            src_color = bc;
        } else if (randnum==1) {
            source_pose = C.getFrame("box_1")->getPose();
            src_color = bc2;
        } else if (randnum==2) {
            source_pose = C.getFrame("box_2")->getPose();
            src_color = bc3;
        } else {
            source_pose = C.getFrame(to_elevate.c_str())->getPose(); // To bias the selection towards elevated book
        }

        // Then make second space which will be blocked through obstacle
        int b = 0;
        do{
            b = rnd.num(3, 14);
        } while(abs(a-b) < 3);
        arr p = C.getFrame("obstacle_0")->getRelativePosition();
        arr r = C.getFrame((selected_shelf + std::to_string(b)).c_str())->getRelativePosition();
        arr q = C.getFrame((selected_shelf + std::to_string(b)).c_str())->getQuaternion();
        arr s = {r(0), p(1), r(2)};
        C.getFrame((selected_shelf + std::to_string(b)).c_str())->setPosition({0, 0, 10});
        C.getFrame((selected_shelf + std::to_string(b+1)).c_str())->setPosition({0, 0, 10});
        C.getFrame("obstacle_0")->setRelativePosition(s).setQuaternion(q); // Block this space
        
        // Lastly remove a single book from the selected book shelf randomly
        int c = 0;
        do{
            c = rnd.num(2, 14);
        } while(abs(a-c) < 1 || abs(b-c) < 1);
        C.getFrame((selected_shelf + std::to_string(c)).c_str())->setPosition({0, 0, 10});

        // C.view(true, "Task 1");
        // Initialize DL
        // C.view(true);
        DL dl(C, false);
        
        // Get the camera view
        std::vector<byteA> img_all;
        arr pts_all;
        std::string config_name = "task1_config_file_" + std::to_string(i) + ".g";
        dl.getCameraView(cam_names, img_all, pts_all, 0.6);
        dl.serialize_csv(pts_all, PATH + "task1_point_cloud_" + std::to_string(i) + ".csv");
        dl.serialize_config(C, PATH + config_name);
        // Randomize the prompts code in here!
        std::string prompt = randomizePrompts(color_name[src_color], color_name[g_clr], is_onTable);

        prompts.push_back(prompt);
        pointCloudFiles.push_back("task1_point_cloud_" + std::to_string(i) + ".npy");
        goal_poses.push_back({goal_pose(0), goal_pose(1), goal_pose(2), goal_pose(3), goal_pose(4), goal_pose(5), goal_pose(6)});
        source_poses.push_back({source_pose(0), source_pose(1), source_pose(2), source_pose(3), source_pose(4), source_pose(5), source_pose(6)});
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



