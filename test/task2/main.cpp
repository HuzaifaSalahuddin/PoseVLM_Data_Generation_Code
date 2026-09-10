#include "HMAPBiman.h"

/*----------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------MAIN------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    // Initialize Configuration and BotOp
    rai::Configuration C;
    C.addFile("../../src/config/task2/DL_task2_conf.g");  

    rai::Configuration C2;
    C2.addFile("../../src/config/task2/DL_task2_actuated_conf.g");  

    std::string target = "sphere";
    std::string interacted_target = "sphere";
    std::vector<std::string> tool_list = {};
    std::vector<std::string> gripper_list = {"l_l_gripper"};
    double filter = 1;
    std::string video_path = "video/config";
    int total_obstacle_count = 0;
    int waypoint_factor = 2;
    arr qF = {0.5, 0.2, 0.1, 1, 0, 0, 0};
    arr q_obs = {};
    C2.setJointState(C.getFrame(target.c_str())->getPose());
    HMAPBiman hmap_biman(C, C2, qF, q_obs, target, interacted_target, total_obstacle_count, tool_list, gripper_list, filter, video_path, waypoint_factor, false);
    hmap_biman.is_model_aval = false;
    hmap_biman.run();
    hmap_biman.displaySolution();

    return 0;
}