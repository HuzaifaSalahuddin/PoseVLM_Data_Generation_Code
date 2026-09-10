#ifndef __DL_H
#define __DL_H

#include <BotOp/bot.h>
#include <Control/ShortPathMPC.h>
#include <Control/timingOpt.h>
#include <Optim/NLP_Solver.h>
#include <KOMO/manipTools.h>
#include <KOMO/pathTools.h>
#include <KOMO/skeleton.h>
#include <Kin/viewer.h>
#include <Kin/F_forces.h>
#include <Kin/forceExchange.h>
#include <OptiTrack/optitrack.h>
#include <chrono>
#include <thread>
#include <random>
#include <PathAlgos/ConfigurationProblem.h>
#include <PathAlgos/RRT_PathFinder.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Geo/depth2PointCloud.h>
#include <string>
#include <Kin/cameraview.h>
#include <Kin/frame.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <future>
#include <memory>
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <nlohmann/json.hpp>

class DL {
public:
    DL(rai::Configuration C, int verbose);
    void getCameraView(std::vector<std::string> cam_names, std::vector<byteA>& img_all, arr& pts_all, double filter = 0.1);
    void serialize_img(const byteA& img, const std::string& filename);
    void serialize_txt(const std::string& str, const std::string& filename);
    void serialize_txt(const arr pos, const std::string& filename);
    void serialize_csv(const arr& array, const std::string& filename);
    rai::Frame& addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat = {});
    void serialize_config(rai::Configuration& C, const std::string& filename);
    static arr eulerToQuaternionDeg(double rollDeg, double pitchDeg, double yawDeg);
private:
    rai::Configuration C;
    int verbose;
    void create_dir(const std::string& filepath);
    static inline double deg2rad(double deg);
};

#endif