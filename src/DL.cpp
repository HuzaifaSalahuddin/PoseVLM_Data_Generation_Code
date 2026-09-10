#include "DL.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
DL::DL(rai::Configuration C, int verbose){
    this->C = C;
    this->verbose = verbose;
    if (verbose > 0) C.view(true);
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void DL::getCameraView(std::vector<std::string> cam_names, std::vector<byteA>& img_all, arr& pts_all, double filter) {
    std::vector<rai::Frame*> pts_frames;
    std::unordered_map<int, arr> pts_color;
    arr pts_view;
    for(const auto& cam_name : cam_names){
        //cout << "Processing camera: " << cam_name << endl;
        rai::Configuration C_cam;
        C_cam.copy(C);
        arr pts;
        rai::CameraView V(C_cam, true);
        OpenGL imgGl;
        floatA depth;
        byteA segmentation;
        byteA img;
        
        
        rai::CameraView::Sensor sensor = V.addSensor(C.getFrame(cam_name.c_str()), 240, 240, 1.3, -1, {0.3, 5});
        V.computeImageAndDepth(img, depth);
        segmentation = V.computeSegmentationImage();

        uint target_frame_id = C.getFrame("plane")->ID;
        arr target_frame_color = id2color(target_frame_id);
        uintA seg_id = V.computeSegmentationID();

        uint target_r = (uint)(target_frame_color(0) * 256);
        uint target_g = (uint)(target_frame_color(1) * 256);
        uint target_b = (uint)(target_frame_color(2) * 256);

        // Mask the image and depth data
        byteA masked_img = img;
        floatA depth_masked = depth;
        for (uint i = 0; i < segmentation.d0; i++) {
            for (uint j = 0; j < segmentation.d1; j++) {
                for (uint c = 0; c < 3; c++) {
                    if (segmentation(i, j, c) % 2 != 0) {
                        segmentation(i, j, c) = segmentation(i, j, c) + 1; // same as x+1 if x is odd
                    }
                }   
                bool matches = true;
                for (uint c = 0; c < 3; c++) {
                    double val = target_frame_color(c); 
                    byte tcol = (byte)std::round(val * 255.0);
                    if (segmentation(i, j, c) != tcol) {
                        matches = false;
                        break;
                    }
                }
                if (matches) {
                    masked_img(i, j, 0) = 0;
                    masked_img(i, j, 1) = 0;
                    masked_img(i, j, 2) = 0;
                    depth_masked(i, j) = 0.0f;
                }
            }
        }

        depthData2pointCloud(pts, depth_masked, sensor.getFxycxy());

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        img_all.push_back(img);
        double foo = rnd.uni(0, 1000);
  
        for(uint i = 0; i < pts.d0; i++){
            for(uint j = 0; j < pts.d1; j++){
                if (dis(gen) <= filter) {
                    
                    std::string pts_name = "pts_" + cam_name;
                    addMarker(C_cam, {pts(i,j,0), pts(i,j,1), pts(i,j,2)}, pts_name.c_str(), cam_name.c_str(), 0.00000001, true);
                    arr pos = C_cam.getFrame(pts_name.c_str())->getPosition();
                    delete C_cam.getFrame(pts_name.c_str());

                    pts_view.append(pos);
    
                    double r = img(i,j,0);
                    double g = img(i,j,1);
                    double b = img(i,j,2);
                    
                    pos.append({r, g, b});
                    pts_all.append(pos);

                    int pts_id = color2id(segmentation.p + 3*(j + i*pts.d0));
                    pts_color[pts_id].append(pos);
                }
            }
        }

        if(verbose > 1){
            imgGl.text="image";  imgGl.watchImage(img, true);   
            imgGl.text="masked image";  imgGl.watchImage(masked_img, true); 
            //imgGl.text="depth";  imgGl.watchImage(depth, true);
            imgGl.text="segmentation";  imgGl.watchImage(segmentation, true);
        }

    }
    
    if(verbose > 0){
        rai::Configuration C_pt;
        for (const auto& pair : pts_color) {
            int key = pair.first;
            rai::Frame* pts_frame = C_pt.addFrame(std::to_string(key).c_str());
            arr p = pts_color[key];
            p.reshape(p.d0/6, 6);
            arr p1 = p.cols(0, 3);
            arr p2 = p.cols(3, 6);
            byteA p2_color;
            for(uint i = 0; i < p2.d0; i++){
                byte r = p2(i, 0);
                byte g = p2(i, 1);
                byte b = p2(i, 2);
                p2_color.append({r, g, b});
            }
            pts_frame->setPointCloud(p1, p2_color);
        }
        C_pt.view(true, "Point cloud");
        
 
    }

    pts_all.reshape(pts_all.d0/6, 6);
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void DL::serialize_img(const byteA& img, const std::string& filename) {
    create_dir(filename);
    int numChannels = img.d2;  // Adjust based on how your image stores channels
    // Save the color image as PNG
    std::string img_filename = filename + "_image.png";
    stbi_write_png(img_filename.c_str(), img.d1, img.d0, numChannels, img.p, img.d1 * numChannels);
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void DL::serialize_txt(const std::string& str, const std::string& filename) {
    create_dir(filename);
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << str;
        outFile.close();
    } 
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void DL::serialize_txt(const arr pos, const std::string& filename) {
    create_dir(filename);
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        for (uint i = 0; i < pos.d0; i++) {
            outFile << pos(i) << " ";
        }
        outFile.close();
    }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

void DL::serialize_config(rai::Configuration& C, const std::string& filename) {
    create_dir(filename);
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << C;
        outFile.close();
    }
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void DL::create_dir(const std::string& filepath) {
    std::string directory = filepath.substr(0, filepath.find_last_of('/'));
    mkdir(directory.c_str(), 0755); 
 }

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
void DL::serialize_csv(const arr& array, const std::string& filename) {
    create_dir(filename);
    std::ofstream file(filename);

    // Check if the file is open
    if (!file) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return;
    }

    int rows = array.d0;
    int cols = array.d1;
    
    if(array.d2 == 0){
        // Write the array data to the file
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                file << array(i,j); // Convert uint8_t to int for proper CSV output
                if(j < cols - 1){
                    file << ","; // Add comma between elements
                }
            }
            file << "\n"; // End of row
        }
    } else {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                for (int k = 0; k < array.d2; k++) {
                    file << array(i,j,k); // Convert uint8_t to int for proper CSV output
                    if(k < array.d2 - 1){
                        file << ","; // Add comma between elements
                    }
                }
                file << "\n"; // End of row
            }
        }
    }

    file.close();
}
/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/
rai::Frame& DL::addMarker(rai::Configuration& C, const arr pos, const std::string& name, const std::string& parent, double size, bool is_relative, arr quat) {
    
    rai::Frame* marker = C.addFrame(name.c_str(), parent.c_str());
    marker->setShape(rai::ST_marker, {size, size, size, size});
    if(is_relative) {
        marker->setRelativePosition(pos);
    } else {
        marker->setPosition(pos);
    }
    if(quat.N == 0) {
        quat = {1, 0, 0, 0};
    }
    marker->setQuaternion(quat); // w, x, y, z
    return *marker;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

inline double DL::deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

/*----------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------------------------*/

arr DL::eulerToQuaternionDeg(double rollDeg, double pitchDeg, double yawDeg) {
  // Convert degrees to radians (using the static inline helper)
  double roll  = deg2rad(rollDeg);
  double pitch = deg2rad(pitchDeg);
  double yaw   = deg2rad(yawDeg);

  // Half angles
  double cy = std::cos(yaw   * 0.5);
  double sy = std::sin(yaw   * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);


  double cr = std::cos(roll  * 0.5);
  double sr = std::sin(roll  * 0.5);

  // Quaternion formula (w, x, y, z)
  // Assuming roll -> X, pitch -> Y, yaw -> Z
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  return {w, x, y, z};
}