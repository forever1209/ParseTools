#include "CameraPreprocess.h"
#include "ParseCamera.h"
#include <algorithm>
#include <chrono>
#include <regex>
namespace mach
{
    namespace tool
    {
        CameraPreprocess::CameraPreprocess()
        {
            m_strBasePath_ = "";
        }
        CameraPreprocess::~CameraPreprocess()
        {
            std::cout << "CameraPreprocess Release ." << std::endl;
        }
        bool CameraPreprocess::CheckCalibInfoExist(const std::string &file, const std::string &vechile_id)
        {
            m_strBasePath_ = file;
            bool l_bOk = CreatePathIfNotExist(file);
            if(!l_bOk)
            {
                std::cout << "CameraPreprocess , Create Folder Failed , Quit ." << std::endl;
                return false;
            }
            std::unordered_map<std::string,std::string> l_mapVehicleId = {
            {"car201" , "car_byd_han_00001"} ,
            {"car202" , "car_byd_han_00002"} ,
            {"car602" , "car_00602"}
            };
            if(l_mapVehicleId.count(vechile_id)>0)
            {
                std::string l_strFilePath = file + "calibresult/" + l_mapVehicleId[vechile_id];
                bool bIsExist = (0 == access(l_strFilePath.c_str(), F_OK));
                if(!bIsExist)
                {
                    std::cout << "CheckCalibInfoExist " << l_strFilePath << "Not Exist ." << std::endl;
                    return false;
                }
                PREPROCESSTYPE l_type = PREPROCESSTYPE::BYD_TYPE;
                if(vechile_id.find("car602") != std::string::npos)
                {
                    l_type = PREPROCESSTYPE::HS_TYPE;
                }
                return Preprocess(l_strFilePath , l_type);
            }
            return false;
        }
        bool CameraPreprocess::Preprocess(const std::string &path , const PREPROCESSTYPE & type)
        {
            auto l_vecUseFiles = GetAllFilesByCamlist(path);
            std::unordered_map<std::string , ParamsData> l_mapCameraParam;
            std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
            for(const auto & file : l_vecUseFiles)
            {
                auto l_strCameraName = file;
                std::regex pattern("_intrinsic\\.json");
                l_strCameraName = std::regex_replace(l_strCameraName, std::regex(pattern) , "");
                auto l_strCameraPath = path + "/camera_params/" + file;
                bool l_bOk = m_pCameraParser->Open(l_strCameraPath);
                if(l_bOk)
                {
                    l_mapCameraParam[l_strCameraName] = m_pCameraParser->GetParams();
                }
            }
            if(!l_mapCameraParam.empty())
            {
                if(type == PREPROCESSTYPE::BYD_TYPE)
                {
                    return PreprocessByd(l_mapCameraParam);
                }
                else
                {
                    return PreprocessHs(l_mapCameraParam);
                }
            }
            return false;
        }
        bool CameraPreprocess::PreprocessHs(const std::unordered_map<std::string, ParamsData> &params)
        {
            auto l_mapCameraParam = params;
            cv::Mat K_mat(3, 3, CV_64F);
            cv::Mat_<double> D_mat(1,4);
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            bool l_bOk = true;
            // TODO :REMOVE
            int index = 0;
            for(const auto & cameraParams : l_mapCameraParam)
            {
                std::cout<<"name is "<<cameraParams.first<<std::endl;
                index++;
                auto camintric = cameraParams.second.camera_data_;
                std::string l_strMapBinName = m_strBasePath_ + "preprocess/resize/" + cameraParams.first + "_map.bin";
                for (int i = 0; i < camintric.K.size(); ++i) 
                {
                    for (int j = 0; j < camintric.K[i].size(); ++j) 
                    {
                        K_mat.at<double>(i, j) = static_cast<double>(camintric.K[i][j]); // 将 long double 转换为 double
                        // std::cout << K_mat.at<double>(i, j) << " ";
                    }
                    // std::cout<< std::endl;
                }
                for (int i = 0; i < camintric.D.size(); ++i) 
                {
                    D_mat(0, i) = static_cast<double>(camintric.D[i]); // 将 long double 转换为 double 并填充到矩阵中
                    // std::cout << D_mat(0, i) ;
                }
                cv::Size dim(camintric.resolution_.width_,camintric.resolution_.height_);
                if(cameraParams.first.find("200") != std::string::npos)
                {
                    cv::Mat new_K;
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.5,dim,0.5);
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(704, 564));
                    cv::resize(map2, map_y, cv::Size(704, 564));
                    if(cameraParams.first.find("cam_front_200")!= std::string::npos)
                    {
                        cv::Rect roi_x(0, 406 - 256, map_x.cols, 256);
                        cv::Rect roi_y(0, 406 - 256, map_y.cols, 256);
                        map_x = map_x(roi_x);
                        map_y = map_y(roi_y);
                        // std::cout<<"cam_front_200 map_x \n"<<map_x<<" \n map_y \n"<<map_y<<std::endl;
                    }
                    else
                    {
                        // 裁剪图像
                        // cv::Rect roi(338 - 256, 0, 256, map_x.rows);
                        // map_x = map_x(roi);
                        // map_y = map_y(roi);
                        cv::Rect roi_x(0, 338 - 256, 704, 256);
                        map_x = map_x(roi_x);

                        cv::Rect roi_y(0, 338 - 256, 704, 256);
                        map_y = map_y(roi_y);
                    }
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,704,256);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"fisheye_allinone.txt");
                }
                if(cameraParams.first.find("120") != std::string::npos)
                {
                    cv::Mat new_K;
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.325,dim,1.0);
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(1408, 792));
                    cv::resize(map2, map_y, cv::Size(1408, 792));
                    // cv::Rect roi(0, 712-512, map_x.cols, 512);
                    // 裁剪 map_x 和 map_y
                    cv::Rect roi_x(0, 792 - 512, 1408, 512);
                    map_x = map_x(roi_x);

                    cv::Rect roi_y(0, 792 - 512, 1408, 512);
                    map_y = map_y(roi_y);
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"front_allinone.txt");

                    //
                    std::string l_strMapBinFov30Name = m_strBasePath_ + "preprocess/resize/" + cameraParams.first + "_sim_fov30_map.bin";
                    cv::Mat new_K_fov30;
                    cv::Mat map1_fov30, map2_fov30;
                    new_K_fov30 = sim_fov_intrinsic["fov30"];
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K_fov30, dim, CV_32F, map1_fov30, map2_fov30);
                    cv::Mat map_x_fov30, map_y_fov30;
                    cv::resize(map1_fov30, map_x_fov30, cv::Size(1408, 792));
                    cv::resize(map2_fov30, map_y_fov30, cv::Size(1408, 792));

                    cv::Rect roi_fov30(0, 536 - 512, 1408, 512);
                    // cv::Rect roi_fov30(0, 792 - 512, map_x.cols, 512);
                    // cv::Rect roi_fov30_x(0, 792 - 512, 1408, 512); // 定义感兴趣区域
                    map_x_fov30 = map_x_fov30(roi_fov30); // 裁剪 map_x
                    map_y_fov30 = map_y_fov30(roi_fov30);
                    WriteMapBinFile(l_strMapBinFov30Name,map_x_fov30,map_y_fov30,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinFov30Name,cameraParams.first+"_30","front_allinone.txt");
                }
                if(cameraParams.first.find("100") != std::string::npos)
                {
                    cv::Mat new_K = sim_fov_intrinsic["fov70"];
                    new_K.rowRange(0, 2) = new_K.rowRange(0, 2) / 2.0;
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(1408, 792));
                    cv::resize(map2, map_y, cv::Size(1408, 792));
                    // cv::Rect roi(0, 712-512, map_x.cols, 512);
                    // 裁剪 map_x 和 map_y
                    cv::Rect roi_x(0, 712 - 512, 1408, 512);
                    map_x = map_x(roi_x);
                    cv::Rect roi_y(0, 712 - 512, 1408, 512);
                    map_y = map_y(roi_y);
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"back_allinone.txt");
                }
            }
            return l_bOk;
        }
        bool CameraPreprocess::PreprocessByd(const std::unordered_map<std::string, ParamsData> &params)
        {
            auto l_mapCameraParam = params;
            cv::Mat K_mat(3, 3, CV_64F);
            cv::Mat_<double> D_mat(1,4);
            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            bool l_bOk = true;
            // TODO :REMOVE
            int index = 0;
            for(const auto & cameraParams : l_mapCameraParam)
            {
                std::cout<<"name is "<<cameraParams.first<<std::endl;
                index++;
                auto camintric = cameraParams.second.camera_data_;
                std::string l_strMapBinName = m_strBasePath_ + "preprocess/resize/" + cameraParams.first + "_map.bin";
                for (int i = 0; i < camintric.K.size(); ++i) 
                {
                    for (int j = 0; j < camintric.K[i].size(); ++j) 
                    {
                        K_mat.at<double>(i, j) = static_cast<double>(camintric.K[i][j]); // 将 long double 转换为 double
                        // std::cout << K_mat.at<double>(i, j) << " ";
                    }
                    // std::cout<< std::endl;
                }
                for (int i = 0; i < camintric.D.size(); ++i) 
                {
                    D_mat(0, i) = static_cast<double>(camintric.D[i]); // 将 long double 转换为 double 并填充到矩阵中
                    // std::cout << D_mat(0, i) ;
                }
                cv::Size dim(camintric.resolution_.width_,camintric.resolution_.height_);
                if(cameraParams.first.find("200") != std::string::npos)
                {
                    cv::Mat new_K;
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.5,dim,0.5);
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(704, 564));
                    cv::resize(map2, map_y, cv::Size(704, 564));
                    // 裁剪图像
                    // cv::Rect roi(338 - 256, 0, 256, map_x.rows);
                    // map_x = map_x(roi);
                    // map_y = map_y(roi);
                    cv::Rect roi_x(0, 338 - 256, 704, 256);
                    map_x = map_x(roi_x);

                    cv::Rect roi_y(0, 338 - 256, 704, 256);
                    map_y = map_y(roi_y);
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,704,256);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"fisheye_allinone.txt");
                }
                if(cameraParams.first.find("120") != std::string::npos)
                {
                    cv::Mat new_K;
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.325,dim,1.0);
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(1408, 792));
                    cv::resize(map2, map_y, cv::Size(1408, 792));
                    // cv::Rect roi(0, 712-512, map_x.cols, 512);
                    // 裁剪 map_x 和 map_y
                    cv::Rect roi_x(0, 712 - 512, 1408, 512);
                    map_x = map_x(roi_x);

                    cv::Rect roi_y(0, 712 - 512, 1408, 512);
                    map_y = map_y(roi_y);
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"front_allinone.txt");

                    //
                    std::string l_strMapBinFov30Name = m_strBasePath_ + "preprocess/resize/" + cameraParams.first + "_sim_fov30_map.bin";
                    cv::Mat new_K_fov30;
                    cv::Mat map1_fov30, map2_fov30;
                    new_K_fov30 = sim_fov_intrinsic["fov30"];
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K_fov30, dim, CV_32F, map1_fov30, map2_fov30);
                    cv::Mat map_x_fov30, map_y_fov30;
                    cv::resize(map1_fov30, map_x_fov30, cv::Size(1408, 792));
                    cv::resize(map2_fov30, map_y_fov30, cv::Size(1408, 792));
                    cv::Rect roi_fov30(0, 792 - 512, 1408, 512);
                    // cv::Rect roi_fov30(0, 792 - 512, map_x.cols, 512);
                    // cv::Rect roi_fov30_x(0, 792 - 512, 1408, 512); // 定义感兴趣区域
                    map_x_fov30 = map_x_fov30(roi_fov30); // 裁剪 map_x
                    map_y_fov30 = map_y_fov30(roi_fov30);
                    WriteMapBinFile(l_strMapBinFov30Name,map_x_fov30,map_y_fov30,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinFov30Name,cameraParams.first+"_30","front_allinone.txt");
                }
                if(cameraParams.first.find("100") != std::string::npos)
                {
                    cv::Mat new_K = sim_fov_intrinsic["fov70"];
                    new_K.rowRange(0, 2) = new_K.rowRange(0, 2) / 2.0;
                    cv::Mat map1, map2;
                    cv::fisheye::initUndistortRectifyMap(K_mat, D_mat, R, new_K, dim, CV_32F, map1, map2);
                    cv::Mat map_x, map_y;
                    cv::resize(map1, map_x, cv::Size(1408, 792));
                    cv::resize(map2, map_y, cv::Size(1408, 792));
                    // cv::Rect roi(0, 712-512, map_x.cols, 512);
                    // 裁剪 map_x 和 map_y
                    cv::Rect roi_x(0, 712 - 512, 1408, 512);
                    map_x = map_x(roi_x);
                    cv::Rect roi_y(0, 712 - 512, 1408, 512);
                    map_y = map_y(roi_y);
                    WriteMapBinFile(l_strMapBinName,map_x,map_y,1408,512);
                    l_bOk &= GenerateAllinoneMap(l_strMapBinName,cameraParams.first,"back_allinone.txt");
                }
            }
            return l_bOk;
        }
        std::vector<std::string> CameraPreprocess::GetAllFilesByCamlist(const std::string &path)
        {
            std::vector<std::string> files;
            if(m_vecCamera.empty())
            {
                return files;
            }
            DIR *dir;
            std::string l_strDirPath = path + "/" + "camera_params/";
            struct dirent *ent;
            if ((dir = opendir(l_strDirPath.c_str()))!= NULL) {
                while ((ent = readdir(dir))!= NULL) {
                    if (ent->d_type == DT_REG) {
                        std::string l_strCalibFile = ent->d_name;
                        auto it = std::find_if(m_vecCamera.begin(),m_vecCamera.end(),[&l_strCalibFile](const std::string & camera_name)
                        {
                            std::string l_strTempName = camera_name + "_intrinsic.json";
                            return l_strTempName==l_strCalibFile;
                        });
                        if(it!=m_vecCamera.end())
                        {
                            files.emplace_back(ent->d_name);
                        }
                    }
                }
                closedir(dir);
            } else {
                std::cerr << "Could not open directory " << path << std::endl;
            }            
            return files;
        }
        bool CameraPreprocess::CreatePathIfNotExist(const std::string &path)
        {
            std::string l_strPreprocessPath = path + "/" + "preprocess/";
            bool bIsExist = (0 == access(l_strPreprocessPath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strPreprocessPath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            std::string l_strResizePath = l_strPreprocessPath + "resize/";
            bIsExist &= (0 == access(l_strResizePath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strResizePath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            std::string l_strAllinonePath = l_strPreprocessPath + "table_allinone/";
            bIsExist &= (0 == access(l_strAllinonePath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strAllinonePath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            return bIsExist;
        }
        void CameraPreprocess::WriteMapBinFile(const std::string &name, const cv::Mat &map_x, const cv::Mat &map_y, const int &x, const int &y)
        {
            std::ofstream fh(name,std::ios::binary);
            if (!fh.is_open()) {
                std::cerr << "Failed to open " << name << "file for writing." << std::endl;
                return;
            }
            uint8_t buffer[sizeof(float)];
            for (int y_ = 0; y_ < y; ++y_) {
                for (int x_ = 0; x_ < x; ++x_) {
                    float value = map_x.at<float>(y_, x_);
                    ToLittleEndian(value, buffer);
                    fh.write(reinterpret_cast<char*>(&buffer), sizeof(float));
                }
            }
            for (int y_ = 0; y_ < y; ++y_) {
                for (int x_ = 0; x_ < x; ++x_) {
                    float value = map_y.at<float>(y_, x_);
                    ToLittleEndian(value, buffer);
                    fh.write(reinterpret_cast<char*>(&buffer), sizeof(float));
                }
            }
            fh.close();
        }
        bool CameraPreprocess::GenerateAllinoneMap(const std::string &map_bin, const std::string &map_name,const std::string & input_name)
        {
            std::string l_strAllinoneMapName = "";
            if(!m_mapCamMap.empty())
            {
                if(m_mapCamMap.count(map_name)>0)
                {
                    l_strAllinoneMapName = m_strBasePath_ + "preprocess/table_allinone/" + m_mapCamMap[map_name] + ".bin";
                }
            }
            std::string l_strBinPath = m_strBasePath_ + "../gwarp-tool-simple";
            std::string l_strChmod = "chmod +x " + l_strBinPath;
            system(l_strChmod.c_str());
            std::string l_strInputPath = m_strBasePath_ + "../input/" + input_name;
            if(!l_strAllinoneMapName.empty() && !map_bin.empty())
            {
                std::string l_strCmd = l_strBinPath + " -m " + map_bin + " -h " + l_strInputPath + " -b " + l_strAllinoneMapName + " > /dev/null";
                // std::cout<<"cmd is "<<l_strCmd<<std::endl;
                //gwarp-tool-simple -m ./resize/{key}_sim_fov70_map.bin -h input/back_allinone.txt -b ./table_allinone/{map_name}.bin > /dev/null
                system(l_strCmd.c_str());
                return true;
            }
            return false;
        }
        bool CameraPreprocess::IsLittleEndian()
        {
            uint16_t number = 1;
            return reinterpret_cast<uint8_t*>(&number)[0] == 1;
        }
        void CameraPreprocess::ToLittleEndian(float value, uint8_t *buffer)
        {
            std::memcpy(buffer, &value, sizeof(value));
            if (!IsLittleEndian()) {
                std::reverse(buffer, buffer + sizeof(value));
            }
        }
    } // namespace tool

} // namespace mach
