#include "YamlReWrite.h"
#include <yaml-cpp/yaml.h>
#include "ParseCamera.h"
#include "ParseEgo.h"
#include <ParseGnss.h>
#include <ParseLidar.h>
namespace mach
{
    namespace tool
    {
        YamlReWriteTool::YamlReWriteTool()
        {
            std::cout << "YamlReWriteTool Init " << std::endl;
        }
        YamlReWriteTool::~YamlReWriteTool()
        {
            std::cout << "YamlReWriteTool Release " << std::endl;
        }
        void YamlReWriteTool::CheckYamlFile(const std::string &filepath)
        {
            // std::cout<<"l_strConfig "<<filepath <<std::endl;
            std::cout<<"begin parse :" <<filepath<<std::endl;
            try {
                std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
                std::shared_ptr<mach::tool::BaseParse> m_pGnssParser = std::make_shared<mach::tool::GnssParse>(mach::tool::ParseFileType::GNSS_PARAMS);
                std::shared_ptr<mach::tool::BaseParse> m_pLidarParser = std::make_shared<mach::tool::LidarParse>(mach::tool::ParseFileType::LIDAR_PARAMS);
                YAML::Node config = YAML::LoadFile(filepath);
                std::string l_strCalibTime = "";
                auto l_strCameraName = config["pilot_perception_map_node"]["ros__parameters"]["cam_name"].as<std::string>();
                YAML::Node calib_result = config["pilot_perception_map_node"]["ros__parameters"]["calib_result"];
                YAML::Node cam_intr = calib_result["cam_intr"];
                YAML::Node lidar2cam = calib_result["lidar2cam"];
                YAML::Node lidar2ins = calib_result["lidar2ins"];
                YAML::Node ins2ego = calib_result["ins2ego"];
                YAML::Node lidar2ego = calib_result["lidar2ego"];
                auto distortion_model = calib_result["cam_intr"]["distortion_model"].as<std::string>();
                // std::cout<<"distortion_model "<<distortion_model<<" , camera name is "<<l_strCameraName<<std::endl;
                if(!m_mapParamsData_.empty())
                {
                    auto l_mapCalibCam = m_mapParamsData_["camera"];
                    for(const auto & file : (l_mapCalibCam.begin())->second)
                    {
                        if(file.find(l_strCameraName)!=std::string::npos)
                        {
                            std::string l_strFilePath = (l_mapCalibCam.begin())->first + "/" + file;
                            // std::cout<<"file name is "<<l_strFilePath<<std::endl;
                            bool l_bOK = m_pCameraParser->Open(l_strFilePath);
                            if(l_bOK)
                            {
                                auto l_stParseData = m_pCameraParser->GetParams();
                                if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::INTRINSIC)
                                {
                                    YAML::Node resolution = cam_intr["resolution"];
                                    YAML::Node K = cam_intr["K"];
                                    YAML::Node D = cam_intr["D"];
                                    resolution[0] = CompareData(l_stParseData.camera_data_.resolution_.width_,resolution[0].as<uint>())?resolution[0].as<uint>():l_stParseData.camera_data_.resolution_.width_;
                                    resolution[1] = CompareData(l_stParseData.camera_data_.resolution_.height_,resolution[1].as<uint>())?resolution[1].as<uint>():l_stParseData.camera_data_.resolution_.height_;
                                    int l_iIndexK = 0;
                                    for(const auto & data : l_stParseData.camera_data_.K)
                                    {
                                        for(const auto & k_data : data)
                                        {
                                            // std::cout<<"before k is "<< K[l_iIndexK].as<double>()<<" k_data is "<<k_data<<std::endl;
                                            K[l_iIndexK] = CompareDoubleData(k_data,K[l_iIndexK].as<long double>())?K[l_iIndexK].as<long double>():k_data;
                                            // std::cout<<"k is "<< K[l_iIndexK].as<double>()<<" k_data is "<<k_data<<std::endl;
                                            l_iIndexK++;
                                        }
                                    }
                                    cam_intr["K"] = K;
                                    int l_iIndexD = 0;
                                    for(const auto & d_data : l_stParseData.camera_data_.D)
                                    {
                                        // std::cout<<"before D is "<< D[l_iIndexD].as<double>()<<" d_data is "<<d_data<<std::endl;
                                        D[l_iIndexD] = CompareDoubleData(d_data,D[l_iIndexD].as<long double>())?D[l_iIndexD].as<long double>():d_data;
                                        // std::cout<<"D is "<< D[l_iIndexD].as<double>()<<" d_data is "<<d_data<<std::endl;
                                        l_iIndexD++;
                                    }
                                    cam_intr["D"] = D;
                                    // std::cout<<"resolution[0] "<<resolution[0].as<int>()<< " , resolution[1] "<<resolution[1].as<int>()<<std::endl;
                                }
                                if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::EXTRINSIC)
                                {
                                    YAML::Node t = lidar2cam["t"];
                                    YAML::Node r = lidar2cam["r"];
                                    {
                                        t[0] = CompareDoubleData(t[0].as<long double>(),l_stParseData.camera_data_.transform_.translation_.point_.x_)?t[0].as<long double>():l_stParseData.camera_data_.transform_.translation_.point_.x_;
                                        t[1] = CompareDoubleData(t[1].as<long double>(),l_stParseData.camera_data_.transform_.translation_.point_.y_)?t[1].as<long double>():l_stParseData.camera_data_.transform_.translation_.point_.y_;
                                        t[2] = CompareDoubleData(t[2].as<long double>(),l_stParseData.camera_data_.transform_.translation_.point_.z_)?t[2].as<long double>():l_stParseData.camera_data_.transform_.translation_.point_.z_;
                                    }
                                    {
                                        r[0] = CompareDoubleData(r[0].as<long double>(),l_stParseData.camera_data_.transform_.rotation_.point_.x_)?r[0].as<long double>():l_stParseData.camera_data_.transform_.rotation_.point_.x_;
                                        r[1] = CompareDoubleData(r[1].as<long double>(),l_stParseData.camera_data_.transform_.rotation_.point_.y_)?r[1].as<long double>():l_stParseData.camera_data_.transform_.rotation_.point_.y_;
                                        r[2] = CompareDoubleData(r[2].as<long double>(),l_stParseData.camera_data_.transform_.rotation_.point_.z_)?r[2].as<long double>():l_stParseData.camera_data_.transform_.rotation_.point_.z_;                                        
                                        r[3] = CompareDoubleData(r[3].as<long double>(),l_stParseData.camera_data_.transform_.rotation_.w_)?r[3].as<long double>():l_stParseData.camera_data_.transform_.rotation_.w_;
                                    }
                                    lidar2cam["t"] = t;
                                    lidar2cam["r"] = r;
                                }
                            }
                        }
                    }
                    auto l_mapCalibLidar = m_mapParamsData_["lidar"];
                    for(const auto & file : (l_mapCalibLidar.begin())->second)
                    {
                        std::string l_strFilePath = (l_mapCalibLidar.begin())->first + "/" + file;
                        bool l_bOK = m_pLidarParser->Open(l_strFilePath);
                        if(l_bOK)
                        {
                            auto l_stParseData = m_pLidarParser->GetParams();
                            if(l_stParseData.type_!=ParseFileType::LIDAR_PARAMS)
                            {
                                break;
                            }
                            if(file.find("lidar_gnss")!=std::string::npos)
                            {
                                YAML::Node t = lidar2ins["t"];
                                YAML::Node r = lidar2ins["r"];
                                {
                                    t[0] = CompareDoubleData(t[0].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.x_)?t[0].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                    t[1] = CompareDoubleData(t[1].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.y_)?t[1].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                    t[2] = CompareDoubleData(t[2].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.z_)?t[2].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.z_;
                                }
                                {
                                    r[0] = CompareDoubleData(r[0].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.x_)?r[0].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                    r[1] = CompareDoubleData(r[1].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.y_)?r[1].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                    r[2] = CompareDoubleData(r[2].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.z_)?r[2].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.z_;                                        
                                    r[3] = CompareDoubleData(r[3].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.w_)?r[3].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.w_;
                                }
                                lidar2ins["t"] = t;
                                lidar2ins["r"] = r;
                            }
                            if(file.find("lidar_ego")!=std::string::npos)
                            {
                                YAML::Node t = lidar2ego["t"];
                                YAML::Node r = lidar2ego["r"];
                                {
                                    t[0] = CompareDoubleData(t[0].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.x_)?t[0].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                    t[1] = CompareDoubleData(t[1].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.y_)?t[1].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                    t[2] = CompareDoubleData(t[2].as<long double>(),l_stParseData.lidar_data_.transform_.translation_.point_.z_)?t[2].as<long double>():l_stParseData.lidar_data_.transform_.translation_.point_.z_;
                                }
                                {
                                    r[0] = CompareDoubleData(r[0].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.x_)?r[0].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                    r[1] = CompareDoubleData(r[1].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.y_)?r[1].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                    r[2] = CompareDoubleData(r[2].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.point_.z_)?r[2].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.point_.z_;                                        
                                    r[3] = CompareDoubleData(r[3].as<long double>(),l_stParseData.lidar_data_.transform_.rotation_.w_)?r[3].as<long double>():l_stParseData.lidar_data_.transform_.rotation_.w_;
                                }
                                lidar2ego["t"] = t;
                                lidar2ego["r"] = r;                               
                            }
                        }
                    }
                
                    auto l_mapCalibGnss = m_mapParamsData_["gnss"];
                    for(const auto & file : (l_mapCalibGnss.begin())->second)
                    {
                        if(file.find("gnss_ego")!=std::string::npos)
                        {
                            std::string l_strFilePath = (l_mapCalibGnss.begin())->first + "/" + file;
                            bool l_bOK = m_pGnssParser->Open(l_strFilePath);
                            auto l_stParseData = m_pGnssParser->GetParams();
                            l_strCalibTime = l_stParseData.gnss_data_.calib_time_;
                            YAML::Node t = ins2ego["t"];
                            YAML::Node r = ins2ego["r"];
                            {
                                t[0] = CompareDoubleData(t[0].as<long double>(),l_stParseData.gnss_data_.transform_.translation_.point_.x_)?t[0].as<long double>():l_stParseData.gnss_data_.transform_.translation_.point_.x_;
                                t[1] = CompareDoubleData(t[1].as<long double>(),l_stParseData.gnss_data_.transform_.translation_.point_.y_)?t[1].as<long double>():l_stParseData.gnss_data_.transform_.translation_.point_.y_;
                                t[2] = CompareDoubleData(t[2].as<long double>(),l_stParseData.gnss_data_.transform_.translation_.point_.z_)?t[2].as<long double>():l_stParseData.gnss_data_.transform_.translation_.point_.z_;
                            }
                            {
                                r[0] = CompareDoubleData(r[0].as<long double>(),l_stParseData.gnss_data_.transform_.rotation_.point_.x_)?r[0].as<long double>():l_stParseData.gnss_data_.transform_.rotation_.point_.x_;
                                r[1] = CompareDoubleData(r[1].as<long double>(),l_stParseData.gnss_data_.transform_.rotation_.point_.y_)?r[1].as<long double>():l_stParseData.gnss_data_.transform_.rotation_.point_.y_;
                                r[2] = CompareDoubleData(r[2].as<long double>(),l_stParseData.gnss_data_.transform_.rotation_.point_.z_)?r[2].as<long double>():l_stParseData.gnss_data_.transform_.rotation_.point_.z_;                                        
                                r[3] = CompareDoubleData(r[3].as<long double>(),l_stParseData.gnss_data_.transform_.rotation_.w_)?r[3].as<long double>():l_stParseData.gnss_data_.transform_.rotation_.w_;
                            }
                            ins2ego["t"] = t;
                            ins2ego["r"] = r;
                        }
                    }
                }
                calib_result["cam_intr"] = cam_intr;
                calib_result["lidar2cam"] = lidar2cam;
                calib_result["lidar2ins"] = lidar2ins;
                calib_result["ins2ego"]   = ins2ego;
                calib_result["lidar2ego"] = lidar2ego;
                config["pilot_perception_map_node"]["ros__parameters"]["calib_result"] = calib_result;
                std::ofstream fout(filepath);
                fout << config;
                l_strCalibTime = "# " + l_strCalibTime;
                fout << std::endl << l_strCalibTime ;
                fout.close();
            }
            catch (const YAML::Exception& e) 
            {
                std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
            }
            std::cout<<filepath<<" done ."<<std::endl;
        }
        void YamlReWriteTool::SetParamInfoMap(const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> &map)
        {
            m_mapParamsData_ = map;
        }
    }
}
