#include "TomlReWrite.h"
#include "ParseCamera.h"
#include "ParseEgo.h"
#include <ParseGnss.h>
#include <ParseLidar.h>
namespace mach
{
    namespace tool
    {
        TomlReWriteTool::TomlReWriteTool()
        {
            m_mapParamsData_.clear();
            std::cout << "TomlReWriteTool Init " << std::endl;
        }
        TomlReWriteTool::~TomlReWriteTool()
        {
            std::cout << "TomlReWriteTool Release " << std::endl;
        }
        void TomlReWriteTool::CheckCalibInfoExist(const std::string & file,const std::string &name)
        {
            if(!name.empty())
            {
                std::cout<<"begin parse :" <<name<<std::endl;
                if("lidar_params.toml"==name)
                {
                    CompareLidar(file);
                    std::cout<<name<<" done ."<<std::endl;
                }
                if("allinone.toml"==name)
                {
                    CompareAllInOne(file);
                    std::cout<<name<<" done ."<<std::endl;
                }
                if("tracking.toml"==name)
                {
                    CompareTracking(file);
                    std::cout<<name<<" done ."<<std::endl;
                }
                if("obstacle_postfusion_params.toml"==name)
                {
                    //CompareObstaclePostfusion(file);
                    std::cout<<name<<" done ."<<std::endl;
                }
                if("static_obstacle_binder_params.toml"==name)
                {
                    CompareSaticObstacle(file);
                    std::cout<<name<<" done ."<<std::endl;
                }
                std::cout<<"-----------------------------------"<<std::endl;
            }
        }
        void TomlReWriteTool::SetParamInfoMap(const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> &map)
        {
            m_mapParamsData_ = map;
        }
        void TomlReWriteTool::CompareAllInOne(const std::string &path)
        {
            auto l_tmData = toml::parse(path);
            std::string l_strCalibTime = "";
            auto &l_tmCalibrationParam = toml::find(l_tmData, "calibration_parameters");
            std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
            std::shared_ptr<mach::tool::BaseParse> m_pGnssParser = std::make_shared<mach::tool::GnssParse>(mach::tool::ParseFileType::GNSS_PARAMS);
            for(const auto &calibPair :l_tmCalibrationParam.as_table())
            {
                if(calibPair.first.find("cam")!=std::string::npos)
                {
                    auto &l_tmCamParams = toml::find(l_tmCalibrationParam, calibPair.first.c_str());
                    if(!m_mapParamsData_.empty())
                    {
                        auto l_mapCalibCam = m_mapParamsData_["camera"];
                        for(const auto & file : (l_mapCalibCam.begin())->second)
                        {
                            if(file.find(calibPair.first)!=std::string::npos)
                            {
                                std::string l_strFilePath = (l_mapCalibCam.begin())->first + "/" + file;
                                bool l_bOK = m_pCameraParser->Open(l_strFilePath);
                                if(l_bOK)
                                {
                                    auto l_stParseData = m_pCameraParser->GetParams();
                                    if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::INTRINSIC)
                                    {
                                        l_tmCamParams["image_width"] = CompareData(toml::find<uint>(l_tmCamParams,"image_width"),l_stParseData.camera_data_.resolution_.width_)?toml::find<uint>(l_tmCamParams,"image_width") :  l_stParseData.camera_data_.resolution_.width_;
                                        l_tmCamParams["image_height"] = CompareData(toml::find<uint>(l_tmCamParams,"image_height"),l_stParseData.camera_data_.resolution_.height_)?toml::find<uint>(l_tmCamParams,"image_height") : l_stParseData.camera_data_.resolution_.height_;
                                        l_tmCamParams["distortion_model"] = CompareData(toml::find<std::string>(l_tmCamParams,"distortion_model"),l_stParseData.camera_data_.distortion_model_)?toml::find<std::string>(l_tmCamParams,"distortion_model") : l_stParseData.camera_data_.distortion_model_;
                                        auto &l_tmCamMatrix = toml::find(l_tmCamParams, "camera_matrix");
                                        if(toml::find<uint>(l_tmCamMatrix, "rows")>0&&l_stParseData.camera_data_.K.size()==toml::find<uint>(l_tmCamMatrix, "rows")&&l_stParseData.camera_data_.K[0].size()==toml::find<uint>(l_tmCamMatrix, "cols"))
                                        {
                                            auto &l_tmCamMatrixData = toml::find(l_tmCamMatrix, "data");
                                            int l_iDataIndex = 0;
                                            for(const auto & matrixs : l_stParseData.camera_data_.K)
                                            {
                                                for(const auto & matrix : matrixs)
                                                {
                                                    // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << matrix << " " << std::endl;
                                                    l_tmCamMatrixData.at(l_iDataIndex) = CompareDoubleData(matrix,toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)):matrix;
                                                    l_iDataIndex++;
                                                }
                                            }
                                        }
                                        auto &l_tmCamDistortion = toml::find(l_tmCamParams, "distortion_coefficients");
                                        if(toml::find<uint>(l_tmCamDistortion, "cols")>0 && toml::find<uint>(l_tmCamDistortion, "cols")==l_stParseData.camera_data_.D.size())
                                        {
                                            auto &l_tmCamDistortionData = toml::find(l_tmCamDistortion, "data");
                                            int l_iDataIndex = 0;
                                            for(const auto &data : l_stParseData.camera_data_.D)
                                            {
                                                // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << data << " " << std::endl;
                                                l_tmCamDistortionData.at(l_iDataIndex) = CompareDoubleData(data,toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)):data;
                                                l_iDataIndex++;
                                            }
                                        }
                                    }
                                    if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::EXTRINSIC)
                                    {
                                        auto &l_tmRfuCamMatrix = toml::find(l_tmCamParams, "cam_rfu_matrix"); 
                                        auto &l_tmCameraRotation = toml::find(l_tmRfuCamMatrix, "rotation"); 
                                        auto &l_tmCameraTranslation = toml::find(l_tmRfuCamMatrix, "translation"); 
                                        if(l_tmCameraRotation.size()==4)
                                        {
                                            l_tmCameraRotation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.w_,toml::get<long double>(l_tmCameraRotation.at(0)))?toml::get<long double>(l_tmCameraRotation.at(0)):l_stParseData.camera_data_.transform_.rotation_.w_;
                                            l_tmCameraRotation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmCameraRotation.at(1)))?toml::get<long double>(l_tmCameraRotation.at(1)):l_stParseData.camera_data_.transform_.rotation_.point_.x_;
                                            l_tmCameraRotation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmCameraRotation.at(2)))?toml::get<long double>(l_tmCameraRotation.at(2)):l_stParseData.camera_data_.transform_.rotation_.point_.y_;
                                            l_tmCameraRotation.at(3) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmCameraRotation.at(3)))?toml::get<long double>(l_tmCameraRotation.at(3)):l_stParseData.camera_data_.transform_.rotation_.point_.z_;
                                        }
                                        if(l_tmCameraTranslation.size()==3)
                                        {
                                            l_tmCameraTranslation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmCameraTranslation.at(0)))?toml::get<long double>(l_tmCameraTranslation.at(0)):l_stParseData.camera_data_.transform_.translation_.point_.x_;
                                            l_tmCameraTranslation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmCameraTranslation.at(1)))?toml::get<long double>(l_tmCameraTranslation.at(1)):l_stParseData.camera_data_.transform_.translation_.point_.y_;
                                            l_tmCameraTranslation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmCameraTranslation.at(2)))?toml::get<long double>(l_tmCameraTranslation.at(2)):l_stParseData.camera_data_.transform_.translation_.point_.z_;
                                        }
                                    }
                                    // std::cout<<"l_stParseData : "<<l_stParseData<<std::endl;
                                }
                                else
                                {
                                    std::cout<<"parse file "<<l_strFilePath<<" error "<<std::endl;
                                }
                            }
                        }
                        
                    }
                }
                if(calibPair.first.find("gnss")!=std::string::npos)
                {
                    if(!m_mapParamsData_.empty())
                    {
                        auto l_mapCalibGnss = m_mapParamsData_["gnss"];
                        for(const auto & file : (l_mapCalibGnss.begin())->second)
                        {
                            if(file.find(calibPair.first)!=std::string::npos)
                            {
                                std::string l_strFilePath = (l_mapCalibGnss.begin())->first + "/" + file;
                                bool l_bOK = m_pGnssParser->Open(l_strFilePath);
                                auto l_stParseData = m_pGnssParser->GetParams();
                                l_strCalibTime = l_stParseData.gnss_data_.calib_time_;
                                if(l_bOK)
                                {
                                    auto &l_tmGnssParams = toml::find(l_tmCalibrationParam, "gnss_ego");
                                    auto &l_tmGnssRotation = toml::find(l_tmGnssParams, "rotation"); 
                                    auto &l_tmGnssTranslation = toml::find(l_tmGnssParams, "translation");
                                    if(l_tmGnssRotation.size()==4)
                                    {
                                        l_tmGnssRotation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.w_,toml::get<long double>(l_tmGnssRotation.at(0)))?toml::get<long double>(l_tmGnssRotation.at(0)):l_stParseData.gnss_data_.transform_.rotation_.w_;
                                        l_tmGnssRotation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmGnssRotation.at(1)))?toml::get<long double>(l_tmGnssRotation.at(1)):l_stParseData.gnss_data_.transform_.rotation_.point_.x_;
                                        l_tmGnssRotation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmGnssRotation.at(2)))?toml::get<long double>(l_tmGnssRotation.at(2)):l_stParseData.gnss_data_.transform_.rotation_.point_.y_;
                                        l_tmGnssRotation.at(3) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmGnssRotation.at(3)))?toml::get<long double>(l_tmGnssRotation.at(3)):l_stParseData.gnss_data_.transform_.rotation_.point_.z_;
                                    }
                                    if(l_tmGnssTranslation.size()==3)
                                    {
                                        l_tmGnssTranslation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmGnssTranslation.at(0)))?toml::get<long double>(l_tmGnssTranslation.at(0)):l_stParseData.gnss_data_.transform_.translation_.point_.x_;
                                        l_tmGnssTranslation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmGnssTranslation.at(1)))?toml::get<long double>(l_tmGnssTranslation.at(1)):l_stParseData.gnss_data_.transform_.translation_.point_.y_;
                                        l_tmGnssTranslation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmGnssTranslation.at(2)))?toml::get<long double>(l_tmGnssTranslation.at(2)):l_stParseData.gnss_data_.transform_.translation_.point_.z_;
                                    } 
                                }
                            }
                        }
                        
                    }
                    std::cout<<" this is gnss params "<<std::endl;
                }
            }
            std::ofstream l_outStream(path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_strCalibTime = "# " + l_strCalibTime;
            l_outStream << std::endl << l_strCalibTime ;
            l_outStream.close();

        }
        void TomlReWriteTool::CompareLidar(const std::string &path)
        {
            auto l_tmData = toml::parse(path);
            auto &l_tmLidarGnss = toml::find(l_tmData, "lidar_gnss");
            auto &l_tmFrontLidar = toml::find(l_tmData, "front_lidar");
            auto &l_tmRfuEgo = toml::find(l_tmData, "rfu_ego");
            auto &l_tmGnssEgo = toml::find(l_tmData, "gnss_ego");
            std::string l_strCalibTime = "";
            std::shared_ptr<mach::tool::BaseParse> m_pGnssParser = std::make_shared<mach::tool::GnssParse>(mach::tool::ParseFileType::GNSS_PARAMS);
            std::shared_ptr<mach::tool::BaseParse> m_pLidarParser = std::make_shared<mach::tool::LidarParse>(mach::tool::ParseFileType::LIDAR_PARAMS);
            if(!m_mapParamsData_.empty())
            {
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
                        if(file.find("front_lidar")!=std::string::npos)
                        {
                            auto &l_tmFrontLidarRotation = toml::find(l_tmFrontLidar, "rotation"); 
                            auto &l_tmFrontLidarTranslation = toml::find(l_tmFrontLidar, "translation"); 
                            if(l_tmFrontLidarRotation.size()==4)
                            {
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"w : "<< toml::get<long double>(l_tmFrontLidarRotation.at(0)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"x : "<< toml::get<long double>(l_tmFrontLidarRotation.at(1)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"y : "<< toml::get<long double>(l_tmFrontLidarRotation.at(2)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"z : "<< toml::get<long double>(l_tmFrontLidarRotation.at(3)) <<std::endl;
                                l_tmFrontLidarRotation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.w_,toml::get<long double>(l_tmFrontLidarRotation.at(0)))?toml::get<long double>(l_tmFrontLidarRotation.at(0)):l_stParseData.lidar_data_.transform_.rotation_.w_;
                                l_tmFrontLidarRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmFrontLidarRotation.at(1)))?toml::get<long double>(l_tmFrontLidarRotation.at(1)):l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                l_tmFrontLidarRotation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmFrontLidarRotation.at(2)))?toml::get<long double>(l_tmFrontLidarRotation.at(2)):l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                l_tmFrontLidarRotation.at(3) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmFrontLidarRotation.at(3)))?toml::get<long double>(l_tmFrontLidarRotation.at(3)):l_stParseData.lidar_data_.transform_.rotation_.point_.z_;
                            }
                            if(l_tmFrontLidarTranslation.size()==3)
                            {
                                l_tmFrontLidarTranslation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmFrontLidarTranslation.at(0)))?toml::get<long double>(l_tmFrontLidarTranslation.at(0)):l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                l_tmFrontLidarTranslation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmFrontLidarTranslation.at(1)))?toml::get<long double>(l_tmFrontLidarTranslation.at(1)):l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                l_tmFrontLidarTranslation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmFrontLidarTranslation.at(2)))?toml::get<long double>(l_tmFrontLidarTranslation.at(2)):l_stParseData.lidar_data_.transform_.translation_.point_.z_;
                            }
                        }
                        if(file.find("lidar_ego")!=std::string::npos)
                        {
                            auto &l_tmLidarEgoRotation = toml::find(l_tmRfuEgo, "rotation"); 
                            auto &l_tmLidarEgoTranslation = toml::find(l_tmRfuEgo, "translation"); 
                            if(l_tmLidarEgoRotation.size()==4)
                            {
                                l_tmLidarEgoRotation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.w_,toml::get<long double>(l_tmLidarEgoRotation.at(0)))?toml::get<long double>(l_tmLidarEgoRotation.at(0)):l_stParseData.lidar_data_.transform_.rotation_.w_;
                                l_tmLidarEgoRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmLidarEgoRotation.at(1)))?toml::get<long double>(l_tmLidarEgoRotation.at(1)):l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                l_tmLidarEgoRotation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmLidarEgoRotation.at(2)))?toml::get<long double>(l_tmLidarEgoRotation.at(2)):l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                l_tmLidarEgoRotation.at(3) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmLidarEgoRotation.at(3)))?toml::get<long double>(l_tmLidarEgoRotation.at(3)):l_stParseData.lidar_data_.transform_.rotation_.point_.z_;
 
                            }
                            if(l_tmLidarEgoTranslation.size()==3)
                            {
                                l_tmLidarEgoTranslation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmLidarEgoTranslation.at(0)))?toml::get<long double>(l_tmLidarEgoTranslation.at(0)):l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                l_tmLidarEgoTranslation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmLidarEgoTranslation.at(1)))?toml::get<long double>(l_tmLidarEgoTranslation.at(1)):l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                l_tmLidarEgoTranslation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmLidarEgoTranslation.at(2)))?toml::get<long double>(l_tmLidarEgoTranslation.at(2)):l_stParseData.lidar_data_.transform_.translation_.point_.z_;
                            }
                        }
                        if(file.find("lidar_gnss")!=std::string::npos)
                        {
                            auto &l_tmLidarGnssRotation = toml::find(l_tmLidarGnss, "rotation"); 
                            auto &l_tmLidarGnssTranslation = toml::find(l_tmLidarGnss, "translation"); 
                            if(l_tmLidarGnssRotation.size()==4)
                            {
                                l_tmLidarGnssRotation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.w_,toml::get<long double>(l_tmLidarGnssRotation.at(0)))?toml::get<long double>(l_tmLidarGnssRotation.at(0)):l_stParseData.lidar_data_.transform_.rotation_.w_;
                                l_tmLidarGnssRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmLidarGnssRotation.at(1)))?toml::get<long double>(l_tmLidarGnssRotation.at(1)):l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                l_tmLidarGnssRotation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmLidarGnssRotation.at(2)))?toml::get<long double>(l_tmLidarGnssRotation.at(2)):l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                l_tmLidarGnssRotation.at(3) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmLidarGnssRotation.at(3)))?toml::get<long double>(l_tmLidarGnssRotation.at(3)):l_stParseData.lidar_data_.transform_.rotation_.point_.z_;
 
                            }
                            if(l_tmLidarGnssTranslation.size()==3)
                            {
                                l_tmLidarGnssTranslation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmLidarGnssTranslation.at(0)))?toml::get<long double>(l_tmLidarGnssTranslation.at(0)):l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                l_tmLidarGnssTranslation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmLidarGnssTranslation.at(1)))?toml::get<long double>(l_tmLidarGnssTranslation.at(1)):l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                l_tmLidarGnssTranslation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmLidarGnssTranslation.at(2)))?toml::get<long double>(l_tmLidarGnssTranslation.at(2)):l_stParseData.lidar_data_.transform_.translation_.point_.z_;                                
                            }
                        }
                    }
                }
                auto l_mapCalibGnss = m_mapParamsData_["gnss"];
                for(const auto & file : (l_mapCalibGnss.begin())->second)
                {
                    std::string l_strFilePath = (l_mapCalibGnss.begin())->first + "/" + file;
                    if(file.find("gnss_ego")!=std::string::npos)
                    {
                        bool l_bOK = m_pGnssParser->Open(l_strFilePath);
                        if(l_bOK)
                        {
                            auto &l_tmGnssRotation = toml::find(l_tmGnssEgo, "rotation"); 
                            auto &l_tmGnssTranslation = toml::find(l_tmGnssEgo, "translation"); 
                            auto l_stParseData = m_pGnssParser->GetParams();
                            if(l_stParseData.type_!=ParseFileType::GNSS_PARAMS)
                            {
                                break;
                            }
                            l_strCalibTime = l_stParseData.gnss_data_.calib_time_;
                            if(l_tmGnssRotation.size()==4)
                            {
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"w : "<< toml::get<long double>(l_tmGnssRotation.at(0)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"x : "<< toml::get<long double>(l_tmGnssRotation.at(1)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"y : "<< toml::get<long double>(l_tmGnssRotation.at(2)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"z : "<< toml::get<long double>(l_tmGnssRotation.at(3)) <<std::endl;
                                l_tmGnssRotation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.w_,toml::get<long double>(l_tmGnssRotation.at(0)))?toml::get<long double>(l_tmGnssRotation.at(0)):l_stParseData.gnss_data_.transform_.rotation_.w_;
                                l_tmGnssRotation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmGnssRotation.at(1)))?toml::get<long double>(l_tmGnssRotation.at(1)):l_stParseData.gnss_data_.transform_.rotation_.point_.x_;
                                l_tmGnssRotation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmGnssRotation.at(2)))?toml::get<long double>(l_tmGnssRotation.at(2)):l_stParseData.gnss_data_.transform_.rotation_.point_.y_;
                                l_tmGnssRotation.at(3) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmGnssRotation.at(3)))?toml::get<long double>(l_tmGnssRotation.at(3)):l_stParseData.gnss_data_.transform_.rotation_.point_.z_;
                            }
                            if(l_tmGnssTranslation.size()==3)
                            {
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"x : "<< toml::get<long double>(l_tmGnssTranslation.at(0)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"y : "<< toml::get<long double>(l_tmGnssTranslation.at(1)) <<std::endl;
                                //std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) <<"z : "<< toml::get<long double>(l_tmGnssTranslation.at(2)) <<std::endl;
                                l_tmGnssTranslation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmGnssTranslation.at(0)))?toml::get<long double>(l_tmGnssTranslation.at(0)):l_stParseData.gnss_data_.transform_.translation_.point_.x_;
                                l_tmGnssTranslation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmGnssTranslation.at(1)))?toml::get<long double>(l_tmGnssTranslation.at(1)):l_stParseData.gnss_data_.transform_.translation_.point_.y_;
                                l_tmGnssTranslation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmGnssTranslation.at(2)))?toml::get<long double>(l_tmGnssTranslation.at(2)):l_stParseData.gnss_data_.transform_.translation_.point_.z_;
                            }
                        }
                    }
                }
            }
            std::ofstream l_outStream(path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_strCalibTime = "# " + l_strCalibTime;
            l_outStream << std::endl << l_strCalibTime ;
            l_outStream.close();
        }
        void TomlReWriteTool::CompareTracking(const std::string &path)
        {
            auto l_tmData = toml::parse(path);
            auto &l_tmCalibrationParam = toml::find(l_tmData, "calib_params");
            std::shared_ptr<mach::tool::BaseParse> m_pGnssParser = std::make_shared<mach::tool::GnssParse>(mach::tool::ParseFileType::GNSS_PARAMS);
            std::shared_ptr<mach::tool::BaseParse> m_pLidarParser = std::make_shared<mach::tool::LidarParse>(mach::tool::ParseFileType::LIDAR_PARAMS);
            std::string l_strCalibTime = "";
            if(!m_mapParamsData_.empty())
            {
                auto l_mapCalibLidar = m_mapParamsData_["lidar"];
                for(const auto & file : (l_mapCalibLidar.begin())->second)
                {
                    std::string l_strFilePath = (l_mapCalibLidar.begin())->first + "/" + file;
                    bool l_bOK = m_pLidarParser->Open(l_strFilePath);
                    if(l_bOK)
                    {
                        auto l_stParseData = m_pLidarParser->GetParams();
                        if(l_stParseData.type_==ParseFileType::LIDAR_PARAMS)
                        {
                            if(file.find("lidar_gnss")!=std::string::npos)
                            {
                                auto &l_tmLidar2GnssParam = toml::find(l_tmCalibrationParam, "lidar2gnss");
                                auto &l_tmLidar2GnssRotation = toml::find(l_tmLidar2GnssParam, "rotation"); 
                                auto &l_tmLidar2GnssTranslation = toml::find(l_tmLidar2GnssParam, "translation"); 
                                if(l_tmLidar2GnssRotation.size()==4)
                                {
                                    l_tmLidar2GnssRotation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.w_,toml::get<long double>(l_tmLidar2GnssRotation.at(0)))?toml::get<long double>(l_tmLidar2GnssRotation.at(0)):l_stParseData.lidar_data_.transform_.rotation_.w_;
                                    l_tmLidar2GnssRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmLidar2GnssRotation.at(1)))?toml::get<long double>(l_tmLidar2GnssRotation.at(1)):l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                    l_tmLidar2GnssRotation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmLidar2GnssRotation.at(2)))?toml::get<long double>(l_tmLidar2GnssRotation.at(2)):l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                    l_tmLidar2GnssRotation.at(3) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmLidar2GnssRotation.at(3)))?toml::get<long double>(l_tmLidar2GnssRotation.at(3)):l_stParseData.lidar_data_.transform_.rotation_.point_.z_;
                                }
                                if(l_tmLidar2GnssTranslation.size()==3)
                                {
                                    l_tmLidar2GnssTranslation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmLidar2GnssTranslation.at(0)))?toml::get<long double>(l_tmLidar2GnssTranslation.at(0)):l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                    l_tmLidar2GnssTranslation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmLidar2GnssTranslation.at(1)))?toml::get<long double>(l_tmLidar2GnssTranslation.at(1)):l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                    l_tmLidar2GnssTranslation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmLidar2GnssTranslation.at(2)))?toml::get<long double>(l_tmLidar2GnssTranslation.at(2)):l_stParseData.lidar_data_.transform_.translation_.point_.z_;                                
                                }
                            }
                        }
                    }
                }
                auto l_mapCalibGnss = m_mapParamsData_["gnss"];
                for(const auto & file : (l_mapCalibGnss.begin())->second)
                {
                    std::string l_strFilePath = (l_mapCalibGnss.begin())->first + "/" + file;
                    if(file.find("gnss_ego")!=std::string::npos)
                    {
                        bool l_bOK = m_pGnssParser->Open(l_strFilePath);
                        if(l_bOK)
                        {
                            auto &l_tmGnss2EgoParam = toml::find(l_tmCalibrationParam, "gnss2ego");
                            auto &l_tmGnss2EgoRotation = toml::find(l_tmGnss2EgoParam, "rotation"); 
                            auto &l_tmGnss2EgoTranslation = toml::find(l_tmGnss2EgoParam, "translation"); 
                            auto l_stParseData = m_pGnssParser->GetParams();
                            l_strCalibTime = l_stParseData.gnss_data_.calib_time_;
                            if(l_stParseData.type_==ParseFileType::GNSS_PARAMS)
                            {
                                if(l_tmGnss2EgoRotation.size()==4)
                                {
                                    l_tmGnss2EgoRotation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.w_,toml::get<long double>(l_tmGnss2EgoRotation.at(0)))?toml::get<long double>(l_tmGnss2EgoRotation.at(0)):l_stParseData.gnss_data_.transform_.rotation_.w_;
                                    l_tmGnss2EgoRotation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmGnss2EgoRotation.at(1)))?toml::get<long double>(l_tmGnss2EgoRotation.at(1)):l_stParseData.gnss_data_.transform_.rotation_.point_.x_;
                                    l_tmGnss2EgoRotation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmGnss2EgoRotation.at(2)))?toml::get<long double>(l_tmGnss2EgoRotation.at(2)):l_stParseData.gnss_data_.transform_.rotation_.point_.y_;
                                    l_tmGnss2EgoRotation.at(3) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmGnss2EgoRotation.at(3)))?toml::get<long double>(l_tmGnss2EgoRotation.at(3)):l_stParseData.gnss_data_.transform_.rotation_.point_.z_;
                                }
                                if(l_tmGnss2EgoTranslation.size()==3)
                                {
                                    l_tmGnss2EgoTranslation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmGnss2EgoTranslation.at(0)))?toml::get<long double>(l_tmGnss2EgoTranslation.at(0)):l_stParseData.gnss_data_.transform_.translation_.point_.x_;
                                    l_tmGnss2EgoTranslation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmGnss2EgoTranslation.at(1)))?toml::get<long double>(l_tmGnss2EgoTranslation.at(1)):l_stParseData.gnss_data_.transform_.translation_.point_.y_;
                                    l_tmGnss2EgoTranslation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmGnss2EgoTranslation.at(2)))?toml::get<long double>(l_tmGnss2EgoTranslation.at(2)):l_stParseData.gnss_data_.transform_.translation_.point_.z_;                                
                                }
                            }
                        }
                    }
                }
            }
            std::ofstream l_outStream(path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_strCalibTime = "# " + l_strCalibTime;
            l_outStream << std::endl << l_strCalibTime ;
            l_outStream.close();
        }
        void TomlReWriteTool::CompareObstaclePostfusion(const std::string &path)
        {
            auto l_tmData = toml::parse(path);
            std::string l_strCalibTime = "";
            auto &l_tmCalibrationParam = toml::find(l_tmData, "calibration_parameters");
            std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
            if(!m_mapParamsData_.empty())
            {
                auto l_mapCalibCam = m_mapParamsData_["camera"];
                auto l_tbCalibParam = l_tmCalibrationParam.as_table();
                std::string l_strCameraName = "";
                if(l_tbCalibParam.size()>0)
                {
                    l_strCameraName = l_tbCalibParam.begin()->first;
                    for(const auto & file : (l_mapCalibCam.begin())->second)
                    {
                        if(file.find(l_strCameraName)!=std::string::npos)
                        {
                            std::string l_strFilePath = (l_mapCalibCam.begin())->first + "/" + file;
                            bool l_bOK = m_pCameraParser->Open(l_strFilePath);
                            if(l_bOK)
                            {
                                auto l_stParseData = m_pCameraParser->GetParams();
                                auto &l_tmCamParams = toml::find(l_tmCalibrationParam, l_strCameraName);
                                if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::INTRINSIC)
                                {
                                    l_tmCamParams["image_width"] = CompareData(toml::find<uint>(l_tmCamParams,"image_width"),l_stParseData.camera_data_.resolution_.width_)?toml::find<uint>(l_tmCamParams,"image_width") :  l_stParseData.camera_data_.resolution_.width_;
                                    l_tmCamParams["image_height"] = CompareData(toml::find<uint>(l_tmCamParams,"image_height"),l_stParseData.camera_data_.resolution_.height_)?toml::find<uint>(l_tmCamParams,"image_height") : l_stParseData.camera_data_.resolution_.height_;
                                    l_tmCamParams["distortion_model"] = CompareData(toml::find<std::string>(l_tmCamParams,"distortion_model"),l_stParseData.camera_data_.distortion_model_)?toml::find<std::string>(l_tmCamParams,"distortion_model") : l_stParseData.camera_data_.distortion_model_;
                                    auto &l_tmCamMatrix = toml::find(l_tmCamParams, "camera_matrix");   
                                    if(toml::find<uint>(l_tmCamMatrix, "rows")>0&&l_stParseData.camera_data_.K.size()==toml::find<uint>(l_tmCamMatrix, "rows")&&l_stParseData.camera_data_.K[0].size()==toml::find<uint>(l_tmCamMatrix, "cols"))
                                    {
                                        auto &l_tmCamMatrixData = toml::find(l_tmCamMatrix, "data");
                                        int l_iDataIndex = 0;
                                        for(const auto & matrixs : l_stParseData.camera_data_.K)
                                        {
                                            for(const auto & matrix : matrixs)
                                            {
                                                // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << matrix << " " << std::endl;
                                                l_tmCamMatrixData.at(l_iDataIndex) = CompareDoubleData(matrix,toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)):matrix;
                                                l_iDataIndex++;
                                            }
                                        }
                                    }
                                    auto &l_tmCamDistortion = toml::find(l_tmCamParams, "distortion_coefficients");
                                    if(toml::find<uint>(l_tmCamDistortion, "cols")>0 && toml::find<uint>(l_tmCamDistortion, "cols")==l_stParseData.camera_data_.D.size())
                                    {
                                        auto &l_tmCamDistortionData = toml::find(l_tmCamDistortion, "data");
                                        int l_iDataIndex = 0;
                                        for(const auto &data : l_stParseData.camera_data_.D)
                                        {
                                            // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << data << " " << std::endl;
                                            l_tmCamDistortionData.at(l_iDataIndex) = CompareDoubleData(data,toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)):data;
                                            l_iDataIndex++;
                                        }
                                    }     
                                }
                                if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::EXTRINSIC)
                                {
                                    l_strCalibTime = l_stParseData.camera_data_.calib_time_;
                                    auto &l_tmRfuCamMatrix = toml::find(l_tmCamParams, "rfu_cam_matrix"); 
                                    auto &l_tmCameraRotation = toml::find(l_tmRfuCamMatrix, "rotation"); 
                                    auto &l_tmCameraTranslation = toml::find(l_tmRfuCamMatrix, "translation"); 
                                    if(l_tmCameraRotation.size()==4)
                                    {
                                        l_tmCameraRotation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.w_,toml::get<long double>(l_tmCameraRotation.at(0)))?toml::get<long double>(l_tmCameraRotation.at(0)):l_stParseData.camera_data_.transform_.rotation_.w_;
                                        l_tmCameraRotation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmCameraRotation.at(1)))?toml::get<long double>(l_tmCameraRotation.at(1)):l_stParseData.camera_data_.transform_.rotation_.point_.x_;
                                        l_tmCameraRotation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmCameraRotation.at(2)))?toml::get<long double>(l_tmCameraRotation.at(2)):l_stParseData.camera_data_.transform_.rotation_.point_.y_;
                                        l_tmCameraRotation.at(3) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmCameraRotation.at(3)))?toml::get<long double>(l_tmCameraRotation.at(3)):l_stParseData.camera_data_.transform_.rotation_.point_.z_;
                                    }
                                    if(l_tmCameraTranslation.size()==3)
                                    {
                                        l_tmCameraTranslation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmCameraTranslation.at(0)))?toml::get<long double>(l_tmCameraTranslation.at(0)):l_stParseData.camera_data_.transform_.translation_.point_.x_;
                                        l_tmCameraTranslation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmCameraTranslation.at(1)))?toml::get<long double>(l_tmCameraTranslation.at(1)):l_stParseData.camera_data_.transform_.translation_.point_.y_;
                                        l_tmCameraTranslation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmCameraTranslation.at(2)))?toml::get<long double>(l_tmCameraTranslation.at(2)):l_stParseData.camera_data_.transform_.translation_.point_.z_;
                                    }
                                }
                            }
                        }
                    }
                
                }
            }
            std::ofstream l_outStream(path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_strCalibTime = "# " + l_strCalibTime;
            l_outStream << std::endl << l_strCalibTime ;
            l_outStream.close();
        }
        void TomlReWriteTool::CompareSaticObstacle(const std::string &path)
        {
            auto l_tmData = toml::parse(path);
            std::string l_strCalibTime = "";
            auto &l_tmPerceptionData = toml::find(l_tmData, "perception_parameters");
            auto &l_tmCalibrationParam = toml::find(l_tmData, "calibration_parameters");
            const std::string & l_strCameraName = toml::find<std::string>(l_tmPerceptionData, "camera_name");
            std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
            std::shared_ptr<mach::tool::BaseParse> m_pGnssParser = std::make_shared<mach::tool::GnssParse>(mach::tool::ParseFileType::GNSS_PARAMS);
            std::shared_ptr<mach::tool::BaseParse> m_pLidarParser = std::make_shared<mach::tool::LidarParse>(mach::tool::ParseFileType::LIDAR_PARAMS);
            if(!m_mapParamsData_.empty())
            {
                auto l_mapCalibCam = m_mapParamsData_["camera"];
                for(const auto & file : (l_mapCalibCam.begin())->second)
                {
                    if(file.find(l_strCameraName)!=std::string::npos)
                    {
                        std::string l_strFilePath = (l_mapCalibCam.begin())->first + "/" + file;
                        bool l_bOK = m_pCameraParser->Open(l_strFilePath);
                        if(l_bOK)
                        {
                            auto l_stParseData = m_pCameraParser->GetParams();
                            auto &l_tmCamParams = toml::find(l_tmCalibrationParam, l_strCameraName);
                            if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::INTRINSIC)
                            {
                                l_tmCamParams["image_width"] = CompareData(toml::find<uint>(l_tmCamParams,"image_width"),l_stParseData.camera_data_.resolution_.width_)?toml::find<uint>(l_tmCamParams,"image_width") :  l_stParseData.camera_data_.resolution_.width_;
                                l_tmCamParams["image_height"] = CompareData(toml::find<uint>(l_tmCamParams,"image_height"),l_stParseData.camera_data_.resolution_.height_)?toml::find<uint>(l_tmCamParams,"image_height") : l_stParseData.camera_data_.resolution_.height_;
                                l_tmCamParams["distortion_model"] = CompareData(toml::find<std::string>(l_tmCamParams,"distortion_model"),l_stParseData.camera_data_.distortion_model_)?toml::find<std::string>(l_tmCamParams,"distortion_model") : l_stParseData.camera_data_.distortion_model_;
                                auto &l_tmCamMatrix = toml::find(l_tmCamParams, "camera_matrix");   
                                if(toml::find<uint>(l_tmCamMatrix, "rows")>0&&l_stParseData.camera_data_.K.size()==toml::find<uint>(l_tmCamMatrix, "rows")&&l_stParseData.camera_data_.K[0].size()==toml::find<uint>(l_tmCamMatrix, "cols"))
                                {
                                    auto &l_tmCamMatrixData = toml::find(l_tmCamMatrix, "data");
                                    int l_iDataIndex = 0;
                                    for(const auto & matrixs : l_stParseData.camera_data_.K)
                                    {
                                        for(const auto & matrix : matrixs)
                                        {
                                            // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << matrix << " " << std::endl;
                                            l_tmCamMatrixData.at(l_iDataIndex) = CompareDoubleData(matrix,toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamMatrixData.at(l_iDataIndex)):matrix;
                                            l_iDataIndex++;
                                        }
                                    }
                                }
                                auto &l_tmCamDistortion = toml::find(l_tmCamParams, "distortion_coefficients");
                                if(toml::find<uint>(l_tmCamDistortion, "cols")>0 && toml::find<uint>(l_tmCamDistortion, "cols")==l_stParseData.camera_data_.D.size())
                                {
                                    auto &l_tmCamDistortionData = toml::find(l_tmCamDistortion, "data");
                                    int l_iDataIndex = 0;
                                    for(const auto &data : l_stParseData.camera_data_.D)
                                    {
                                        // std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << data << " " << std::endl;
                                        l_tmCamDistortionData.at(l_iDataIndex) = CompareDoubleData(data,toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)))?toml::get<long double>(l_tmCamDistortionData.at(l_iDataIndex)):data;
                                        l_iDataIndex++;
                                    }
                                }     
                            }
                            if(l_stParseData.type_ == ParseFileType::CAMERA_PARAMS && l_stParseData.camera_data_.type == CameraParamType::EXTRINSIC)
                            {
                                auto &l_tmRfuCamMatrix = toml::find(l_tmCamParams, "rfu_cam_matrix"); 
                                auto &l_tmCameraRotation = toml::find(l_tmRfuCamMatrix, "rotation"); 
                                auto &l_tmCameraTranslation = toml::find(l_tmRfuCamMatrix, "translation"); 
                                if(l_tmCameraRotation.size()==4)
                                {
                                    l_tmCameraRotation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.w_,toml::get<long double>(l_tmCameraRotation.at(0)))?toml::get<long double>(l_tmCameraRotation.at(0)):l_stParseData.camera_data_.transform_.rotation_.w_;
                                    l_tmCameraRotation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmCameraRotation.at(1)))?toml::get<long double>(l_tmCameraRotation.at(1)):l_stParseData.camera_data_.transform_.rotation_.point_.x_;
                                    l_tmCameraRotation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmCameraRotation.at(2)))?toml::get<long double>(l_tmCameraRotation.at(2)):l_stParseData.camera_data_.transform_.rotation_.point_.y_;
                                    l_tmCameraRotation.at(3) = CompareDoubleData(l_stParseData.camera_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmCameraRotation.at(3)))?toml::get<long double>(l_tmCameraRotation.at(3)):l_stParseData.camera_data_.transform_.rotation_.point_.z_;
                                }
                                if(l_tmCameraTranslation.size()==3)
                                {
                                    l_tmCameraTranslation.at(0) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmCameraTranslation.at(0)))?toml::get<long double>(l_tmCameraTranslation.at(0)):l_stParseData.camera_data_.transform_.translation_.point_.x_;
                                    l_tmCameraTranslation.at(1) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmCameraTranslation.at(1)))?toml::get<long double>(l_tmCameraTranslation.at(1)):l_stParseData.camera_data_.transform_.translation_.point_.y_;
                                    l_tmCameraTranslation.at(2) = CompareDoubleData(l_stParseData.camera_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmCameraTranslation.at(2)))?toml::get<long double>(l_tmCameraTranslation.at(2)):l_stParseData.camera_data_.transform_.translation_.point_.z_;
                                }
                            }
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
                        if(l_bOK)
                        {
                            auto l_stParseData = m_pGnssParser->GetParams();
                            l_strCalibTime = l_stParseData.gnss_data_.calib_time_;
                            if(l_stParseData.type_ == ParseFileType::GNSS_PARAMS)
                            {
                                auto &l_tmGnssParams = toml::find(l_tmCalibrationParam, "gnss_ego");
                                auto &l_tmGnssRotation = toml::find(l_tmGnssParams, "rotation"); 
                                auto &l_tmGnssTranslation = toml::find(l_tmGnssParams, "translation");
                                if(l_tmGnssRotation.size()==4)
                                {
                                    l_tmGnssRotation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.w_,toml::get<long double>(l_tmGnssRotation.at(0)))?toml::get<long double>(l_tmGnssRotation.at(0)):l_stParseData.gnss_data_.transform_.rotation_.w_;
                                    l_tmGnssRotation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmGnssRotation.at(1)))?toml::get<long double>(l_tmGnssRotation.at(1)):l_stParseData.gnss_data_.transform_.rotation_.point_.x_;
                                    l_tmGnssRotation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmGnssRotation.at(2)))?toml::get<long double>(l_tmGnssRotation.at(2)):l_stParseData.gnss_data_.transform_.rotation_.point_.y_;
                                    l_tmGnssRotation.at(3) = CompareDoubleData(l_stParseData.gnss_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmGnssRotation.at(3)))?toml::get<long double>(l_tmGnssRotation.at(3)):l_stParseData.gnss_data_.transform_.rotation_.point_.z_;
                                }
                                if(l_tmGnssTranslation.size()==3)
                                {
                                    l_tmGnssTranslation.at(0) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmGnssTranslation.at(0)))?toml::get<long double>(l_tmGnssTranslation.at(0)):l_stParseData.gnss_data_.transform_.translation_.point_.x_;
                                    l_tmGnssTranslation.at(1) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmGnssTranslation.at(1)))?toml::get<long double>(l_tmGnssTranslation.at(1)):l_stParseData.gnss_data_.transform_.translation_.point_.y_;
                                    l_tmGnssTranslation.at(2) = CompareDoubleData(l_stParseData.gnss_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmGnssTranslation.at(2)))?toml::get<long double>(l_tmGnssTranslation.at(2)):l_stParseData.gnss_data_.transform_.translation_.point_.z_;
                                } 
                            }
                        }
                    }
                }

                auto l_mapCalibLidar = m_mapParamsData_["lidar"];
                for(const auto & file : (l_mapCalibLidar.begin())->second)
                {
                    if(file.find("lidar_ego")!=std::string::npos)
                    {
                        std::string l_strFilePath = (l_mapCalibLidar.begin())->first + "/" + file;
                        bool l_bOK = m_pLidarParser->Open(l_strFilePath);
                        if(l_bOK)
                        {
                            auto l_stParseData = m_pLidarParser->GetParams();
                            if(l_stParseData.type_ == ParseFileType::LIDAR_PARAMS)
                            {
                                auto &l_tmLidarParams = toml::find(l_tmCalibrationParam, "rfu_ego");
                                auto &l_tmLidarRotation = toml::find(l_tmLidarParams, "rotation"); 
                                auto &l_tmLidarTranslation = toml::find(l_tmLidarParams, "translation");
                                if(l_tmLidarRotation.size()==4)
                                {
                                    l_tmLidarRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.w_,toml::get<long double>(l_tmLidarRotation.at(0)))?toml::get<long double>(l_tmLidarRotation.at(0)):l_stParseData.lidar_data_.transform_.rotation_.w_;
                                    l_tmLidarRotation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.x_,toml::get<long double>(l_tmLidarRotation.at(1)))?toml::get<long double>(l_tmLidarRotation.at(1)):l_stParseData.lidar_data_.transform_.rotation_.point_.x_;
                                    l_tmLidarRotation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.y_,toml::get<long double>(l_tmLidarRotation.at(2)))?toml::get<long double>(l_tmLidarRotation.at(2)):l_stParseData.lidar_data_.transform_.rotation_.point_.y_;
                                    l_tmLidarRotation.at(3) = CompareDoubleData(l_stParseData.lidar_data_.transform_.rotation_.point_.z_,toml::get<long double>(l_tmLidarRotation.at(3)))?toml::get<long double>(l_tmLidarRotation.at(3)):l_stParseData.lidar_data_.transform_.rotation_.point_.z_;
                                }
                                if(l_tmLidarTranslation.size()==3)
                                {
                                    l_tmLidarTranslation.at(0) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.x_,toml::get<long double>(l_tmLidarTranslation.at(0)))?toml::get<long double>(l_tmLidarTranslation.at(0)):l_stParseData.lidar_data_.transform_.translation_.point_.x_;
                                    l_tmLidarTranslation.at(1) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.y_,toml::get<long double>(l_tmLidarTranslation.at(1)))?toml::get<long double>(l_tmLidarTranslation.at(1)):l_stParseData.lidar_data_.transform_.translation_.point_.y_;
                                    l_tmLidarTranslation.at(2) = CompareDoubleData(l_stParseData.lidar_data_.transform_.translation_.point_.z_,toml::get<long double>(l_tmLidarTranslation.at(2)))?toml::get<long double>(l_tmLidarTranslation.at(2)):l_stParseData.lidar_data_.transform_.translation_.point_.z_;
                                } 
                            }
                        }
                    }
                }
            }
            std::ofstream l_outStream(path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_strCalibTime = "# " + l_strCalibTime;
            l_outStream << std::endl << l_strCalibTime ;
            l_outStream.close();
        }
    } // namespace tool

} // namespace mach

