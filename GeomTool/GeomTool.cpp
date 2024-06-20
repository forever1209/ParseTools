#include "GeomTool.h"
#include "PubParseDef.h"
#include "ParseCamera.h"
#include <unordered_map>
#include <opencv2/calib3d.hpp>
#include "xtensor-blas/xlinalg.hpp"
#include <xtensor-python/pyarray.hpp>
#include "LaneLine/LaneLineTool.h"
#include "Obstacle/ObstacleTool.h"
namespace mach
{
    namespace tool
    {
        GeomTool::GeomTool()
        {
            m_base_path_ = "";
            DetectionConfig byd_det_fb_config {
                {512, 1408},
                32,
                {-25.6, 25.6 ,0.8},
                {-80.0, 80.0 ,0.8},
                {-3, 5 ,8},
                {2.0, 82.0 , 0.5},
                {
                    {{512, 1408}, {1080, 1920}, {0.1, 0.1}},
                    {{512, 1408}, {1080, 1920}, {0.0, 0.0}},
                    {{512, 1408}, {1080, 1920}, {0.1, 0.1}}
                }
            };
            DetectionConfig byd_det_lr_config {
                {256, 704},
                16,
                {-25.6, 25.6 ,0.8},
                {-80.0, 80.0 ,0.8},
                {-3, 5, 8},
                {0.0, 33.6-0.1,0.3},
                {
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}}
                }
            };

            DetectionConfig byd_lane_config {
                {512, 1408},
                16,
                {0, 120,0.6*2},
                {-15, 15,0.15*2},
                {-10.0, 10.0,2.0},
                {4.8, 120,0.6*2},
                {
                    {{512, 1408}, {1080, 1920}, {0.0, 0.0}},
                    {{512, 1408}, {1080, 1920}, {0.1, 0.1}}
                }
            };

            // 定义HSConfig
            DetectionConfig hs_det_fb_config {
                {512, 1408},
                32,
                {-25.6, 25.6,0.8},
                {-80.0, 80.0,0.8},
                {-3, 5,8},
                {2.0, 82.0,0.5},
                {
                    {{512, 1408}, {1080, 1920}, {0.0, 0.0}},
                    {{512, 1408}, {1080, 1920}, {0.323, 0.323}},
                    {{512, 1408}, {1080, 1920}, {0.1, 0.1}}
                }
            };

            DetectionConfig hs_det_lr_config {
                {256, 704},
                16,
                {-25.6, 25.6,0.8},
                {-80.0, 80.0,0.8},
                {-3, 5,8},
                {0.0, 33.6-0.1,0.3},
                {
                    {{256, 704}, {1536, 1920}, {0.278, 0.278}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}},
                    {{256, 704}, {1536, 1920}, {0.35, 0.45}}
                }
            };

            DetectionConfig hs_lane_config {
                {512, 1408},
                16,
                {0, 120.0,0.6*2},
                {-15, 15,0.15*2},
                {-10.0, 10.0,2.0},
                {4.8, 120.0,0.6*2},
                {
                    {{512, 1408}, {1080, 1920}, {0.323, 0.323}},
                    {{512, 1408}, {1080, 1920}, {0, 0}}
                }
            };
            byd_det_fb_config_ = byd_det_fb_config;
            byd_det_lr_config_ = byd_det_lr_config;
            byd_lane_config_   = byd_lane_config ;
            hs_det_fb_config_  = hs_det_fb_config;
            hs_det_lr_config_  = hs_det_lr_config;
            hs_lane_config_    = hs_lane_config ;
        }
        GeomTool::~GeomTool()
        {

        }
        bool GeomTool::InitTool(const GeomConfigType &type, const std::string &base_path)
        {
            m_enConfigType_ = type;
            m_base_path_ = base_path;
            std::string l_strOutPath = m_base_path_ + "/output/";
            bool bIsExist = (0 == access(l_strOutPath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strOutPath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            std::string l_strOutlanePath = m_base_path_ + "/output/lane_bin/";
            std::string l_strOutObstaclePath = m_base_path_ + "/output/obstacle_bin/";
            bIsExist &= (0 == access(l_strOutlanePath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strOutlanePath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            bIsExist &= (0 == access(l_strOutObstaclePath.c_str(), F_OK));
            if(!bIsExist)
            {
                bIsExist = mkdir(l_strOutObstaclePath.c_str(), 0777) == 0 ? true:false;
                if(!bIsExist)
                {
                    return false;
                }
            }
            if(m_enConfigType_ == GeomConfigType::DEFAULT_CONFIG)
            {
                return false;
            }
            return true;
        }
        void GeomTool::ConflateData(const std::string &pkl_path,const std::string &vehicle_id,const std::string &config_path)
        {
            std::unordered_map<std::string,std::string> l_mapVehicleId = {
            {"car201" , "car_byd_han_00001"} ,
            {"car202" , "car_byd_han_00002"} ,
            {"car602" , "car_00602"}
            };
            std::string l_strConfigPath = "";
            if(l_mapVehicleId.count(vehicle_id)>0)
            {
                l_strConfigPath = l_mapVehicleId[vehicle_id];
            }
            else
            {
                return;
            }
            std::string l_strCalibPath = m_base_path_ + "/ChildrenPath/calibresult/" + l_strConfigPath + "/camera_params/";
            bool l_bExist = CheckCalibExist(l_strCalibPath);
            if(l_bExist)
            {
                InitCameraInfo(l_strCalibPath);
                InitImageMapping(pkl_path);
                if(m_enConfigType_ == GeomConfigType::GEOM_BYD_CONFIG)
                {
                    m_stVehicleConfig_.det_fb_config = byd_det_fb_config_;
                    m_stVehicleConfig_.det_lr_config = byd_det_lr_config_;
                    m_stVehicleConfig_.lane_config = byd_lane_config_;
                }
                if(m_enConfigType_ == GeomConfigType::GEOM_HS_CONFIG)
                {
                    m_stVehicleConfig_.det_fb_config = hs_det_fb_config_;
                    m_stVehicleConfig_.det_lr_config = hs_det_lr_config_;
                    m_stVehicleConfig_.lane_config = hs_lane_config_;
                }
                std::vector<xt::xarray<double>> ida_mats;
                for(const auto &item :m_stVehicleConfig_.det_fb_config.ida_cfg)
                {
                    double resize = std::max(static_cast<double>(item.final_HW.first) / item.HW.first,
                             static_cast<double>(item.final_HW.second) / item.HW.second);
                    int l_iNewW = (int) item.HW.second * resize ;
                    int l_iNewH = (int) item.HW.first * resize ;
                    int l_iCropH = (int) ((1-Mean(item.bot_pct_lim)) * l_iNewH - item.final_HW.first);
                    int l_iCropW = (int) (std::max(0, (l_iNewW - item.final_HW.second ))/ 2);
                    ida_mats.emplace_back(CreateIdaMat(resize,l_iCropW,l_iCropH));
                }
                xt::xarray<double> ida_mats_fb_stacked = xt::zeros<double>({ida_mats.size(), ida_mats[0].shape()[0], ida_mats[0].shape()[1]});

                // 将 ida_mats 中的每个元素复制到 ida_mats_fb_stacked 中对应的位置
                for (std::size_t i = 0; i < ida_mats.size(); ++i)
                {
                    xt::view(ida_mats_fb_stacked, i, xt::all(), xt::all()) = ida_mats[i];
                }
                // std::cout << ida_mats_fb_stacked << std::endl;
                auto calibMatrix = PrepareCalib();
                auto intrin_fb = xt::view(calibMatrix.intrinsic_mats, xt::range(4, xt::placeholders::_));
                auto extrin_fb = xt::view(calibMatrix.extrinsic_mats, xt::range(4, xt::placeholders::_));
                //fb geom
                DumpGeom(ida_mats_fb_stacked,m_stVehicleConfig_.det_fb_config,intrin_fb, extrin_fb ,m_base_path_ + "/output/geom_fb.npy");

                //lr
                ida_mats.clear();
                for(const auto &item :m_stVehicleConfig_.det_lr_config.ida_cfg)
                {
                    double resize = std::max(static_cast<double>(item.final_HW.first) / item.HW.first,
                             static_cast<double>(item.final_HW.second) / item.HW.second);
                    int l_iNewW = (int) item.HW.second * resize ;
                    int l_iNewH = (int) item.HW.first * resize ;
                    int l_iCropH = (int) ((1-Mean(item.bot_pct_lim)) * l_iNewH - item.final_HW.first);
                    int l_iCropW = (int) (std::max(0, (l_iNewW - item.final_HW.second ))/ 2);
                    ida_mats.emplace_back(CreateIdaMat(resize,l_iCropW,l_iCropH));
                }
                xt::xarray<double> ida_mats_lr_stacked = xt::zeros<double>({ida_mats.size(), ida_mats[0].shape()[0], ida_mats[0].shape()[1]});

                // 将 ida_mats 中的每个元素复制到 ida_mats_lr_stacked 中对应的位置
                for (std::size_t i = 0; i < ida_mats.size(); ++i)
                {
                    xt::view(ida_mats_lr_stacked, i, xt::all(), xt::all()) = ida_mats[i];
                }
                auto intrin_lr = xt::view(calibMatrix.intrinsic_mats, xt::range(xt::placeholders::_, 4));
                auto extrin_lr = xt::view(calibMatrix.extrinsic_mats, xt::range(xt::placeholders::_, 4));
                DumpGeom(ida_mats_lr_stacked,m_stVehicleConfig_.det_lr_config,intrin_lr, extrin_lr ,m_base_path_ + "/output/geom_lr.npy");
                //lane
                ida_mats.clear();
                for(const auto &item :m_stVehicleConfig_.lane_config.ida_cfg)
                {
                    double resize = std::max(static_cast<double>(item.final_HW.first) / item.HW.first,
                             static_cast<double>(item.final_HW.second) / item.HW.second);
                    int l_iNewW = (int) item.HW.second * resize ;
                    int l_iNewH = (int) item.HW.first * resize ;
                    int l_iCropH = (int) ((1-Mean(item.bot_pct_lim)) * l_iNewH - item.final_HW.first);
                    int l_iCropW = (int) (std::max(0, (l_iNewW - item.final_HW.second ))/ 2);
                    ida_mats.emplace_back(CreateIdaMat(resize,l_iCropW,l_iCropH));
                }
                xt::xarray<double> ida_mats_lane_stacked = xt::zeros<double>({ida_mats.size(), ida_mats[0].shape()[0], ida_mats[0].shape()[1]});

                // 将 ida_mats 中的每个元素复制到 ida_mats_lane_stacked 中对应的位置
                for (std::size_t i = 0; i < ida_mats.size(); ++i)
                {
                    xt::view(ida_mats_lane_stacked, i, xt::all(), xt::all()) = ida_mats[i];
                }
                auto intrin_lane = xt::view(calibMatrix.intrinsic_mats, xt::keep(5, 4), xt::all());
                auto extrin_lane_temp = xt::view(calibMatrix.extrinsic_mats, xt::keep(5, 4), xt::all());
                xt::xarray<double> ego2rfu = {{0.0f, -1.0f, 0.0f, 0.0f},
                                 {1.0f, 0.0f, 0.0f, 0.0f},
                                 {0.0f, 0.0f, 1.0f, 0.33f},
                                 {0.0f, 0.0f, 0.0f, 1.0f}};
                xt::xarray<double> extrin_lane = xt::empty<double>({int(extrin_lane_temp.shape()[0]), int(extrin_lane_temp.shape()[1]), int(extrin_lane_temp.shape()[2])});
                for(int i=0 ; i<extrin_lane_temp.shape()[0] ; ++i)
                {
                    auto extrin_mat = xt::view(extrin_lane_temp, i, xt::all(), xt::all());
                    auto result = xt::linalg::dot(extrin_mat,ego2rfu);
                    xt::view(extrin_lane, i) = result;
                }
                DumpGeom(ida_mats_lane_stacked,m_stVehicleConfig_.lane_config,intrin_lane, extrin_lane ,m_base_path_ + "/output/geom_lane.npy");
                // std::cout<<"extrin_lane shape []"<<extrin_lane.shape()[0]<<" , "<<extrin_lane.shape()[1]<<" , "<<extrin_lane.shape()[2]<<std::endl;
                // std::cout<<"intrin_lane \n"<<intrin_lane<<std::endl;
                // std::cout<<"extrin_lane \n"<<extrin_lane<<std::endl;
                {
                    std::shared_ptr<mach::tool::LaneLineTool> m_pLaneLine = std::make_shared<mach::tool::LaneLineTool>(m_base_path_);
                    m_pLaneLine->GenVoxelMapping(m_base_path_ + "/output/geom_lane.npy",m_base_path_ + "/input/laneline/voxel_num.npy");
                }
                {
                    std::shared_ptr<mach::tool::ObstacleTool> m_pObstacle = std::make_shared<mach::tool::ObstacleTool>(m_base_path_);
                    m_pObstacle->GenVoxelMapping(m_base_path_ + "/output/geom_fb.npy",m_base_path_ + "/output/geom_lr.npy",m_base_path_ + "/input/obstacle/voxel_num.npy");
                    m_pObstacle->GenBin(config_path);
                }
            }
            std::cout<<"CheckCalibExist "<< l_strCalibPath <<" is : "<<l_bExist<<std::endl;
        }
        bool GeomTool::InitCameraInfo(const std::string &config_path)
        {
            if(m_vecCamera.empty())
            {
                return false;
            }
            std::shared_ptr<mach::tool::BaseParse> m_pCameraParser = std::make_shared<mach::tool::CameraParse>(mach::tool::ParseFileType::CAMERA_PARAMS);
            for(const auto &camera:m_vecCamera)
            {
                std::string l_stringExtrConfigPath = config_path + "/" + camera + "_extrinsic.json";
                std::string l_stringIntrConfigPath = config_path + "/" + camera + "_intrinsic.json";
                if(!CheckCalibExist(l_stringExtrConfigPath) || !CheckCalibExist(l_stringIntrConfigPath))
                {
                    return false;
                }
                bool l_bOk = m_pCameraParser->Open(l_stringIntrConfigPath);
                //内，外参
                std::pair<ParamsData,ParamsData> l_paCameraData;
                if(l_bOk)
                {
                   l_paCameraData.first = m_pCameraParser->GetParams();
                }
                else
                {
                    return false;
                }
                l_bOk = m_pCameraParser->Open(l_stringExtrConfigPath);
                if(l_bOk)
                {
                   l_paCameraData.second = m_pCameraParser->GetParams();
                //    std::cout<<"camera "<<camera<<"extr info \n"<<l_paCameraData.second<<std::endl;
                }
                else
                {
                    return false;
                }
                m_mapCameraData_[camera] = l_paCameraData;
            }

        }
        bool GeomTool::InitImageMapping(const std::string &pkl_path)
        {
            for(const auto &camera : m_vecCamera)
            {
                if(camera.find("200") != std::string::npos)
                {
                    cv::Size dim(1920, 1536);
                    cv::Mat K_mat(3, 3, CV_64F);
                    cv::Mat_<double> D_mat(1,4);
                    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
                    cv::Mat new_K;
                    if(m_mapCameraData_.count(camera)<=0)
                    {
                        continue;
                    }
                    auto l_vecK = m_mapCameraData_[camera].first.camera_data_.K;
                    auto l_vecD = m_mapCameraData_[camera].first.camera_data_.D;
                    for (int i = 0; i < l_vecK.size(); ++i) 
                    {
                        for (int j = 0; j < l_vecK[i].size(); ++j) 
                        {
                            K_mat.at<double>(i, j) = static_cast<double>(l_vecK[i][j]); // 将 long double 转换为 double
                            // std::cout << K_mat.at<double>(i, j) << " ";
                        }
                        // std::cout<< std::endl;
                    }
                    for (int i = 0; i < l_vecD.size(); ++i) 
                    {
                        D_mat(0, i) = static_cast<double>(l_vecD[i]); // 将 long double 转换为 double 并填充到矩阵中
                        // std::cout << D_mat(0, i) ;
                    }
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.5,dim,0.5);
                    // std::cout<<"camera "<<camera<<" new key \n"<<new_K<<std::endl;
                    m_mapIntrinMat_[camera] = new_K;
                }
                if(camera.find("120") != std::string::npos)
                {
                    cv::Size dim(3840, 2160);
                    cv::Mat K_mat(3, 3, CV_64F);
                    cv::Mat_<double> D_mat(1,4);
                    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
                    cv::Mat new_K;
                    if(m_mapCameraData_.count(camera)<=0)
                    {
                        continue;
                    }
                    auto l_vecK = m_mapCameraData_[camera].first.camera_data_.K;
                    auto l_vecD = m_mapCameraData_[camera].first.camera_data_.D;
                    for (int i = 0; i < l_vecK.size(); ++i) 
                    {
                        for (int j = 0; j < l_vecK[i].size(); ++j) 
                        {
                            K_mat.at<double>(i, j) = static_cast<double>(l_vecK[i][j]); // 将 long double 转换为 double
                            // std::cout << K_mat.at<double>(i, j) << " ";
                        }
                        // std::cout<< std::endl;
                    }
                    for (int i = 0; i < l_vecD.size(); ++i) 
                    {
                        D_mat(0, i) = static_cast<double>(l_vecD[i]); // 将 long double 转换为 double 并填充到矩阵中
                        // std::cout << D_mat(0, i) ;
                    }
                    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_mat,D_mat,dim,R,new_K,0.325,dim,1.0);
                    new_K.row(0) /= 2;
                    new_K.row(1) /= 2;
                    m_mapIntrinMat_[camera] = new_K;

                    // auto l_Matrix = parse.GetPklData(pkl_path , "fov30");
                    PklParse parse(m_base_path_);
                    auto l_array = parse.GetPklDataXt(pkl_path , "fov30");
                    auto block_view = xt::view(l_array, xt::range(0, 2), xt::range(0, 3));
                    block_view /= 2;
                    cv::Mat new_K_fov30(3, 3, CV_64F);
                    // FillCvMatFromEigen(l_Matrix,new_K_fov30);
                    FillCvMatFromXtensor(l_array,new_K_fov30);
                    m_mapIntrinMat_[camera+"_fov30"] = new_K_fov30;
                }
                if(camera.find("100") != std::string::npos)
                {
                    PklParse parse(m_base_path_);
                    auto l_array = parse.GetPklDataXt(pkl_path , "fov70");
                    auto block_view = xt::view(l_array, xt::range(0, 2), xt::range(0, 3));
                    block_view /= 2;
                    cv::Mat new_K(3, 3, CV_64F);
                    FillCvMatFromXtensor(l_array,new_K);
                    m_mapIntrinMat_[camera] = new_K;
                }
            }
        }
        bool GeomTool::CheckCalibExist(const std::string &config_path)
        {
            bool bIsExist = (0 == access(config_path.c_str(), F_OK));
            return bIsExist;
        }
        double GeomTool::Mean(const std::pair<double, double> &datas)
        {
            return (datas.first+datas.second)/2;
        }
        xt::xarray<double> GeomTool::CreateIdaMat(const double &resize, const double &crop_w, const double &crop_h)
        {
            xt::xarray<double> ida_mat = xt::eye<double>(4);
            ida_mat(0, 0) = resize;
            ida_mat(1, 1) = resize;
            ida_mat(0, 3) = -crop_w;
            ida_mat(1, 3) = -crop_h;
            return ida_mat;
        }
        void GeomTool::FillCvMatFromXtensor(const xt::xarray<double> &xtensorMatrix, cv::Mat &cvMatrix)
        {
            for (size_t i = 0; i < xtensorMatrix.shape()[0]; ++i) {
                for (size_t j = 0; j < xtensorMatrix.shape()[1]; ++j) {
                    cvMatrix.at<double>(i, j) = xtensorMatrix(i, j);
                }
            }
        }
        xt::xarray<double> GeomTool::GenerateFrustum(const DetectionConfig &config_data)
        {
            int l_iOgfH = config_data.final_dim.first;
            int l_iOgfW = config_data.final_dim.second;
            int l_iFH = l_iOgfH / config_data.downsample_factor;
            int l_iFW = l_iOgfW / config_data.downsample_factor;
            int l_iSizeD = (config_data.d_bound[1] - config_data.d_bound[0]) / config_data.d_bound[2];

            // Create a range array using xt::arange
            xt::xarray<double> range = xt::arange<double>(config_data.d_bound[0], config_data.d_bound[1], config_data.d_bound[2]);
            // 使用reshape成员函数将range重塑为(-1, 1, 1)的形状
            auto reshaped_bound = range.reshape({static_cast<int>(range.size()), 1, 1});
            // 使用xt::tile在fH和fW轴上复制数组
            auto d_coords = xt::tile(reshaped_bound, {1, l_iFH, l_iFW});
            // 打印d_coords的形状和内容
            // std::cout << "d_coords shape: (" << d_coords.shape()[0] << ", " << d_coords.shape()[1] << ", " << d_coords.shape()[2] << ")" << std::endl;
            // std::cout << d_coords << std::endl;

            xt::xarray<double> rangeSpaceX = xt::arange<double>((config_data.downsample_factor / 2), (l_iOgfW + config_data.downsample_factor / 2),config_data.downsample_factor);
            auto reshaped_rangeSpaceX = rangeSpaceX.reshape({1, 1, l_iFW});
            int D = d_coords.shape()[0];
            auto x_coords = xt::tile(reshaped_rangeSpaceX, { D, l_iFH, 1});
            // std::cout << "x_coords shape: (" << x_coords.shape()[0] << ", " << x_coords.shape()[1] << ", " << x_coords.shape()[2] << ")" << std::endl;
            // std::cout<<"x_coords \n "<<x_coords<<std::endl;

            xt::xarray<double> rangeSpaceY = xt::arange<double>((config_data.downsample_factor / 2), (l_iOgfH + config_data.downsample_factor / 2),config_data.downsample_factor);
            auto reshaped_rangeSpaceY = rangeSpaceY.reshape({1, l_iFH, 1});
            auto y_coords = xt::tile(reshaped_rangeSpaceY, { D, 1, l_iFW});
            // std::cout<<"y_coords \n "<<y_coords<<std::endl;
            // std::cout << "y_coords shape: (" << y_coords.shape()[0] << ", " << y_coords.shape()[1] << ", " << y_coords.shape()[2] << ")" << std::endl;
            xt::xarray<double> paddings = xt::ones_like(d_coords);
            // std::cout << "paddings shape: (" << paddings.shape()[0] << ", " << paddings.shape()[1] << ", " << paddings.shape()[2] << ")" << std::endl;
            
            std::vector<xt::xarray<double>> frustum_mats;
            frustum_mats.emplace_back(x_coords);
            frustum_mats.emplace_back(y_coords);
            frustum_mats.emplace_back(d_coords);
            frustum_mats.emplace_back(paddings);
            xt::xarray<double> frustum_mats_stacked = xt::zeros<double>({ frustum_mats[0].shape()[0], frustum_mats[0].shape()[1],frustum_mats[0].shape()[2],frustum_mats.size()});

            //     // 将 ida_mats 中的每个元素复制到 ida_mats_stacked 中对应的位置
            for (std::size_t i = 0; i < frustum_mats.size(); ++i)
            {
                xt::view(frustum_mats_stacked,  xt::all(), xt::all(), xt::all(),i) = frustum_mats[i];
            }
            // std::cout<<"frustum_mats_stacked shape "<<frustum_mats_stacked.shape()[0]<<" , "<<frustum_mats_stacked.shape()[1]<<" , "<<frustum_mats_stacked.shape()[2]<<" , "<<frustum_mats_stacked.shape()[3]<<std::endl;
            return frustum_mats_stacked;
            // auto frustum = xt::stack(xt::xarray<double>({x_coords, y_coords, d_coords, paddings}), -1);
            // xt::xarray<double> frustum = xt::stack(xt::xtuple(x_coords, y_coords, d_coords, paddings), -1);

        }
        xt::xarray<double> GeomTool::GenerateFrustumByPy(const DetectionConfig &config_data)
        {
            py::module sys = py::module::import("sys");
        // 将Cython编译的Python代码库文件所在目录添加到sys.path中
            std::string library_path = m_base_path_ + "/lib";
            sys.attr("path").attr("append")(library_path);
            py::module module = py::module::import("libParsePkl");
            py::object pkl_parser = module.attr("PklParser")("");
            py::list d_bound;
            for(const auto & data : config_data.d_bound)
            {
                d_bound.append(data);
            }
            py::object frustum = pkl_parser.attr("create_frustum")(config_data.final_dim,config_data.downsample_factor,d_bound);
            // 将 frustum 转换为 NumPy 数组
            py::array_t<double> np_array = frustum.cast<py::array_t<double>>();

            // 确保 NumPy 数组是连续的
            py::buffer_info buffer_info = np_array.request();
            if (buffer_info.ndim != 4) {
                throw std::runtime_error("Expected a 4D NumPy array");
            }

            // 使用 xtensor 的适配函数将 NumPy 数组转换为 xt::xarray<double>
            xt::xarray<double> result = xt::adapt(
                static_cast<double*>(buffer_info.ptr),           // 指向数据的指针
                {static_cast<std::size_t>(buffer_info.shape[0]), // 各维度的大小
                static_cast<std::size_t>(buffer_info.shape[1]),
                static_cast<std::size_t>(buffer_info.shape[2]),
                static_cast<std::size_t>(buffer_info.shape[3])}
            );
            return result;
        }
        void GeomTool::CalVoxelCfg(const DetectionConfig &config_data, xt::xarray<double> &voxel_size, xt::xarray<double> &voxel_coord, xt::xarray<double> &voxel_num)
        {
            // 输入数据
            xt::xarray<double> x_bound = {config_data.x_bound[0], config_data.x_bound[1], config_data.x_bound[2]};
            xt::xarray<double> y_bound = {config_data.y_bound[0], config_data.y_bound[1], config_data.y_bound[2]};
            xt::xarray<double> z_bound = {config_data.z_bound[0], config_data.z_bound[1], config_data.z_bound[2]};
            // 计算 voxel_size
            voxel_size = xt::xarray<double>{x_bound(2), y_bound(2), z_bound(2)};

            
            voxel_coord = xt::xarray<double>{x_bound(0) + x_bound(2) / 2.0, y_bound(0) + y_bound(2) / 2.0, z_bound(0) + z_bound(2) / 2.0};

            
            voxel_num = xt::xarray<double>{round((x_bound(1) - x_bound(0)) / x_bound(2)), round((y_bound(1) - y_bound(0)) / y_bound(2)), round((z_bound(1) - z_bound(0)) / z_bound(2))};
        }
        void GeomTool::DumpGeom(const xt::xarray<double> &ida_mats, const DetectionConfig &config_data,xt::xarray<double> intrin ,xt::xarray<double> extrin,const std::string &file_path)
        {
            auto frustum = GenerateFrustumByPy(config_data);
            // auto frustum = GenerateFrustum(config_data);
            xt::xarray<double> voxel_size, voxel_coord,voxel_num;
            CalVoxelCfg(config_data,voxel_size, voxel_coord,voxel_num);
            // std::cout<<"voxel_size \n"<<voxel_size<<std::endl; 
            // std::cout<<"voxel_coord \n"<<voxel_coord<<std::endl; 
            // std::cout<<"intrin_ \n"<<intrin<<std::endl;
            // std::cout<<"extrin_ \n"<<extrin<<std::endl;
            // std::cout<<"ida_mats \n"<<ida_mats<<std::endl; 
            // std::cout<<"file path is "<<file_path<<std::endl;
            GetGeometry(intrin,extrin,ida_mats,voxel_size,voxel_coord,voxel_num,frustum,file_path);
            // for(const auto &matrix :ida_mat_vec)
            // {
            //     std::cout<<matrix<<"\n";
            // }
            // std::cout<<std::endl;
        }
        void GeomTool::GetGeometry(xt::xarray<double> intrin, xt::xarray<double> extrin, const xt::xarray<double> &ida_mats, const xt::xarray<double> &voxel_size, const xt::xarray<double> &voxel_coord, const xt::xarray<double> &voxel_num,const  xt::xarray<double>  &frustum,const std::string &file_path)
        {
            xt::pyarray<double> py_intrin = intrin;
            xt::pyarray<double> py_extrin = extrin;
            xt::pyarray<double> py_ida_mats = ida_mats;
            xt::pyarray<double> py_voxel_size = voxel_size;
            xt::pyarray<double> py_voxel_coord = voxel_coord;
            xt::pyarray<double> py_voxel_num = voxel_num;
            xt::pyarray<double> py_frustum = frustum;
            py::module sys = py::module::import("sys");
        // 将Cython编译的Python代码库文件所在目录添加到sys.path中
            std::string library_path = m_base_path_ + "/lib";
            sys.attr("path").attr("append")(library_path);
            py::module module = py::module::import("libParsePkl");
            py::object pkl_parser = module.attr("PklParser")("");
            py::object data = pkl_parser.attr("get_geometry")(py_intrin,py_extrin,py_ida_mats,py_voxel_size,py_voxel_coord,py_voxel_num,py_frustum,file_path);
            // py::dict py_dict = data.cast<py::dict>();
            // int batch_size = 1;
            // int num_cams = intrin.shape()[0];
            // xt::xarray<double> intrin_mats = xt::eye<double>({num_cams, 4, 4});
            // // std::cout<<"num_cams \n "<<num_cams<<std::endl;
            // // std::cout<<"intrin_mats \n "<<intrin_mats<<std::endl;
            // std::cout<<"extrin \n "<<extrin<<std::endl;
            // auto intrin_view = xt::view(intrin_mats, xt::all(), xt::range(0, 3), xt::range(0, 3));
            // xt::xarray<double> intrinsic_tiled = xt::tile(intrin, {1, 1, 1});  // intrinsic 不需要重复
            // xt::view(intrin_view, xt::all(), xt::all(), xt::all()) = intrinsic_tiled;
            // std::cout<<"intrin_mats \n"<<intrin_mats<<std::endl;
            // xt::xarray<double> tgt2imgs = xt::empty<double>({int(intrin_mats.shape()[0]), int(intrin_mats.shape()[1]), int(intrin_mats.shape()[2])});
            // for(int i=0 ; i<tgt2imgs.shape()[0] ; ++i)
            // {
            //     auto intrin_mat = xt::view(intrin_mats, i, xt::all(), xt::all());
            //     auto extrin_mat = xt::view(extrin, i, xt::all(), xt::all());
            //     auto result = xt::linalg::dot(intrin_mat,extrin_mat);
            //     xt::view(tgt2imgs, i) = result;
            // }
            // // std::cout<<"befor ida mat \n "<<ida_mats<<std::endl;
            // auto reshaped_ida_mat = xt::reshape_view(ida_mats, {batch_size,num_cams, 1, 1, 1, 4, 4});
            // std::cout<<"after reshaped_ida_mat \n "<<reshaped_ida_mat<<std::endl;
            // for(int i=0 ; i< num_cams ; ++i)
            // {
            //     auto sub_matrix = xt::view(reshaped_ida_mat,0, i, 0,0,0, xt::all(), xt::all());
            //     std::cout << "Sub Matrix i:"<< i <<" \n" << sub_matrix  << std::endl;
            //     auto inv_rotation_scale_matrix = xt::linalg::inv(sub_matrix);
            //     std::cout<<"inv_rotation_scale_matrix \n "<<inv_rotation_scale_matrix<<std::endl;
            // }
            // std::cout<<"inv_ after reshaped_ida_mat "<<reshaped_ida_mat<<std::endl;
            // auto points = frustum;
            // std::cout<<"frustum \n "<<points<<std::endl;
    // for (std::size_t i = 0; i <(int)extrin.shape()[0]; ++i) {
    //     tgt2imgs(i) = xt::linalg::dot(extrin(i), intrin_mats(i));
    // }

        }
        xt::xarray<double> GeomTool::GetSensorTranMatrix(const std::string &camera_name)
        {
            if(m_mapCameraData_.count(camera_name)>0)
            {
                auto extrinsic = m_mapCameraData_[camera_name].second.camera_data_.transform_;
                auto translation = extrinsic.translation_;
                auto rotation = extrinsic.rotation_;
                xt::xarray<long double> quats_array = {{rotation.point_.x_,rotation.point_.y_,rotation.point_.z_,rotation.w_}};
                xt::xarray<long double> trans_array = {{translation.point_.x_,translation.point_.y_,translation.point_.z_}};
                xt::pyarray<double> quats = quats_array;
                xt::pyarray<double> trans = trans_array;
                py::module sys = py::module::import("sys");
            // 将Cython编译的Python代码库文件所在目录添加到sys.path中
                std::string library_path = m_base_path_ + "/lib";
                sys.attr("path").attr("append")(library_path);
                py::module module = py::module::import("libParsePkl");
                py::object data = module.attr("get_sensor_tran_matrix")(quats,trans);
            // 将返回的 NumPy 数组转换为 xt::xarray<double>
                py::array_t<double> np_array = data.cast<py::array_t<double>>();

                // 确保 NumPy 数组是连续的
                py::buffer_info buffer_info = np_array.request();
                if (buffer_info.ndim != 2 || buffer_info.shape[0] != 4 || buffer_info.shape[1] != 4) {
                    throw std::runtime_error("Expected a 4x4 NumPy array");
                }

                // 使用 xtensor 的适配函数将 NumPy 数组转换为 xt::xarray<double>
                xt::xarray<double> result = xt::adapt(
                    static_cast<double*>(buffer_info.ptr),           // 指向数据的指针
                    {static_cast<std::size_t>(buffer_info.shape[0]), // 各维度的大小
                    static_cast<std::size_t>(buffer_info.shape[1])}
                );

                return result;
                // #define use_cpp
                #ifdef use_cpp
                auto rotation_mat = Quat2mat(quats);
                std::cout<<"use cpp trans"<<std::endl;
                // Create a 4x4 transformation matrix
                xt::xarray<double> trans_matrix = xt::eye<double>(4);
                xt::view(trans_matrix, xt::range(0, 3), xt::range(0, 3)) = rotation_mat;
                // std::cout<<"trans "<<trans<<std::endl;
                // std::cout<<"trans_matrix \n"<<trans_matrix<<std::endl;
                for (std::size_t i = 0; i < 3; ++i) {
                    trans_matrix(i, 3) = trans[i];
                }
                return trans_matrix;
                #endif
            }
            return xt::xarray<double>();
        }
        xt::xarray<double> GeomTool::Quat2mat(const xt::xarray<double> &quats)
        {
            xt::xarray<double> rotation = xt::empty<double>({3, 3});
            double x = quats(0), y = quats(1), z = quats(2), w = quats(3);
            rotation(0, 0) = 1 - 2 * (y * y + z * z);
            rotation(0, 1) = 2 * (x * y - z * w);
            rotation(0, 2) = 2 * (x * z + y * w);
            rotation(1, 0) = 2 * (x * y + z * w);
            rotation(1, 1) = 1 - 2 * (x * x + z * z);
            rotation(1, 2) = 2 * (y * z - x * w);
            rotation(2, 0) = 2 * (x * z - y * w);
            rotation(2, 1) = 2 * (y * z + x * w);
            rotation(2, 2) = 1 - 2 * (x * x + y * y);
            return rotation;
        }
        struct PrepareCalibMatrix GeomTool::PrepareCalib()
        {
            std::vector<xt::xarray<double>> intrinsic_mats;
            std::vector<xt::xarray<double>> extrinsic_mats;
            for(const auto &name :m_vecCamera)
            {
                if(name.find("cam_front_120")!=std::string::npos)
                {
                    // std::cout<<"name "<<name <<" int mat is: "<<m_mapIntrinMat_[name]<<std::endl;
                    double* data_ptr = m_mapIntrinMat_[name].ptr<double>();
                    xt::xarray<double> int_mat = xt::adapt(data_ptr, m_mapIntrinMat_[name].total(), xt::no_ownership());
                    int_mat.reshape({m_mapIntrinMat_[name].rows, m_mapIntrinMat_[name].cols});
                    intrinsic_mats.emplace_back(int_mat);
                    // std::cout<<"name "<<name+"_fov30" <<" int mat is: "<<m_mapIntrinMat_[name+"_fov30"]<<std::endl;
                    double* data_ptr_fov30 = m_mapIntrinMat_[name+"_fov30"].ptr<double>();
                    xt::xarray<double> int_mat_fov30 = xt::adapt(data_ptr_fov30, m_mapIntrinMat_[name+"_fov30"].total(), xt::no_ownership());
                    int_mat_fov30.reshape({m_mapIntrinMat_[name].rows, m_mapIntrinMat_[name].cols});
                    intrinsic_mats.emplace_back(int_mat_fov30);
                    extrinsic_mats.emplace_back(GetSensorTranMatrix(name));
                    extrinsic_mats.emplace_back(GetSensorTranMatrix(name));
                    continue;
                }
                extrinsic_mats.emplace_back(GetSensorTranMatrix(name));
                // std::cout<<"name "<<name <<" int mat is: "<<m_mapIntrinMat_[name]<<std::endl;
                double* data_ptr = m_mapIntrinMat_[name].ptr<double>();
                xt::xarray<double> int_mat = xt::adapt(data_ptr, m_mapIntrinMat_[name].total(), xt::no_ownership());
                int_mat.reshape({m_mapIntrinMat_[name].rows, m_mapIntrinMat_[name].cols});
                intrinsic_mats.emplace_back(int_mat);
            }
            xt::xarray<double> intrinsic_mats_stacked = xt::zeros<double>({intrinsic_mats.size(), intrinsic_mats[0].shape()[0], intrinsic_mats[0].shape()[1]});
            xt::xarray<double> extrinsic_mats_stacked = xt::zeros<double>({extrinsic_mats.size(), extrinsic_mats[0].shape()[0], extrinsic_mats[0].shape()[1]});
                            // 将 ida_mats 中的每个元素复制到 ida_mats_stacked 中对应的位置
            for (std::size_t i = 0; i < intrinsic_mats.size(); ++i)
            {
                xt::view(intrinsic_mats_stacked, i, xt::all(), xt::all()) = intrinsic_mats[i];
            }
            for (std::size_t i = 0; i < extrinsic_mats.size(); ++i)
            {
                xt::view(extrinsic_mats_stacked, i, xt::all(), xt::all()) = extrinsic_mats[i];
            }
            // std::cout<<"intrinsic_mats_stacked \n "<<intrinsic_mats_stacked<<std::endl;
            // std::cout<<"intrinsic_mats_stacked shape " <<intrinsic_mats_stacked.shape()[0] <<"*" <<intrinsic_mats_stacked.shape()[1] << "*" <<intrinsic_mats_stacked.shape()[2] <<std::endl;
            // std::cout<<"extrinsic_mats_stacked shape " <<extrinsic_mats_stacked.shape()[0] <<"*" <<extrinsic_mats_stacked.shape()[1] << "*" <<extrinsic_mats_stacked.shape()[2] <<std::endl;
            // std::cout<<"intrinsic_mats_stacked \n"<<intrinsic_mats_stacked<<std::endl;
            // std::cout<<"extrinsic_mats_stacked \n"<<extrinsic_mats_stacked<<std::endl;
            PrepareCalibMatrix matrix;
            matrix.intrinsic_mats = intrinsic_mats_stacked;
            matrix.extrinsic_mats = extrinsic_mats_stacked;
            return matrix;
        }
    } // namespace tool
} // namespace mach