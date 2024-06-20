#ifndef __TOOL_GEOMTOOL_H
#define __TOOL_GEOMTOOL_H
#include <iostream>
#include <iomanip>
#include <PubParseDef.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
#include <opencv2/core.hpp>
#include <xtensor.hpp>
namespace mach
{
    namespace tool
    {
        struct PrepareCalibMatrix
        {
            xt::xarray<double> intrinsic_mats; 
            xt::xarray<double> extrinsic_mats; 
        };
        namespace py = pybind11;
        class PklParse
        {
        public:
            PklParse(const std::string &path){m_strBasePath_ = path;}
            ~PklParse(){std::cout <<"release PklParse"<<std::endl;}
            xt::xarray<double>   GetPklDataXt(const std::string &pkl_file,const std::string &str_key)
            {
                xt::xarray<double> empty_array;
                try {
                    py::module sys = py::module::import("sys");
                // 将Cython编译的Python代码库文件所在目录添加到sys.path中
                    std::string library_path = m_strBasePath_ + "/lib";
                    sys.attr("path").attr("append")(library_path);
                    py::module module = py::module::import("libParsePkl");
                    py::object pkl_parser = module.attr("PklParser")(pkl_file);
                    py::object data = pkl_parser.attr("get_data")();
                    py::dict py_dict = data.cast<py::dict>();
                    for (auto item : py_dict) {
                        std::string key = item.first.cast<std::string>();
                        if(key.find(str_key)!= std::string::npos)
                        {
                            // std::cout<<"key is "<<key<<std::endl;
                            if (py::isinstance<py::array>(item.second)) 
                            {
                                py::array_t<double> array = item.second.cast<py::array_t<double>>();
                                // std::cout<<"array "<<array<<std::endl;
                                auto buffer_info = array.request();
                                double *ptr = static_cast<double *>(buffer_info.ptr);
                                // Convert to Xtensor array
                                xt::xarray<double> xt_array = xt::adapt(ptr, buffer_info.size,
                                                                        xt::no_ownership(),
                                                                        std::vector<std::size_t>{buffer_info.shape[0], buffer_info.shape[1]});
                                return xt_array;
                            }
                            break;
                        }
                    }
                    return empty_array;
                } catch (py::error_already_set& e) {
                    // 捕获并打印 Python 异常
                    PyErr_Print();
                    return empty_array;
                }
            }
        private:
            std::string m_strBasePath_ = "";
        };
        class GeomTool
        {
        public:
            struct VehicleConfig
            {
                DetectionConfig det_fb_config;
                DetectionConfig det_lr_config;
                DetectionConfig lane_config;
            };
            GeomTool();
            ~GeomTool();
            bool InitTool(const GeomConfigType & type , const std::string &base_path);
            void ConflateData(const std::string &pkl_path ,const std::string &vehicle_id ,const std::string &config_path);
        private:
            bool InitCameraInfo(const std::string &config_path);
            bool InitImageMapping(const std::string &pkl_path);
            bool CheckCalibExist(const std::string & config_path);
            double Mean(const std::pair<double,double>& datas);
            xt::xarray<double> CreateIdaMat(const double& resize, const double &crop_w, const double &crop_h) ;
            void FillCvMatFromXtensor(const xt::xarray<double> & xtensorMatrix, cv::Mat& cvMatrix);
            xt::xarray<double> GenerateFrustum(const DetectionConfig & config_data);
            xt::xarray<double> GenerateFrustumByPy(const DetectionConfig & config_data);
            void CalVoxelCfg(const DetectionConfig & config_data,xt::xarray<double> &voxel_size,xt::xarray<double> &voxel_coord,xt::xarray<double> &voxel_num);
            void DumpGeom(const xt::xarray<double>& ida_mats,const DetectionConfig &config_data,xt::xarray<double> intrin ,xt::xarray<double> extrin,const std::string &file_path);
            void GetGeometry(xt::xarray<double> intrin ,xt::xarray<double> extrin,const xt::xarray<double> &ida_mats,const xt::xarray<double> &voxel_size,const xt::xarray<double> &voxel_coord,const xt::xarray<double> &voxel_num,const  xt::xarray<double>  &frustum,const std::string &file_path);
            xt::xarray<double> GetSensorTranMatrix(const std::string &camera_name);
            xt::xarray<double> Quat2mat(const xt::xarray<double>& quats);
            PrepareCalibMatrix PrepareCalib();
        private:
            py::object m_pPklTool_ ;
            GeomConfigType m_enConfigType_ = GeomConfigType::DEFAULT_CONFIG;
            std::string m_base_path_ = "";
            DetectionConfig byd_det_fb_config_;
            DetectionConfig byd_det_lr_config_;
            DetectionConfig byd_lane_config_;
            DetectionConfig hs_det_fb_config_;
            DetectionConfig hs_det_lr_config_;
            DetectionConfig hs_lane_config_;
            std::vector<std::string> m_vecCamera = {
                "cam_front_200",
                "cam_back_200",
                "cam_left_200",
                "cam_right_200",
                "cam_front_120",
                "cam_back_100"};
            VehicleConfig m_stVehicleConfig_;
            std::unordered_map<std::string,std::pair<ParamsData,ParamsData>> m_mapCameraData_ ;
            std::map<std::string,cv::Mat> m_mapIntrinMat_;
        };
    } // namespace tool
    
} // namespace mach


#endif //__TOOL_GEOMTOOL_H