#ifndef __TOOL_PREPROCESSTOOLS_CAMERAPREPROCESS_H
#define __TOOL_PREPROCESSTOOLS_CAMERAPREPROCESS_H
#include <iostream>
#include <unordered_map>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "PubParseDef.h"
namespace mach
{
    namespace tool
    {
        enum PREPROCESSTYPE
        {
            HS_TYPE,
            BYD_TYPE
        };
        class CameraPreprocess
        {
        public:
            CameraPreprocess();
            ~CameraPreprocess();
            bool CheckCalibInfoExist(const std::string & file,const std::string &vechile_id = "");
        private:
            bool Preprocess(const std::string &path , const PREPROCESSTYPE & type);
            bool PreprocessHs(const std::unordered_map<std::string , ParamsData> & params);
            bool PreprocessByd(const std::unordered_map<std::string , ParamsData> & params);
            std::vector<std::string> GetAllFilesByCamlist(const std::string &path);
            bool CreatePathIfNotExist(const std::string & path);
            void WriteMapBinFile(const std::string &name , const cv::Mat &map_x ,  const cv::Mat &map_y , const int &x , const int &y);
            bool GenerateAllinoneMap(const std::string &map_bin , const std::string &map_name ,const std::string & input_name);
            bool IsLittleEndian() ;
            void ToLittleEndian(float value, uint8_t* buffer);
        private:
            std::vector<std::string> m_vecCamera = {
                "cam_front_120",
                "cam_back_200",
                "cam_front_200",
                "cam_right_200",
                "cam_left_200",
                "cam_back_100"};
            std::map<std::string, std::string> m_mapCamMap = {
                {"cam_front_120", "front_1408x512_120"},
                {"cam_front_120_30", "front_1408x512_30"},
                {"cam_back_200","back_704x256_200"},
                {"cam_front_200","front_704x256_200"},
                {"cam_right_200","right_704x256_200"},
                {"cam_left_200","left_704x256_200"},
                {"cam_back_100","back_1408x512_70"}
            };
            std::map<std::string, cv::Mat> sim_fov_intrinsic = {
                {"fov120", (cv::Mat_<double>(3, 3) << 1.85546934e+03, 0.00000000e+00, 1.91230452e+03,
                                                    0.00000000e+00, 1.85257078e+03, 1.04834280e+03,
                                                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00)},
                {"fov70",  (cv::Mat_<double>(3, 3) << 3.17605988e+03, 0.00000000e+00, 1.90708104e+03,
                                                    0.00000000e+00, 3.16498729e+03, 1.11712543e+03,
                                                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00)},
                {"fov60",  (cv::Mat_<double>(3, 3) << 1.63730627e+03, 0.00000000e+00, 9.32765191e+02,
                                                    0.00000000e+00, 1.85440344e+03, 4.96221947e+02,
                                                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00)},
                {"fov30",  (cv::Mat_<double>(3, 3) << 7.13449591e+03, 0.00000000e+00, 2.07952416e+03,
                                                    0.00000000e+00, 7.10720219e+03, 1.25206216e+03,
                                                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00)},
                {"fov190", (cv::Mat_<double>(3, 3) << 9.00e+02, 0.00e+00, 1.84e+03,
                                                    0.00e+00, 1.00e+03, 1.10e+03,
                                                    0.00e+00, 0.00e+00, 1.00e+00)}
            };
            std::string m_strBasePath_ = "";
        };
    } // namespace tool
    
} // namespace mach
#endif //__TOOL_PREPROCESSTOOLS_CAMERAPREPROCESS_H