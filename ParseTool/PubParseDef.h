#ifndef __TOOL_PARSETOOLS_PUBDEF_H
#define __TOOL_PARSETOOLS_PUBDEF_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <cstdlib>
namespace mach
{
    namespace tool
    {
        enum ParseFileType
        {
            CAMERA_PARAMS ,
            EGO_PARAMS ,
            GNSS_PARAMS ,
            LIDAR_PARAMS ,
            DEFAULT
        };
        enum GeomConfigType
        {
            GEOM_BYD_CONFIG ,
            GEOM_HS_CONFIG ,
            DEFAULT_CONFIG
        };
        enum CameraParamType
        {
            EXTRINSIC ,
            INTRINSIC ,
            ERROR
        };
        typedef struct point3D
        {
            long double x_;
            long double y_;
            long double z_;
            point3D():x_(0),y_(0),z_(0){}
            point3D(const long double & x, const long double & y, const long double & z):
            x_(x),y_(y),z_(z){}
            friend std::ostream& operator<<(std::ostream& os, const point3D& v) {
                os << std::left << "point3D x: " << std::right << v.x_ << std::endl;
                os << std::left << "point3D y: " << std::right << v.y_ << std::endl;
                os << std::left << "point3D z: " << std::right << v.z_ << std::endl;
                return os;
            }
        }Point3D;
        typedef struct translation
        {
            Point3D point_;
            translation():point_(point3D()){}
            translation(const long double & x, const long double & y, const long double & z):
            point_(point3D(x,y,z)){

            }
            friend std::ostream& operator<<(std::ostream& os, const translation& v) {
                os << std::left << "translation x: " << std::right << v.point_.x_ << std::endl;
                os << std::left << "translation y: " << std::right << v.point_.y_ << std::endl;
                os << std::left << "translation z: " << std::right << v.point_.z_ << std::endl;
                return os;
            }

        }Translation;
        typedef struct rotation
        {
            Point3D point_;
            long double w_;
            rotation():point_(point3D()),w_(0){}
            rotation(const long double & x, const long double & y, const long double & z , const long double & w):
            point_(point3D(x,y,z)),w_(w){

            }
            friend std::ostream& operator<<(std::ostream& os, const rotation& v) {
                os << std::left << "rotation x: " << std::right << v.point_.x_ << std::endl;
                os << std::left << "rotation y: " << std::right << v.point_.y_ << std::endl;
                os << std::left << "rotation z: " << std::right << v.point_.z_ << std::endl;
                os << std::left << "rotation w: " << std::right << v.w_ << std::endl;
                return os;
            }
        }Rotation;
        //in
        typedef struct  resolution
        {
            uint width_;
            uint height_;
            resolution():width_(0),height_(0){}
            resolution(const uint & width , const uint &height):width_(width),height_(height){}
            friend std::ostream& operator<<(std::ostream& os, const resolution& v) {
                os << std::left << "resolution width: " << std::right << v.width_ << std::endl;
                os << std::left << "resolution height: " << std::right << v.height_ << std::endl;
                return os;
            }
            /* data */
        }Resolution;
        
        typedef struct transform
        {
            Translation translation_;
            Rotation rotation_;
            transform():translation_(Translation()),rotation_(Rotation()){}
            friend std::ostream& operator<<(std::ostream& os, const transform& v) {
                os << v.translation_ << std::endl;
                os << v.rotation_ << std::endl;
                return os;
            }
        }Transform;
        typedef struct cameraParams
        {
            /* data */
            CameraParamType type;
            //if EXTRINSIC
            Transform transform_;
            Point3D euler_degree_;
            uint8_t calib_status_;
            std::string information_;
            std::string calib_time_;
            //if INTRINSIC
            Resolution resolution_;
            std::string distortion_model_;
            std::vector<std::vector<long double>> K;
            std::vector<long double> D;
            friend std::ostream& operator<<(std::ostream& os, const cameraParams& v) {
                if(v.type == CameraParamType::EXTRINSIC)
                {
                    os << v.transform_ << std::endl;
                    os << v.euler_degree_ << std::endl;
                    os << std::left << "calib_status_ : " << std::right << (int)v.calib_status_ << std::endl;
                    os << std::left << "information_  " << std::right << v.information_ << std::endl;
                    os << std::left << "calib_time_  " << std::right << v.calib_time_ << std::endl;
                }
                if(v.type == CameraParamType::INTRINSIC)
                {
                    os << v.resolution_ << std::endl;
                    os << std::left << "distortion_model_ : " << std::right << v.distortion_model_ << std::endl;
                    os << "K array [ " << std::endl;
                    for(const auto &data : v.K)
                    {
                        os << " [ " << std::endl;
                        for(const auto & point :data)
                        {
                            os << point <<" , " << std::endl;
                        }
                        os << " ] "<<std::endl;
                    }
                    os << "]" << std::endl;
                    os << "D array [ " <<std::endl;
                    for(const auto &data : v.D)
                    {
                        os << " [ " << std::endl;
                        os << data <<" ], " << std::endl;
                    }
                    os << "]" << std::endl;
                }
                return os;
            }
        }CameraParams;
        typedef struct egoParams
        {
            Transform ego_footprint;
            friend std::ostream& operator<<(std::ostream& os, const egoParams& v) {
                os << v.ego_footprint << std::endl;
                return os;
            }
        }EgoParams;
        typedef struct gnssParams
        {
            Transform transform_;
            Point3D euler_degree_;
            uint8_t calib_status_;
            std::string information_;
            std::string calib_time_;
            friend std::ostream& operator<<(std::ostream& os, const gnssParams& v) {
                os << v.transform_ << std::endl;
                os << v.euler_degree_ << std::endl;
                os << std::left << "calib_status_ : " << std::right << (int)v.calib_status_ << std::endl;
                os << std::left << "information_ : " << std::right << v.information_ << std::endl;
                os << std::left << "calib_time_ : " << std::right << v.calib_time_ << std::endl;
                return os;
            }
        }GnssParams;
        typedef struct lidarParams
        {
            Transform transform_;
            Point3D euler_degree_;
            uint8_t calib_status_;
            std::string information_;
            std::string calib_time_;
            friend std::ostream& operator<<(std::ostream& os, const lidarParams& v) {
                os << v.transform_ << std::endl;
                os << v.euler_degree_ << std::endl;
                os << std::left << "calib_status_ : " << std::right << (int)v.calib_status_ << std::endl;
                os << std::left << "information_ : " << std::right << v.information_ << std::endl;
                os << std::left << "calib_time_ : " << std::right << v.calib_time_ << std::endl;
                return os;
            }
        }LidarParams;
        
        typedef struct  paramsData
        {
            ParseFileType type_;
            /* data */
            CameraParams camera_data_;
            EgoParams ego_data_;
            GnssParams gnss_data_;
            LidarParams lidar_data_;
            friend std::ostream& operator<<(std::ostream& os, const paramsData& v) {
                if(v.type_==CAMERA_PARAMS)
                {
                    os << std::left << "ParamsData CameraParams " << std::right << 000 << std::endl;
                    os << v.camera_data_ <<std::endl;
                }
                if(v.type_==EGO_PARAMS)
                {
                    os << std::left << "ParamsData EgoParams " << std::right << 000 << std::endl;
                    os << v.ego_data_ <<std::endl;
                }
                if(v.type_==GNSS_PARAMS)
                {
                    os << std::left << "ParamsData GnssParams " << std::right << 000 << std::endl;
                    os << v.gnss_data_ <<std::endl;
                }
                if(v.type_==LIDAR_PARAMS)
                {
                    os << std::left << "ParamsData LidarParams " << std::right << 000 << std::endl;
                    os << v.lidar_data_ <<std::endl;
                }
                return os;}
        }ParamsData;     

        typedef struct runParam
        {
            std::string src_calib_path_;
            std::string dst_config_path_;
            std::string car_id_;
            std::string base_path_;
            std::string config_type_;
            bool run_camera_preprocess_;
            bool run_conflate_models_;
            bool use_test_;
            runParam()
            {
                src_calib_path_ = "";
                dst_config_path_ = "";
                car_id_ = "";
                base_path_ = "";
                config_type_ = "";
                run_camera_preprocess_ = false;
                run_conflate_models_ = false;
                use_test_ = false;
            }
            /* data */
        }RunParam;
        
        struct IDACfg {
            std::pair<int, int> final_HW;
            std::pair<int, int> HW;
            std::pair<double, double> bot_pct_lim;
        };

        struct DetectionConfig {
            std::pair<int, int> final_dim;
            int downsample_factor;
            std::vector<double> x_bound;
            std::vector<double> y_bound;
            std::vector<double> z_bound;
            std::vector<double> d_bound;
            std::vector<IDACfg> ida_cfg;
        };
    }
}
#endif //__TOOL_PARSETOOLS_PUBDEF_H