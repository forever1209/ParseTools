#include "ParseLidar.h"
namespace mach
{
    namespace tool
    {
        LidarParse::LidarParse(const ParseFileType &type):BaseParse(type)
        {

        }
        LidarParse::~LidarParse()
        {
            std::cout << "LidarParse Release " << std::endl;
        }
        bool LidarParse::Open(const std::string &file)
        {
            if(m_emType_!=ParseFileType::LIDAR_PARAMS)
            {
                return false;
            }
            m_strFileUrl = file;
            return ParseData();
        }
        bool LidarParse::ParseData()
        {
            std::ifstream stream(m_strFileUrl);
            if(stream.is_open())
            {
                try{
                    Json::Reader reader;
                    Json::Value root;
                    bool l_bOk = reader.parse(stream, root);
                    if(l_bOk)
                    {
                        if(m_emType_ == ParseFileType::LIDAR_PARAMS && m_data_.type_ == m_emType_)
                        {
                            if (root.isMember("transform"))
                            {
                                long double translation_x = root["transform"]["translation"]["x"].asDouble();
                                long double translation_y = root["transform"]["translation"]["y"].asDouble();
                                long double translation_z = root["transform"]["translation"]["z"].asDouble();
                                long double rotation_x = root["transform"]["rotation"]["x"].asDouble();
                                long double rotation_y = root["transform"]["rotation"]["y"].asDouble();
                                long double rotation_z = root["transform"]["rotation"]["z"].asDouble();
                                long double rotation_w = root["transform"]["rotation"]["w"].asDouble();
                                long double euler_degree_RotX = root["euler_degree"]["RotX"].asDouble();
                                long double euler_degree_RotY = root["euler_degree"]["RotY"].asDouble();
                                long double euler_degree_RotZ = root["euler_degree"]["RotZ"].asDouble();
                                m_data_.lidar_data_.transform_.translation_ = Translation(translation_x,translation_y,translation_z);
                                m_data_.lidar_data_.transform_.rotation_ = Rotation(rotation_x,rotation_y,rotation_z,rotation_w);
                                m_data_.lidar_data_.euler_degree_ = Point3D(euler_degree_RotX,euler_degree_RotY,euler_degree_RotZ);
                                m_data_.lidar_data_.calib_status_ = root["calib_status"].asUInt();
                                m_data_.lidar_data_.information_ = root["information"].asString();
                                m_data_.lidar_data_.calib_time_ = root["calib_time"].asString();
                                stream.close();
                                return true;
                            }
                        }
                    }
                    else
                    {
                        std::cerr << "Failed to parse JSON" << std::endl;
                        stream.close();
                        return false;
                    }
                }catch (const std::exception& e) {
                    // 捕获解析过程中抛出的异常
                    std::cerr << "Exception caught: " << e.what() << std::endl;
                }
                stream.close();
            }
            std::cerr<<" can not find json file "<<std::endl;
            return false;
        }
    } // namespace tool

} // namespace mach

