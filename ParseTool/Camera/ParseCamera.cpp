#include "ParseCamera.h"
namespace mach
{
    namespace tool
    {
        CameraParse::CameraParse(const ParseFileType &type):BaseParse(type)
        {

        }

        CameraParse::~CameraParse()
        {
            std::cout << "CameraParse Release " << std::endl;
        }
        bool CameraParse::Open(const std::string &file)
        {
            if(m_emType_!=ParseFileType::CAMERA_PARAMS)
            {
                return false;
            }
            m_strFileUrl = file;
            return ParseData();
        }
        bool CameraParse::ParseData()
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
                        if(m_emType_ == ParseFileType::CAMERA_PARAMS && m_data_.type_ == m_emType_)
                        {
                            if(root.isMember("resolution"))
                            {
                                //内参
                                m_data_.camera_data_.type = CameraParamType::INTRINSIC;
                                m_data_.camera_data_.distortion_model_ = root["distortion_model"].asString();
                                m_data_.camera_data_.resolution_.width_ = root["resolution"][0].asUInt();
                                m_data_.camera_data_.resolution_.height_ = root["resolution"][1].asUInt();
                                if(root["K"].isArray())
                                {
                                    const int size = root["K"].size();
                                    std::vector<std::vector<long double>> temp;
                                    m_data_.camera_data_.K.clear();
                                    m_data_.camera_data_.K.swap(temp);
                                    for(uint i = 0 ; i < size ; ++i)
                                    {
                                        std::vector<long double> l_vecK;
                                        l_vecK.emplace_back(std::stold(root["K"][i][0].asString()));
                                        l_vecK.emplace_back(std::stold(root["K"][i][1].asString()));
                                        l_vecK.emplace_back(std::stold(root["K"][i][2].asString()));
                                        m_data_.camera_data_.K.emplace_back(l_vecK);
                                    }
                                }
                                if(root["D"].isArray())
                                {
                                    const int size = root["D"].size();
                                    std::vector<long double> temp;
                                    m_data_.camera_data_.D.clear();
                                    m_data_.camera_data_.D.swap(temp);
                                    for(uint i = 0 ; i < size ; ++i)
                                    {
                                        std::vector<long double> l_vecK;
                                        m_data_.camera_data_.D.emplace_back(std::stold(root["D"][i][0].asString()));
                                    }
                                }
                                // std::cout<<"parse camera INTRINSIC"<<std::endl;
                                // std::cout << m_data_<<std::endl;
                                stream.close();
                                return true;
                            }
                            else if (root.isMember("transform"))
                            {
                                //外参
                                m_data_.camera_data_.type = CameraParamType::EXTRINSIC;
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
                                m_data_.camera_data_.transform_.translation_ = Translation(translation_x,translation_y,translation_z);
                                m_data_.camera_data_.transform_.rotation_ = Rotation(rotation_x,rotation_y,rotation_z,rotation_w);
                                m_data_.camera_data_.euler_degree_ = Point3D(euler_degree_RotX,euler_degree_RotY,euler_degree_RotZ);
                                m_data_.camera_data_.calib_status_ = root["calib_status"].asUInt();
                                m_data_.camera_data_.information_ = root["information"].asString();
                                m_data_.camera_data_.calib_time_ = root["calib_time"].asString();
                                // std::cout << m_data_ << std::endl;
                                // std::cout<<"parse camera EXTRINSIC"<<std::endl;
                                stream.close();
                                return true;
                            }
                            else
                            {
                                std::cout<<"输入的不是相机配置文件"<<std::endl;
                                stream.close();
                                return false;
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
    }
}


