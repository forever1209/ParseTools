#include "ParseEgo.h"

namespace mach
{
    namespace tool
    {
        EgoParse::EgoParse(const ParseFileType &type):BaseParse(type)
        {
            
        }
        EgoParse::~EgoParse()
        {
            std::cout << "EgoParse Release " << std::endl;
        }
        bool EgoParse::Open(const std::string &file)
        {
            if(m_emType_!=ParseFileType::EGO_PARAMS)
            {
                return false;
            }
            m_strFileUrl = file;
            return ParseData();
        }
        bool EgoParse::ParseData()
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
                        if(m_emType_ == ParseFileType::EGO_PARAMS && m_data_.type_ == m_emType_)
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
                                m_data_.ego_data_.ego_footprint.translation_ = Translation(translation_x,translation_y,translation_z);
                                m_data_.ego_data_.ego_footprint.rotation_ = Rotation(rotation_x,rotation_y,rotation_z,rotation_w);
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
    }
}

