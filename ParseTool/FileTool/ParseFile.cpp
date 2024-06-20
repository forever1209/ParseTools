#include "ParseFile.h"
#include "PubParseDef.h"
namespace mach
{
    namespace tool
    {
        bool IsPathExist(const std::string &p_strFilePath)
        {
            bool bIsExist = (0 == access(p_strFilePath.c_str(), F_OK));
            return bIsExist;
        }
        std::vector<std::string> GetAllDirs(const std::string &dirPath)
        {
            std::vector<std::string> dirs;
            if(dirPath.empty())
            {
                return dirs;
            }
            DIR* dir = opendir(dirPath.c_str());
            if (dir != nullptr) {
                dirent* entry;
                while ((entry = readdir(dir)) != nullptr) {
                    if (entry->d_type == DT_DIR && std::string(entry->d_name) != "." && std::string(entry->d_name) != "..") {
                        dirs.push_back(entry->d_name);
                    }
                }
                closedir(dir);
            }
            return dirs;
        }
        std::vector<std::string> GetAllFiles(const std::string &dirPath)
        {
            std::vector<std::string> files;
            DIR *dir;
            struct dirent *ent;
            if ((dir = opendir(dirPath.c_str()))!= NULL) {
                while ((ent = readdir(dir))!= NULL) {
                    if (ent->d_type == DT_REG) {
                        files.emplace_back(ent->d_name);
                    }
                }
                closedir(dir);
            } else {
                std::cerr << "Could not open directory " << dirPath << std::endl;
            }
            return files;
            std::cout<<"end"<<std::endl;
        }
        std::vector<std::string> SplitString(const std::string &str ,char separator)
        {
            std::stringstream ss(str);
            std::vector<std::string> parts;
            std::string partBefore;
            while (std::getline(ss, partBefore, separator)) {
                parts.push_back(partBefore);
            }
            return parts;
        }
        FileSysParse::FileSysParse()
        {
            m_strBaseDir = "";
            m_strBaseConfigDir = "";
            m_vecParamsPath_.clear();
        }
        FileSysParse::~FileSysParse()
        {
            std::vector<std::string> l_vecTemp;
            m_vecParamsPath_.clear();
            m_vecParamsPath_.swap(l_vecTemp);
            std::cout << "FileSysParse Release " << std::endl;
        }
        bool FileSysParse::CheckParamDirExist(const std::string &dir)
        {
            if(IsPathExist(dir))
            {
                m_strBaseDir = dir;
                std::vector<std::string> dirs = GetAllDirs(dir);
                auto it = dirs.begin();
                //过滤，只获取params的最新文件夹下内容
                while (it != dirs.end()) 
                {
                    if ((*it).find("params") != std::string::npos) {
                        ++it;
                    } else {
                        it = dirs.erase(it);
                    }
                }
                m_vecParamsPath_.assign(dirs.begin(),dirs.end());
                return true;
            }
            return false;
        }
        void FileSysParse::ParseFileNamesByType(const std::unordered_map<std::string, std::string> &path_list_map)
        {
            if(path_list_map.empty())
            {
                if(m_vecParamsPath_.empty())
                {
                    return;
                }
                for(const auto &dir_name :m_vecParamsPath_)
                {
                    // std::cout<<"dir name is: "<<dir_name<<std::endl;
                    auto l_vecType = SplitString(dir_name,'_');
                    if(l_vecType.size()>1)
                    {
                        std::string l_strFilePath = m_strBaseDir + "/" + dir_name + "/";
                        auto l_vecFiles = GetAllFiles(l_strFilePath);
                        if(l_vecType[0]=="ego")
                        {
                            std::unordered_map<std::string,std::vector<std::string>> l_mapEgo;
                            l_mapEgo[l_strFilePath] = l_vecFiles;
                            m_mapTypeFile_["ego"] = l_mapEgo;
                        }
                        if(l_vecType[0]=="camera")
                        {
                            std::unordered_map<std::string,std::vector<std::string>> l_mapCamera;
                            l_mapCamera[l_strFilePath] = l_vecFiles;
                            m_mapTypeFile_["camera"] = l_mapCamera;
                        }
                        if(l_vecType[0]=="gnss")
                        {
                            std::unordered_map<std::string,std::vector<std::string>> l_mapGnss;
                            l_mapGnss[l_strFilePath] = l_vecFiles;
                            m_mapTypeFile_["gnss"] = l_mapGnss;
                        }
                        if(l_vecType[0]=="lidar")
                        {
                            std::unordered_map<std::string,std::vector<std::string>> l_mapLidar;
                            l_mapLidar[l_strFilePath] = l_vecFiles;
                            m_mapTypeFile_["lidar"] = l_mapLidar;
                        }

                    }
                    // if()
                }
            }
        }
        bool FileSysParse::CheckConfigDirExist(const std::string &dir)
        {
            if(IsPathExist(dir))
            {
                m_strBaseConfigDir = dir;
                auto l_vecFiles = GetAllFiles(m_strBaseConfigDir);
                auto it = l_vecFiles.begin();
                std::vector<std::string > l_vecTomlFiles;
                std::vector<std::string > l_vecYmalFiles;
                //过滤
                while (it != l_vecFiles.end()) 
                {
                    if ((*it).find(".toml") != std::string::npos) {
                        l_vecTomlFiles.emplace_back(m_strBaseConfigDir + "/" + *it);
                        ++it;
                    }
                    else if ((*it).find(".yaml") != std::string::npos)
                    {
                        l_vecYmalFiles.emplace_back(m_strBaseConfigDir + "/" + *it);
                        ++it;
                    }
                    else
                    {
                        it = l_vecFiles.erase(it);
                    }
                }
                m_mapTypeConfig_["toml"] = l_vecTomlFiles;
                m_mapTypeConfig_["yaml"] = l_vecYmalFiles;
                return true;
            }
            return false;
        }
        const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> FileSysParse::GetParamTypeFiles() const
        {
            return m_mapTypeFile_;
        }
        const std::unordered_map<std::string, std::vector<std::string>> FileSysParse::GetConfigTypeFiles() const
        {
            return m_mapTypeConfig_;
        }
        const std::string FileSysParse::GetFileName(const std::string &path)
        {
            auto l_vecStr = SplitString(path,'/');
            if(l_vecStr.empty())
            {
                std::cout<<"empty"<<std::endl;
                return "";
            }
            return *(l_vecStr.end()-1) ;
        }
    } // namespace name

} // namespace name

