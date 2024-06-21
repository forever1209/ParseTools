#include "ModelPathReWrite.h"
#include <toml.hpp>
#include <regex>
namespace mach
{
    namespace tool
    {
        ModelPathReWriteTool::ModelPathReWriteTool(const std::string & allinone_path):m_strAllinonePath_(allinone_path)
        {
            std::cout <<" ModelPathReWriteTool Init"<<std::endl;
            InitTxtPathVec();
        }

        ModelPathReWriteTool::~ModelPathReWriteTool()
        {
            std::cout<<"ModelPathReWriteTool release "<<std::endl;
        }
        void ModelPathReWriteTool::UpdateModelsPath(const std::string &config_path, const std::string &new_model_path)
        {
            std::vector<std::string> files;
            DIR *dir;
            struct dirent *ent;
            if ((dir = opendir(config_path.c_str()))!= NULL) {
                while ((ent = readdir(dir))!= NULL) {
                    if (ent->d_type == DT_REG) {
                        std::string l_strFileName = ent->d_name;
                        auto it = std::find_if(m_vecConfigPath_.begin(), m_vecConfigPath_.end(),
                        [&l_strFileName](const std::string &file_path)
                        {
                            return (file_path.find(l_strFileName) != std::string::npos);
                        });
                        if(it!=m_vecConfigPath_.end())
                        {
                            files.emplace_back(l_strFileName);
                        }
                    }
                }
                closedir(dir);
            } else {
                std::cerr << "Could not open directory " << config_path << std::endl;
            }
            for(const auto & path : files)
            {
                std::cout<<"path is "<<path <<std::endl;
                std::string l_strConfig = config_path + "/" + path;

                ParseFileUpdate(l_strConfig,new_model_path);
            }
            UpdateAllinonePath(m_strAllinonePath_,new_model_path);
        }
        void ModelPathReWriteTool::InitTxtPathVec()
        {
            std::vector<std::string> l_vecPath;
            if(!m_strAllinonePath_.empty())
            {
                auto l_tmData = toml::parse(m_strAllinonePath_);
                auto &l_tmPerceptionParameters = toml::find(l_tmData, "perception_parameters");
                auto l_tmFrontBackModelPath = toml::find<std::string>(l_tmPerceptionParameters, "front_back_model_path");
                auto l_tmSideModelPath = toml::find<std::string>(l_tmPerceptionParameters, "side_model_path");
                auto l_tmDetHeadModelPath = toml::find<std::string>(l_tmPerceptionParameters, "det_head_model_path");
                // auto l_tmFrontBackModelPath = toml::find<std::string>(l_tmPerceptionParameters, "rv_model_path");
                auto l_tmLaneHeadkModelPath = toml::find<std::string>(l_tmPerceptionParameters, "lane_head_model_path");
                l_vecPath.emplace_back(l_tmFrontBackModelPath);
                l_vecPath.emplace_back(l_tmSideModelPath);
                l_vecPath.emplace_back(l_tmDetHeadModelPath);
                l_vecPath.emplace_back(l_tmLaneHeadkModelPath);
                m_vecConfigPath_ = l_vecPath;
            }
        }
        void ModelPathReWriteTool::ParseFileUpdate(const std::string &config_path, const std::string &new_model_path)
        {
            std::string line, key, val;
            std::ifstream file(config_path);
            std::vector<std::string> lines;

            if (file.is_open()) {
                // 读取所有行到内存中
                while (getline(file, line)) {
                    lines.push_back(line);
                }
                file.close();
            } else {
                std::cerr << "Failed to open the file." << std::endl;
                return;
            }

            // 遍历每一行，查找并修改指定的行
            for (auto& line : lines) {
                // 跳过空行和注释行
                if (line.empty() || line.find("*") != std::string::npos)
                    continue;

                std::istringstream linestream(line);
                if (linestream >> key >> val) {
                    if (val.empty()) {
                        continue;
                    }
                    
                    if (key == "model:") {
                        // std::cout << "model name is " << val << std::endl;
                        // std::cout << "new model_path " << new_model_path << std::endl;
                        std::regex re(".*/([^/]+)/[^/]+$");
                        std::smatch match;
                        std::string l_strModelPath = "";
                        if (std::regex_search(val, match, re) && match.size() > 1) {
                            l_strModelPath = match.str(1);
                        }

                        if (!l_strModelPath.empty()) {
                            std::regex re(l_strModelPath);
                            std::string result = std::regex_replace(val, re, new_model_path);
                            // 修改当前行
                            line = key + " " + result;
                        }
                    }
                }
            }

            // 将修改后的内容写回文件
            std::ofstream outFile(config_path);
            if (outFile.is_open()) {
                for (const auto& modified_line : lines) {
                    outFile << modified_line << std::endl;
                }
                outFile.close();
            } else {
                std::cerr << "Failed to open the file for writing." << std::endl;
            }
        }
        void ModelPathReWriteTool::UpdateAllinonePath(const std::string &config_path, const std::string &new_model_path)
        {
            std::ifstream file(config_path);
            std::string lastLine, currentLine;

            while (std::getline(file, currentLine)) {
                if (!currentLine.empty()) {
                    lastLine = currentLine;
                }
            }
            file.close();
            auto l_tmData = toml::parse(config_path);
            auto &l_tmPerceptionParameters = toml::find(l_tmData, "perception_parameters");
            auto l_tmDemo53Path = toml::find<std::string>(l_tmPerceptionParameters, "front_back_geom_xyz_sort_info_path");
            auto l_tmDemo54Path = toml::find<std::string>(l_tmPerceptionParameters, "side_geom_xyz_sort_info_path");
            auto l_tmTileInfoPath = toml::find<std::string>(l_tmPerceptionParameters, "fuse_tile_info_path");
            auto l_tmXyzPath = toml::find<std::string>(l_tmPerceptionParameters, "geom_xyz_buffer_file");
            auto l_tmOffsetPath = toml::find<std::string>(l_tmPerceptionParameters, "geom_offset_file");
            auto l_tmSe70Path = toml::find<std::string>(l_tmPerceptionParameters, "det_70_se_file");
            auto l_tmSe30Path = toml::find<std::string>(l_tmPerceptionParameters, "det_30_se_file");
            auto l_tmSe70BackPath = toml::find<std::string>(l_tmPerceptionParameters, "det_70_back_se_file");
            auto l_tmLane70Path = toml::find<std::string>(l_tmPerceptionParameters, "lane_70_se_file");
            auto l_tmLane30Path = toml::find<std::string>(l_tmPerceptionParameters, "lane_30_se_file");
            auto l_strDemo53Path =  GetModelPath(l_tmDemo53Path,new_model_path);
            auto l_strDemo54Path =  GetModelPath(l_tmDemo54Path, new_model_path);
            auto l_strTileInfoPath =  GetModelPath(l_tmTileInfoPath, new_model_path);
            auto l_strXyzPath =  GetModelPath(l_tmXyzPath, new_model_path);
            auto l_strOffsetPath =  GetModelPath(l_tmOffsetPath, new_model_path);
            auto l_strSe70Path =  GetModelPath(l_tmSe70Path, new_model_path);
            auto l_strSe30Path =  GetModelPath(l_tmSe30Path, new_model_path);
            auto l_strSe70BackPath =  GetModelPath(l_tmSe70BackPath, new_model_path);
            auto l_strLane70Path =  GetModelPath(l_tmLane70Path, new_model_path);
            auto l_strLane30Path =  GetModelPath(l_tmLane30Path, new_model_path);
            l_tmPerceptionParameters["front_back_geom_xyz_sort_info_path"] = l_strDemo53Path;
            l_tmPerceptionParameters["side_geom_xyz_sort_info_path"] = l_strDemo54Path;
            l_tmPerceptionParameters["fuse_tile_info_path"] = l_strTileInfoPath;
            l_tmPerceptionParameters["geom_xyz_buffer_file"] = l_strXyzPath;
            l_tmPerceptionParameters["geom_offset_file"] = l_strOffsetPath;
            l_tmPerceptionParameters["det_70_se_file"] = l_strSe70Path;
            l_tmPerceptionParameters["det_30_se_file"] = l_strSe30Path;
            l_tmPerceptionParameters["det_70_back_se_file"] = l_strSe70BackPath;
            l_tmPerceptionParameters["lane_70_se_file"] = l_strLane70Path;
            l_tmPerceptionParameters["lane_30_se_file"] = l_strLane30Path;
            std::ofstream l_outStream(config_path);
            l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
            l_outStream << std::endl << lastLine ;
            l_outStream.close();
            // std::cout 
            //     << "l_tmDemo53Path: " << l_tmDemo53Path << "\n"
            //     << "l_tmDemo54Path: " << l_tmDemo54Path << "\n"
            //     << "l_tmTileInfoPath: " << l_tmTileInfoPath << "\n"
            //     << "l_tmXyzPath: " << l_tmXyzPath << "\n"
            //     << "l_tmOffsetPath: " << l_tmOffsetPath << "\n"
            //     << "l_tmSe70Path: " << l_tmSe70Path << "\n"
            //     << "l_tmSe30Path: " << l_tmSe30Path << "\n"
            //     << "l_tmSe70BackPath: " << l_tmSe70BackPath << "\n"
            //     << "l_tmLane70Path: " << l_tmLane70Path << "\n"
            //     << "l_tmLane30Path: " << l_tmLane30Path << "\n";

        }
        std::string ModelPathReWriteTool::GetModelPath(std::string path,const std::string &new_model_path)
        {
            std::regex pattern("a1000/(.*)/devastator_bin");
            std::smatch match;
            std::string extracted = "";
            if (std::regex_search(path, match, pattern)) {
                if (match.size() > 1) {
                    extracted = match.str(1);
                }
            } 
            if(!extracted.empty())
            {
                std::regex re(extracted);
                std::string result = std::regex_replace(path, re, new_model_path);
                return result;
            }
            return "";
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cout<<"please input [allinone_path] [config_path] [new_model_path]!"<<std::endl;
    }
    std::string l_strAllinonePath = argv[1];
    std::string l_strConfigPath = argv[2];
    std::string l_strModelPath = argv[3];
    {
        std::shared_ptr<mach::tool::ModelPathReWriteTool> l_pModelTool = std::make_shared<mach::tool::ModelPathReWriteTool>(l_strAllinonePath);
        l_pModelTool->UpdateModelsPath(l_strConfigPath,l_strModelPath);
    }
}

