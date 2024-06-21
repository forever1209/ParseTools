#ifndef __TOOL_REWRITRTOOLS_MODELPATHREWRITE_H
#define __TOOL_REWRITRTOOLS_MODELPATHREWRITE_H
#include <vector>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <cstring>
#include <regex>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>
#include <stdlib.h>
#include<iostream>
namespace mach
{
    namespace tool
    {
        class ModelPathReWriteTool
        {
        public:
            ModelPathReWriteTool(const std::string & allinone_path);
            ~ModelPathReWriteTool();
        public:
            void UpdateModelsPath(const std::string & config_path ,const std::string & new_model_path);
        private:
            void InitTxtPathVec();
            void ParseFileUpdate(const std::string & config_path ,const std::string & new_model_path);
            void UpdateAllinonePath(const std::string & config_path ,const std::string & new_model_path);
            std::string GetModelPath(std::string path,const std::string &new_model_path);
        private:
            std::string m_strAllinonePath_="";
            std::vector<std::string> m_vecConfigPath_;
        };
    }
}
#endif //__TOOL_REWRITRTOOLS_MODELPATHREWRITE_H