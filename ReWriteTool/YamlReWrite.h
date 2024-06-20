#ifndef __TOOL_REWRITRTOOLS_YAMLREWRITE_H
#define __TOOL_REWRITRTOOLS_YAMLREWRITE_H
#include "PubParseDef.h"
#include <unordered_map>
#include <toml.hpp>
#include <iostream>
namespace mach
{
    namespace tool
    {
        class YamlReWriteTool
        {
        public:
            YamlReWriteTool();
            ~YamlReWriteTool();
        public:
            void CheckYamlFile(const std::string & filepath);
            void SetParamInfoMap(const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> &map);
        private:
            std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> m_mapParamsData_;
            template<typename T>
            bool CompareData(const T & src ,const T & dst)
            {
                return (src==dst);
            }
            template<typename T>
            bool CompareDoubleData(const T & src ,const T & dst)
            {
                double epsilon = 1e-9;
                return std::abs(src - dst) < epsilon; 
            }
        };
    }
}
#endif //__TOOL_REWRITRTOOLS_YAMLREWRITE_H