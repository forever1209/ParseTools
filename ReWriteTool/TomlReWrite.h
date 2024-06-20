#ifndef __TOOL_REWRITRTOOLS_TOMLREWRITE_H
#define __TOOL_REWRITRTOOLS_TOMLREWRITE_H
#include "PubParseDef.h"
#include <unordered_map>
#include <toml.hpp>
namespace mach
{
    namespace tool
    {
        class TomlReWriteTool
        {
        public:
            TomlReWriteTool();
            ~TomlReWriteTool();
            void CheckCalibInfoExist(const std::string & file,const std::string &name = "");
            void SetParamInfoMap(const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>> &map);
        private:
            void CompareAllInOne(const std::string &path);
            void CompareLidar(const std::string &path);
            void CompareTracking(const std::string &path);
            void CompareObstaclePostfusion(const std::string &path);
            void CompareSaticObstacle(const std::string &path);
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
    } // namespace tool
    
} // namespace mach


#endif //__TOOL_REWRITRTOOLS_TOMLREWRITE_H