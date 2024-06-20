#ifndef __TOOL_PARSETOOLS_FILEPARSE_H
#define __TOOL_PARSETOOLS_FILEPARSE_H
#include <iostream>
#include <vector>
#include <unordered_map>
namespace mach
{
    namespace tool
    {
        class FileSysParse 
        {
        public:
            FileSysParse();
            ~FileSysParse();
            // CalibResult Param Parse
            bool CheckParamDirExist(const std::string & dir);
            void ParseFileNamesByType(const std::unordered_map<std::string ,std::string> & path_list_map = {});

            // Config Param File Parse
            bool CheckConfigDirExist(const std::string &dir);
            const std::unordered_map<std::string,std::unordered_map<std::string,std::vector<std::string>>> GetParamTypeFiles()const;
            const std::unordered_map<std::string,std::vector<std::string>> GetConfigTypeFiles()const;
            const std::string GetFileName(const std::string &path);
        private:
            std::string m_strBaseDir ;
            std::vector<std::string> m_vecParamsPath_;
            // type path files
            std::unordered_map<std::string,std::unordered_map<std::string,std::vector<std::string>>> m_mapTypeFile_;
            //config base dir
            std::string m_strBaseConfigDir ;
            // config type file_path
            std::unordered_map<std::string,std::vector<std::string>> m_mapTypeConfig_;
        };
    } // namespace tool
    
} // namespace mach


#endif //__TOOL_PARSETOOLS_FILEPARSE_H