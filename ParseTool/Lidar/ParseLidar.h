#ifndef __TOOL_PARSETOOLS_LIDARPARSE_H
#define __TOOL_PARSETOOLS_LIDARPARSE_H
#include "ParseBase.h"
namespace mach
{
    namespace tool
    {
        class LidarParse : public BaseParse
        {
        public:
            LidarParse(const ParseFileType & type);
            virtual ~LidarParse();
            virtual bool Open(const std::string &file);
        private:
            virtual bool ParseData();
        };
    }
}
#endif //__TOOL_PARSETOOLS_LIDARPARSE_H