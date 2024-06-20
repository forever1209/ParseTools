#ifndef __TOOL_PARSETOOLS_GNSSPARSE_H
#define __TOOL_PARSETOOLS_GNSSPARSE_H
#include "ParseBase.h"
namespace mach
{
    namespace tool
    {
        class GnssParse : public BaseParse
        {
        public:
            GnssParse(const ParseFileType & type);
            virtual ~GnssParse();
            virtual bool Open(const std::string &file);
        private:
            virtual bool ParseData();
        };
    }
}
#endif //__TOOL_PARSETOOLS_GNSSPARSE_H