#ifndef __TOOL_PARSETOOLS_CAMERAPARSE_H
#define __TOOL_PARSETOOLS_CAMERAPARSE_H
#include "ParseBase.h"
namespace mach
{
    namespace tool
    {
        class CameraParse : public BaseParse
        {
        public:
            CameraParse(const ParseFileType & type);
            virtual ~CameraParse();
            virtual bool Open(const std::string &file);
        private:
            virtual bool ParseData();
        };
    }
}
#endif //__TOOL_PARSETOOLS_CAMERAPARSE_H