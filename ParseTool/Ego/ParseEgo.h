#ifndef __TOOL_PARSETOOLS_EGOPARSE_H
#define __TOOL_PARSETOOLS_EGOPARSE_H
#include "ParseBase.h"
namespace mach
{
    namespace tool
    {
        class EgoParse : public BaseParse
        {
        public:
            EgoParse(const ParseFileType & type);
            virtual ~EgoParse();
            virtual bool Open(const std::string &file);
        private:
            virtual bool ParseData();
        };
    }
}
#endif // __TOOL_PARSETOOLS_EGOPARSE_H