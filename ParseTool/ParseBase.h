#ifndef __TOOL_PARSETOOLS_BASEPARSE_H
#define __TOOL_PARSETOOLS_BASEPARSE_H
#include "PubParseDef.h"
#include <json/json.h>
#include <iostream>
#include <fstream>
#include <sstream>
namespace mach
{
    namespace tool
    {
        class BaseParse
        {
        public:
            BaseParse() = delete;
            explicit BaseParse(const ParseFileType & type){m_emType_ = type; m_data_.type_ = type;}
            virtual ~BaseParse() {;}
            virtual bool Open(const std::string &file) = 0;
            virtual const ParamsData GetParams()const {return m_data_;}
        protected:
            virtual bool ParseData() = 0;
        protected:
            ParseFileType m_emType_ = DEFAULT;
            std::string m_strFileUrl = "";
            ParamsData m_data_;
        };
    }
}

#endif //__TOOL_PARSETOOLS_BASEPARSE_H