#include <iostream>
#include <getopt.h>
#include <ParseFile.h>
#include <TomlReWrite.h>
#include <YamlReWrite.h>
#include "CameraPreprocess.h"
#include "GeomTool.h"
#include <pybind11/pybind11.h>
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pyvectorize.hpp>
#include <numeric>
#include <cmath>
const std::string GetExeRunPath()
{
    char *p = NULL;
    const int len = 1024;
    /// to keep the absolute path of executable's path
    char l_pExecutionPath[len];
    memset(l_pExecutionPath, 0, len);
    int n = readlink("/proc/self/exe", l_pExecutionPath, len);
    if (NULL != (p = strrchr(l_pExecutionPath,'/')))
    {
        *p = '\0';
    }
    /******************************************/
    return  l_pExecutionPath;

}
void PrintUsage()
{
    std::cout << "usage: ParseTool [-h] [-s] [-b] [-d] [-i] [-c] [-m]" << std::endl;
    std::cout << "  optional arguments:" << std::endl;
    std::cout << "      -h show this help message and exit " << std::endl;
    std::cout << "      -s Set Src Calibresult Path " << std::endl;
    std::cout << "      -b Set Base Path " << std::endl;
    std::cout << "      -d Set Config Dst Path " << std::endl;
    std::cout << "      -i Set Vehicle Id " << std::endl;
    std::cout << "      -c Camera Preprocess " << std::endl;
    std::cout << "      -t Output Some Test Info " << std::endl;
    std::cout << "      -m Conflate Models " << std::endl;
    std::cout << "  Examples:" << std::endl;
    std::cout << "    Update Config" << std::endl;
    std::cout << "      Method1 : ParseTool -b /home/liulei10/LL/ParseJson/Test/ -i car602" << std::endl;
    std::cout << "      Method2 : ParseTool -s /home/liulei10/LL/ParseJson/calibresult/car_00602 -d /home/liulei10/LL/ParseJson/Test/config/car602 " << std::endl;
    std::cout << "      Method3 : ParseTool -d /home/liulei10/LL/ParseJson/Test/config/car602  -i car602" << std::endl;
    std::cout << "    Camera Preprocess" << std::endl;
    std::cout << "      Method  : ParseTool -c -i car602 " << std::endl;
    std::cout << "    Conflate Models " << std::endl;
    std::cout << "      Method  : ParseTool -m BYD  [BYD/HS] -i car602 -d /home/liulei10/LL/ParseJson/Test/deva-dev-config-car602/config/car602" << std::endl;
}
const std::string SearchFolder(const std::string & root,const std::string & target , const std::string &vehicle_id)
{
    DIR* dir = opendir(root.c_str());
    if (dir != nullptr) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_DIR && std::string(entry->d_name) == target) {
                std::string l_strPath = root + "/" + entry->d_name + "/" + vehicle_id;
                return l_strPath;
            }
        }
    }
    return "";
}
const bool CheckParams(mach::tool::RunParam &params , const std::string & run_path = "")
{
    std::unordered_map<std::string,std::string> l_mapVehicleId = {
        {"car201" , "car_byd_han_00001"} ,
        {"car202" , "car_byd_han_00002"} ,
        {"car602" , "car_00602"}
    };
    if(params.run_camera_preprocess_)
    {
        return true;
    }
    if(params.run_conflate_models_)
    {
        if(params.dst_config_path_.empty())
        {
            std::cout<<"dst config path not set"<<std::endl;
            return false;
        }
        return true;
    }
    if(params.src_calib_path_.empty() && !params.car_id_.empty())
    {
        if(l_mapVehicleId.count(params.car_id_)>0)
        {
            params.src_calib_path_ = run_path + "calibresult/" + l_mapVehicleId[params.car_id_];
        }
    }
    if(!params.src_calib_path_.empty() && !params.dst_config_path_.empty())
    {
        return true;
    }
    if(!params.base_path_.empty() && !params.car_id_.empty())
    {
        std::cout << "Check Whether Config Path In Root Path " << std::endl;
        auto l_strConfigPath = SearchFolder(params.base_path_, "config" , params.car_id_);
        if(l_strConfigPath.empty())
        {
            return false;
        }
        params.dst_config_path_ = l_strConfigPath;
        if(l_mapVehicleId.count(params.car_id_)>0)
        {
            params.src_calib_path_ = run_path + "calibresult/" + l_mapVehicleId[params.car_id_];
        }
        return true;
    }
    return false;
}
void CheckAndPullChildrenPath(const std::string & path)
{
    std::string l_strFilePath = path + "calibresult";
    bool bIsExist = (0 == access(l_strFilePath.c_str(), F_OK));
    if(bIsExist)
    {
        std::string l_strCmd = "cd " + l_strFilePath + " && git pull";
        system(l_strCmd.c_str());
    }
    else
    {
        std::string l_strCmd = "cd " + path + " && git clone git@git-core.megvii-inc.com:transformer/calibresult.git";
        system(l_strCmd.c_str());
    }
}
void ConflateModels(const mach::tool::RunParam &params,const std::string &pkl_path)
// void ConflateModels(const std::string & type,const std::string & car_id ,const std::string &pkl_path)
{
    std::string type = params.config_type_ ;
    std::string car_id = params.car_id_;
    std::string config_path = params.dst_config_path_;
    std::shared_ptr<mach::tool::GeomTool> m_pGeomTool = std::make_shared<mach::tool::GeomTool>();
    mach::tool::GeomConfigType l_enType = mach::tool::GeomConfigType::DEFAULT_CONFIG;
    if(type=="BYD")
    {
        l_enType = mach::tool::GeomConfigType::GEOM_BYD_CONFIG;
    }
    if(type=="HS")
    {
        l_enType = mach::tool::GeomConfigType::GEOM_HS_CONFIG;
    }  
    bool l_bOk = m_pGeomTool->InitTool(l_enType,GetExeRunPath());
    std::cout<<"InitTool is "<<l_bOk<<std::endl;
    m_pGeomTool->ConflateData(pkl_path,car_id,config_path);
}
int main(int argc, char* argv[])
{
    if(argc<=1)
    {
        std::cout << "Please Input Param ." << std::endl;
        PrintUsage();
        return 0;
    }
    namespace py = pybind11;
    py::scoped_interpreter guard{};
    xt::import_numpy();
    auto l_strRunPath = GetExeRunPath();
    std::string l_strCalibPath = l_strRunPath + "/ChildrenPath/";
    CheckAndPullChildrenPath(l_strCalibPath);
    mach::tool::RunParam params;
    int option;
    while ((option = getopt_long(argc, argv, "hs:b:d:i:ctm:", nullptr, nullptr)) != -1) {
        switch (option) {
            case 'h':
                PrintUsage();
                return 0;
            case 's':
                std::cout << "Set calibresult Src Path: " << optarg  << std::endl;
                params.src_calib_path_ = optarg;
                break;
            case 'b':
                std::cout << "Set Base Path: " << optarg << std::endl;
                params.base_path_ = optarg;
                break;
            case 'd':
                std::cout << "Set Config Dst Path: " << optarg << std::endl;
                params.dst_config_path_ = optarg;
                break;
            case 'c':
                params.run_camera_preprocess_ = true;
                // std::cout << "" << std::endl;
                break;
            case 'm':
                params.run_conflate_models_ = true;
                params.config_type_ = optarg;
                // ConflateModels(optarg);
                break;
            case 't':
                params.use_test_ = true;
                break;
            case 'i':
                std::cout << "Set Vehicle Id: " << optarg << std::endl;
                params.car_id_ = optarg;
                break;
            default:
                std::cout << "Unknown option" << std::endl;
                PrintUsage();
                return 0;
        }
    }
    bool l_bOk = CheckParams(params , l_strCalibPath);
    if(l_bOk)
    {
        if(params.run_camera_preprocess_)
        {
            std::shared_ptr<mach::tool::CameraPreprocess> m_pPreprocess = std::make_shared<mach::tool::CameraPreprocess>();
            auto begin = std::chrono::steady_clock::now();
            bool l_bOk = m_pPreprocess->CheckCalibInfoExist(l_strCalibPath,params.car_id_);
            auto end = std::chrono::steady_clock::now();
            std::cout<<"CheckCalibInfoExist need:"<<std::chrono::duration<double, std::milli>(end - begin).count()<<std::endl;
            if(!l_bOk)
            {
                std::cout << "CheckCalibInfoExist Error , Please Check Whether Input VehicleId" <<std::endl;
            }
            return 0;
        }
        if(params.run_conflate_models_)
        {
            std::string l_strPklPath = l_strRunPath + "/input/fovs_intrinsic.pkl";
            ConflateModels(params , l_strPklPath);
            return 0;
        }
        std::shared_ptr<mach::tool::FileSysParse> m_pFileParser = std::make_shared<mach::tool::FileSysParse>();
        std::shared_ptr<mach::tool::TomlReWriteTool> m_pTomlTool = std::make_shared<mach::tool::TomlReWriteTool>();
        std::shared_ptr<mach::tool::YamlReWriteTool> m_pYamlTool = std::make_shared<mach::tool::YamlReWriteTool>();
        bool l_bOk = m_pFileParser->CheckConfigDirExist(params.dst_config_path_);
        if(!l_bOk)
        {
            std::cout << "Config "<<params.dst_config_path_ << "not Exist" << std::endl;
            return 0;
        }
        auto l_mapConfigFiles = m_pFileParser->GetConfigTypeFiles();
        l_bOk &= m_pFileParser->CheckParamDirExist(params.src_calib_path_);
        if(!l_bOk)
        {
            std::cout << "calibresult "<<params.src_calib_path_ << "not Exist" << std::endl;
            return 0;
        }
        m_pFileParser->ParseFileNamesByType();
        auto l_mapParamsFiles = m_pFileParser->GetParamTypeFiles();
        m_pTomlTool->SetParamInfoMap(l_mapParamsFiles);
        m_pYamlTool->SetParamInfoMap(l_mapParamsFiles);
        for(const auto & config:l_mapConfigFiles)
        {
            if(config.first=="toml")
            {
                for(const auto &file:config.second)
                {
                    m_pTomlTool->CheckCalibInfoExist(file,m_pFileParser->GetFileName(file));
                }
            }
            if(config.first=="yaml")
            {
                std::string l_strConfig = "pilot_perception_map.yaml";
                for(const auto &file:config.second)
                {
                    if(file.find(l_strConfig) != std::string::npos)
                    {
                        m_pYamlTool->CheckYamlFile(file);
                    }
                }
            }
        }
    }
    else
    {
        std::cout << "Check Input Path Failed , Please Input right Path !" << std::endl;
    }
    return 0 ;
}