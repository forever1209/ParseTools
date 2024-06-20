#include <iostream>
#include <fstream>
#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xmath.hpp>
#include <cmath>
#include <sys/stat.h>
#include <unistd.h>
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
// 量化函数
void quant(const xt::xarray<float>& tensor, const std::string& name) {
    // 将张量展平为一维数组
    auto flattened_tensor = xt::flatten(tensor);
    // 定义缩放因子
    float scale = 1.0 / 128.0;
    // 使用缩放因子进行量化
    auto quantized_tensor = xt::clip(xt::floor(flattened_tensor / scale + 0.5f), -128.0f, 127.0f);
    // 转换为 int8 类型
    xt::xarray<int8_t> int8_tensor = xt::cast<int8_t>(quantized_tensor);
    // 打印数据类型和形状
    // std::cout << name << " dtype: " << typeid(int8_tensor(0)).name() << ", shape: " << int8_tensor.shape()[0] << std::endl;
    // std::cout << int8_tensor << std::endl;
    // 保存到文件
    std::ofstream out_file(name, std::ios::binary);
    out_file.write(reinterpret_cast<const char*>(int8_tensor.data()), int8_tensor.size() * sizeof(int8_t));
    out_file.close();
}

int main(int argc, char* argv[]) {
    if (argc != 2)
    {
        std::cout<<"please input se npy path !"<<std::endl;
    }
    std::string l_strOutPath = GetExeRunPath() + "/output/se_bin/";
    bool bIsExist = (0 == access(l_strOutPath.c_str(), F_OK));
    if(!bIsExist)
    {
        bIsExist = mkdir(l_strOutPath.c_str(), 0777) == 0 ? true:false;
        if(!bIsExist)
        {
            std::cout<<"se out path not found "<<std::endl;
            return -1;
        }
    }
    std::string l_strSeConfigPath = argv[1];
    // <det_front_30_se_byd.npy> <det_back_se_byd.npy> <det_front_se_byd.npy> <lane_front_30_se_byd.npy> <lane_front_se_byd.npy>" 
    std::string det_30_se_path  = l_strSeConfigPath + "/det_front_30_se_byd.npy";
    std::string det_70_back_se_path  = l_strSeConfigPath + "/det_back_se_byd.npy";
    std::string det_70_se_path       = l_strSeConfigPath + "/det_front_se_byd.npy";
    std::string lane_30_se_path      = l_strSeConfigPath + "/lane_front_30_se_byd.npy";
    std::string lane_70_se_path      = l_strSeConfigPath + "/lane_front_se_byd.npy";
    xt::xarray<float> det_30_se = xt::load_npy<float>(det_30_se_path);
    xt::xarray<float> det_70_back_se = xt::load_npy<float>(det_70_back_se_path);
    xt::xarray<float> det_70_se = xt::load_npy<float>(det_70_se_path);
    xt::xarray<float> lane_30_se = xt::load_npy<float>(lane_30_se_path);
    xt::xarray<float> lane_70_se = xt::load_npy<float>(lane_70_se_path);
    quant(det_30_se,      l_strOutPath + "det_30_se.bin");
    quant(det_70_back_se, l_strOutPath + "det_70_back_se.bin");
    quant(det_70_se,      l_strOutPath + "det_70_se.bin");
    quant(lane_30_se,     l_strOutPath + "lane_30_se.bin");
    quant(lane_70_se,     l_strOutPath + "lane_70_se.bin");
}