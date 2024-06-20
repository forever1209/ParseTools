#include "ObstacleTool.h"
namespace mach
{
    namespace tool
    {
        ObstacleTool::ObstacleTool(const std::string & base_path)
        {
            m_strBasePath_ = base_path;
        }
        ObstacleTool::~ObstacleTool()
        {
            std::cout<<"ObstacleTool release "<<std::endl;
        }
        void ObstacleTool::GenVoxelMapping(const std::string &fb_geom_path , const std::string &lr_geom_path , const std::string &voxel_path)
        {
            m_vecFbKeys.clear();
            m_vecLrKeys.clear();
            try
            {
                xt::xarray<int64_t> fb_geom;
                if(GetType(fb_geom_path)=="'<i8'")
                {
                    fb_geom = xt::load_npy<int64_t>(fb_geom_path);
                    // std::cout<<fb_geom_path<<" i8 "<<std::endl;
                }
                if(GetType(fb_geom_path)=="'<i4'")
                {
                    fb_geom = xt::load_npy<int32_t>(fb_geom_path);
                    // std::cout<<fb_geom_path<<" i4 "<<std::endl;
                }
                xt::xarray<int64_t> lr_geom ;
                if(GetType(lr_geom_path)=="'<i8'")
                {
                    lr_geom = xt::load_npy<int64_t>(lr_geom_path);
                    // std::cout<<lr_geom_path<<" i8 "<<std::endl;
                }
                if(GetType(lr_geom_path)=="'<i4'")
                {
                    lr_geom = xt::load_npy<int32_t>(lr_geom_path);
                    // std::cout<<lr_geom_path<<" i4 "<<std::endl;
                }
                xt::xarray<int64_t> nx ;
                if(GetType(voxel_path)=="'<i8'")
                {
                    nx = xt::load_npy<int64_t>(voxel_path);
                    // std::cout<<voxel_path<<" i8 "<<std::endl;
                }
                if(GetType(voxel_path)=="'<i4'")
                {
                    nx = xt::load_npy<int32_t>(voxel_path);
                    // std::cout<<voxel_path<<" i4 "<<std::endl;
                }

                {
                    VoxelMapping voxel_mapping; 
                    auto B = fb_geom.shape()[0];
                    auto N = fb_geom.shape()[1];
                    auto D = fb_geom.shape()[2];
                    auto H = fb_geom.shape()[3];
                    auto W = fb_geom.shape()[4];
                    for (size_t b = 0; b < B; ++b) {
                        for (size_t n = 0; n < N; ++n) {
                            for (size_t d = 0; d < D; ++d) {
                                for (size_t h = 0; h < H; ++h) {
                                    for (size_t w = 0; w < W; ++w) {
                                        int64_t x = fb_geom(b, n, d, h, w, 0);
                                        int64_t y = fb_geom(b, n, d, h, w, 1);
                                        int64_t z = fb_geom(b, n, d, h, w, 2);
                                        if (x >= 0 && x < nx(0) && y >= 0 && y < nx(1) && z >= 0 && z < nx(2)) {
                                            VoxelKey key = std::make_tuple(b, static_cast<int>(z), static_cast<int>(y), static_cast<int>(x));
                                            VoxelValue value = {std::make_tuple(b, n, d, h, w)};
                                            if(voxel_mapping.count(key)==0)
                                            {
                                                m_vecFbKeys.emplace_back(key);
                                            }
                                            // stream<<"key : "<<key <<" value : "<<value<<std::endl;
                                            voxel_mapping[key].emplace_back(value);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    //use voxel_mapping
                    ParseGeomS8(voxel_mapping,m_strBasePath_ + "/output/fb_valid.npy",m_vecFbKeys);
                }

                {
                    VoxelMapping voxel_mapping; 
                    auto B = lr_geom.shape()[0];
                    auto N = lr_geom.shape()[1];
                    auto D = lr_geom.shape()[2];
                    auto H = lr_geom.shape()[3];
                    auto W = lr_geom.shape()[4];
                    for (size_t b = 0; b < B; ++b) {
                        for (size_t n = 0; n < N; ++n) {
                            for (size_t d = 0; d < D; ++d) {
                                for (size_t h = 0; h < H; ++h) {
                                    for (size_t w = 0; w < W; ++w) {
                                        int64_t x = lr_geom(b, n, d, h, w, 0);
                                        int64_t y = lr_geom(b, n, d, h, w, 1);
                                        int64_t z = lr_geom(b, n, d, h, w, 2);
                                        if (x >= 0 && x < nx(0) && y >= 0 && y < nx(1) && z >= 0 && z < nx(2)) {
                                            VoxelKey key = std::make_tuple(b, static_cast<int>(z), static_cast<int>(y), static_cast<int>(x));
                                            VoxelValue value = {std::make_tuple(b, n, d, h, w)};
                                            if(voxel_mapping.count(key)==0)
                                            {
                                                m_vecLrKeys.emplace_back(key);
                                            }
                                            // stream<<"key : "<<key <<" value : "<<value<<std::endl;
                                            voxel_mapping[key].emplace_back(value);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // use voxel_mapping
                    ParseGeomS8(voxel_mapping,m_strBasePath_ + "/output/lr_valid.npy",m_vecLrKeys);
                }
                // GenBin();
            }
            catch(std::runtime_error& e)
            {
                std::cout<<"UpdateNpy runtime_error "<<": " << e.what() << std::endl;
            }
            catch (std::exception)
            {
                std::cout<<"UpdateNpy exception "<<std::endl;
            }
            catch(const std::exception& e)
            {
                std::cout<<"UpdateNpy exception "<<e.what()<<std::endl;
            }

        }
        void ObstacleTool::ParseGeomS8(VoxelMapping voxel_mapping, const std::string &output_path,const std::vector<VoxelKey> &keys)
        {
            int valid_points2 = 0;
            for (const auto& item : voxel_mapping) {
                valid_points2 += item.second.size();
            }
            xt::xarray<int16_t> geom_opint2 = xt::ones<int16_t>({valid_points2, 8});
            size_t nidx = 0;
            for(const VoxelKey &key : keys)
            {
                int b_voxel, z, y, x;
                std::tie(b_voxel, z, y, x) = key;
                VoxelKey map_key = key;
                if(voxel_mapping.count(map_key)>0)
                {
                    auto values = voxel_mapping[map_key];
                    for(const auto &value : values)
                    {
                        int b, n, d, h, w;
                        std::tie(b, n, d, h, w) = value;
                        assert(b == b_voxel);
                        geom_opint2(nidx, 0) = b;
                        geom_opint2(nidx, 1) = x;
                        geom_opint2(nidx, 2) = y;
                        geom_opint2(nidx, 3) = z;
                        geom_opint2(nidx, 4) = n;
                        geom_opint2(nidx, 5) = d;
                        geom_opint2(nidx, 6) = w;
                        geom_opint2(nidx, 7) = h;
                        ++nidx;
                    }
                }
                
            }
            xt::dump_npy(output_path, geom_opint2);
        }
        std::string ObstacleTool::GetType(const std::string &filename)
        {
            std::ifstream file(filename, std::ios::binary);
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open file: " + filename);
            }

            // Read the magic string
            char magic[6];
            file.read(magic, 6);
            if (std::string(magic, 6) != "\x93NUMPY") {
                throw std::runtime_error("Invalid .npy file");
            }

            // Read version number
            uint8_t major_version = 0;
            uint8_t minor_version = 0;
            file.read(reinterpret_cast<char*>(&major_version), sizeof(major_version));
            file.read(reinterpret_cast<char*>(&minor_version), sizeof(minor_version));

            // Read header length
            uint16_t header_len = 0;
            if (major_version == 1 && minor_version == 0) {
                file.read(reinterpret_cast<char*>(&header_len), sizeof(header_len));
            } else {
                throw std::runtime_error("Unsupported numpy file version.");
            }

            // Read the header
            std::vector<char> header(header_len);
            file.read(header.data(), header_len);

            // Parse the header
            std::string header_str(header.begin(), header.end());
            std::unordered_map<std::string, std::string> header_map;

            // Remove the enclosing braces and split by commas
            header_str = header_str.substr(1, header_str.size() - 3);
            size_t pos = 0;
            while ((pos = header_str.find(", ")) != std::string::npos) {
                std::string token = header_str.substr(0, pos);
                size_t colon_pos = token.find(": ");
                std::string key = token.substr(1, colon_pos - 2);
                std::string value = token.substr(colon_pos + 2);
                header_map[key] = value;
                header_str.erase(0, pos + 2);
            }
            // Add the last pair
            size_t colon_pos = header_str.find(": ");
            std::string key = header_str.substr(1, colon_pos - 2);
            std::string value = header_str.substr(colon_pos + 2);
            header_map[key] = value;

            return header_map["descr"];
        }
        void ObstacleTool::GenBin(const std::string & config_path)
        {
            std::cout<<"gen bin begin "<<std::endl;
            // GenObstacle(m_strBasePath_);
            std::string l_strCmd = m_strBasePath_ + "/ObstacleTool " + m_strBasePath_ + " " + config_path + " \n";
            system(l_strCmd.c_str());
        }
    }
}
