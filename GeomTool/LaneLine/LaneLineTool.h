#include <iostream>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <xtensor.hpp>
#include "xtensor/xnpy.hpp"
#include <functional>
#include <list>
#include <ostream>
#include <map>
// Define custom hash function for VoxelKey
using VoxelKey = std::tuple<int, int, int, int>;
using VoxelValue = std::tuple<int, int, int, int, int>;
using VoxelMapping = std::map<VoxelKey, std::vector<VoxelValue>>;
// Helper function to print a tuple
template<std::size_t... Is, typename Tuple>
void print_tuple(std::ostream& os, const Tuple& t, std::index_sequence<Is...>) {
    ((os << (Is == 0 ? "" : ", ") << std::get<Is>(t)), ...);
}

template<typename... Args>
std::ostream& operator<<(std::ostream& os, const std::tuple<Args...>& t) {
    os << "(";
    print_tuple(os, t, std::index_sequence_for<Args...>{});
    os << ")";
    return os;
}

// Helper function to print a container (vector, list, etc.)
template<typename Container>
std::ostream& print_container(std::ostream& os, const Container& container) {
    os << "[";
    for (auto it = container.begin(); it != container.end(); ++it) {
        if (it != container.begin()) {
            os << ", ";
        }
        os << *it;
    }
    os << "]";
    return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
    return print_container(os, v);
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::list<T>& l) {
    return print_container(os, l);
}

namespace mach
{
    namespace tool
    {
        class LaneLineTool
        {
        public:
            LaneLineTool(const std::string & base_path);
            ~LaneLineTool();

        public:
            void GenVoxelMapping(const std::string &geom_path , const std::string &voxel_path);
        private:
            void ParseGeomS8(VoxelMapping voxel_mapping, const std::string& output_path);
            std::string GetType(const std::string& filename);
            void GenBin();
        private:
            std::vector<VoxelKey> m_vecKeys;
            std::string m_strBasePath_;
        };
    } // namespace tool
    
} // namespace mach
