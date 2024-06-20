#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <vector>
#include "LaneLine/libnpy.hpp"
#include "assert.h"
#include "malloc.h"
#include <fstream>
#include <stdint.h>
#include <sys/syscall.h>
#include <unistd.h>
#include "LaneLine/gen.h"
//提示：修改这2个参数来控制逻辑 ”数据类型“ 和 ”排序方式“
#define TYPE_FLAG 1  // FP32(0) S8(1)
#define SORT_FLAG 0  // sort_yxz_nhw(0)  sort_nhw_xyz(1)

//----------------------------
#define DTYPE int8_t
#define OUT_TMP_ELEM_SIZE sizeof(int)

struct OPoint {
    int16_t b;   // batch
    int16_t px;  // point coord x
    int16_t py;  // point coord y
    int16_t pz;  // point coord z
    int16_t n;   // camera index
    int16_t d;   // depth index
    int16_t mx;  // image coord x
    int16_t my;  // image coord y
};

void test_lvt_lane_line(const std::string &base_path) {
    int inf_block = 1; // demo52 车道线最新的fuse版本实现不用分块，分块需增加额外负担
    // step0. preprocess inputs/output shape
    int batch = 1;
    int num_cams = 2;
    int depth = 96;
    int IH = 32;
    int IW = 88;
    int CH = 80;
    int OW = 100;
    int OZ = 10;

    int num_points = num_cams * depth * IH * IW;
    int16_t* geom_xyz = static_cast<int16_t*>(memalign(64, sizeof(int16_t) * batch * num_points * 8));
    OPoint* geom_xyz_sort = static_cast<OPoint*>(memalign(64, sizeof(OPoint) * batch * num_points));
    int32_t* geom_offset = static_cast<int32_t*>(memalign(64, sizeof(int32_t) * batch * num_points * 3));

    //提示：用户应该把这部分逻辑在离线进行处理，集成时候需要把sort_n/per_block_num_points提前作为参数设置好
    int per_block_num_points = 0;

    size_t ori_valid_points = npy::LoadArrayFromNumpy(base_path + "/output/geom_xyz_valid.npy", geom_xyz);

    // 模拟离线操作：重组geom_xyz为geom_xyz_sort
    ori_valid_points /= 8;
    size_t total_valid_points = 0;
    for (size_t n = 0; n < ori_valid_points; n++) {
        // b, x, y, z, n, d, w, h
        auto t = *reinterpret_cast<OPoint*>(geom_xyz + n * 8);
        geom_xyz_sort[total_valid_points++] = t;
    }
    assert(total_valid_points == ori_valid_points);
    printf("valid points number: %zu, ratio: %f\n", total_valid_points, 1.0f * total_valid_points / num_points);

    //模拟离线操作：对geom_xyz进行排序，优先按照y,x,z排序，然后是n,h,w,d
    std::sort(geom_xyz_sort, geom_xyz_sort + total_valid_points,
                [&](OPoint a, OPoint b) -> bool {
                    if ((((int64_t)a.py << 45) + ((int64_t)a.px << 30) + ((int64_t)a.pz << 15)) <
                        (((int64_t)b.py << 45) + ((int64_t)b.px << 30) + ((int64_t)b.pz << 15))) {
                        return true;
                    }
                    return false;
                });
    // demo52 车道线在geom_xyz进行y,x,z排序后不再分块和二次排序
    for (size_t p_idx = 0; p_idx < total_valid_points; ++p_idx) {
        OPoint point = *(geom_xyz_sort + p_idx);
        int depth_offset = point.n * IH * IW * depth + point.my * IW * depth + point.mx * depth + point.d;
        int image_offset = point.n * IH * IW * CH + point.my * IW * CH + point.mx * CH;
        int bev_offset = point.py * OW * OZ * CH + point.px * OZ * CH + point.pz * CH;
        geom_offset[p_idx * 3] = depth_offset;
        geom_offset[p_idx * 3 + 1] = image_offset;
        geom_offset[p_idx * 3 + 2] = bev_offset;
    }

    // std::ofstream outfile1("lanelineLSS.bin", std::ios::binary);
    // if (outfile1.is_open()) {
    //     int tmp;
    //     tmp = total_valid_points;
    //     outfile1.write(reinterpret_cast<const char*>(&tmp), sizeof(int));
    //     tmp = batch * num_points * 3;
    //     outfile1.write(reinterpret_cast<const char*>(&tmp), sizeof(int));
    //     outfile1.write(reinterpret_cast<const char*>(geom_xyz_sort), sizeof(OPoint) * batch * total_valid_points);
    //     outfile1.write(reinterpret_cast<const char*>(geom_offset), sizeof(int32_t) * batch * num_points * 3);
    //     outfile1.close();
    //     std::cout << "Array saved to lanelineLSS.bin" << std::endl;
    // } else {
    //     std::cerr << "Unable to open file!" << std::endl;
    // }

    std::ofstream outfile2(base_path + "/output/lane_bin/geom_offset.bin", std::ios::binary);
    if (outfile2.is_open()) {
        outfile2.write(reinterpret_cast<const char*>(geom_offset), sizeof(int32_t) * batch * num_points * 3);
        outfile2.close();
        std::cout << "Array saved to geom_offset.bin" << std::endl;
    } else {
        std::cerr << "Unable to open file!" << std::endl;
    }

    std::ofstream outfile3(base_path + "/output/lane_bin/geom_xyz.bin", std::ios::binary);
    if (outfile3.is_open()) {
        outfile3.write(reinterpret_cast<const char*>(geom_xyz_sort), sizeof(OPoint) * batch * total_valid_points);
        outfile3.close();
        std::cout << "Array saved to geom_xyz.bin" << std::endl;
    } else {
        std::cerr << "Unable to open file!" << std::endl;
    }
    free(geom_xyz);
    free(geom_xyz_sort);
}

// int main() {
//     test_lvt_lane_line();
//     return 0;
// }
