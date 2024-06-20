#include <ctype.h>
#include <stdint.h>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <vector>
#include "libnpy.hpp"
#include "assert.h"
#include "malloc.h"
#include <pthread.h>
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>
using namespace std;

struct OPoint_S16 {
    int16_t b;   // batch
    int16_t px;  // point coord x
    int16_t py;  // point coord y
    int16_t pz;  // point coord z
    int16_t n;   // camera index
    int16_t d;   // depth index
    int16_t mx;  // image coord x
    int16_t my;  // image coord y
};

struct TileInfo {
    int32_t depth;
    int32_t tile_flag; // 0: head tile; 1: tail tile; 2: common tile
    int32_t bev_offset;
    int32_t bev_range;
    int32_t depth_offset;
    int32_t depth_range;
    int32_t image_offset;
    int32_t image_range;
    int32_t geom_offset;
    int32_t geom_range;
};

struct initParams{
    int finally_tile_num;
    int l2_tile_num[4];
    int fuse_total_image_feat_sz;
    int fuse_total_depth_feat_sz;
    int fuse_total_valid_points;
    int fuse_total_tile_info_range;
    int total_valid_points_front_back;
    int total_valid_points_left_right;
};

const int max_depth_buf_range = 20 * 1024;
const int max_image_buf_range = 15 * 1024;
const int max_bev_buf_range = 16384;
const int max_geom_buf_range = 20 * 1024; // 1280 * sizeof(OPoint_S16);
const int scaling_param = 8;
const int dsp_core_num = 4;

// void GenObstacle(const std::string &base_path);