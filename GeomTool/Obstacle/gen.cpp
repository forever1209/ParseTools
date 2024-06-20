#include "Obstacle/gen.h"
#include <toml.hpp>
size_t nhw_offset_range(OPoint_S16* geom_nhw_sort, int tile_size, int IH, int IW, int CH) {
    size_t ret = 0;
    size_t offset_0 = CH * sizeof(int8_t);
    size_t offset_1 = IW * offset_0;
    size_t offset_2 = IH * offset_1;

    OPoint_S16 left = *(geom_nhw_sort);
    OPoint_S16 right = *(geom_nhw_sort + tile_size - 1);
    size_t left_offset = left.n * offset_2 + left.my * offset_1 + left.mx * offset_0;
    size_t right_offset = right.n * offset_2 + right.my * offset_1 + right.mx * offset_0;
    ret = right_offset - left_offset + 1 * CH; 
    return ret;
}

size_t nhw_offset(OPoint_S16* geom_nhw_sort, int IH, int IW, int CH) {
    size_t ret = 0;
    size_t offset_0 = CH * sizeof(int8_t);
    size_t offset_1 = IW * offset_0;
    size_t offset_2 = IH * offset_1;

    OPoint_S16 p = *(geom_nhw_sort);
    ret = p.n * offset_2 + p.my * offset_1 + p.mx * offset_0; 
    return ret;
}

size_t nhwd_offset_range(OPoint_S16* geom_nhwd_sort, int tile_size, int IH, int IW, int depth) {
    size_t ret = 0;
    size_t offset_0 = depth * sizeof(int8_t);
    size_t offset_1 = IW * offset_0;
    size_t offset_2 = IH * offset_1;

    OPoint_S16 left = *(geom_nhwd_sort);
    OPoint_S16 right = *(geom_nhwd_sort + tile_size - 1);
    size_t left_offset = left.n * offset_2 + left.my * offset_1 + left.mx * offset_0 + left.d;
    size_t right_offset = right.n * offset_2 + right.my * offset_1 + right.mx * offset_0 + right.d;
    ret = right_offset - left_offset + 1;
    return ret;
}

size_t nhwd_offset(OPoint_S16* geom_nhwd_sort, int IH, int IW, int depth) {
    OPoint_S16 p = *(geom_nhwd_sort);
    size_t ret = 0;
    size_t offset_0 = depth * sizeof(int8_t);
    size_t offset_1 = IW * offset_0;
    size_t offset_2 = IH * offset_1;
    ret = p.n * offset_2 + p.my * offset_1 + p.mx * offset_0 + p.d;
    return ret;
}

size_t yxz_offset_range(OPoint_S16* geom_yxz_sort, int tile_size, int OH, int OW, int OZ, int CH) {
    size_t ret = 0;
    size_t offset_0 = CH * sizeof(int8_t);
    size_t offset_1 = OZ * offset_0;
    size_t offset_2 = OW * offset_1;

    OPoint_S16 left = *(geom_yxz_sort);
    OPoint_S16 right = *(geom_yxz_sort + tile_size - 1);
    size_t left_offset = left.py * offset_2 + left.px * offset_1 + left.pz * offset_0;
    size_t right_offset = right.py * offset_2 + right.px * offset_1 + right.pz * offset_0;
    ret = right_offset - left_offset + 1 * CH; // Note: 因为一个point tile中yxz的跨度每差1，就是差1 * CH
    return ret;
}

size_t yxz_offset(OPoint_S16* geom_yxz_sort, int OH, int OW, int OZ, int CH) {
    size_t ret = 0;
    size_t offset_0 = CH * sizeof(int8_t);
    size_t offset_1 = OZ * offset_0;
    size_t offset_2 = OW * offset_1;

    OPoint_S16 p = *(geom_yxz_sort);
    ret = p.py * offset_2 + p.px * offset_1 + p.pz * offset_0;
    return ret;
}

//  模拟离线操作: geom_preprocess 负责 geom point 的离线预处理逻辑，预处理的产物是 dump_geom_xyz.npy + tile_info.npy
int geom_preprocess(const std::string &base_path , int data_part, int& l1_tile_num, int& l2_tile_num) {
    long temp[3];
    std::string l_strVoxelPath = base_path + "/input/obstacle/voxel_num.npy";
    npy::LoadArrayFromNumpy(l_strVoxelPath, temp);
    int OW = temp[0];
    int OH = temp[1];
    int OZ = temp[2];
    string filename;
    if(data_part)
        filename = "lr";
    else
        filename = "fb";
    std::string l_strGeomPath = base_path + "/output/geom_" + filename + ".npy"; 
    auto shape = npy::LoadArrayShapeFromNumpy(l_strGeomPath);
    int batch = shape[0];
    int num_cams = shape[1];
    int depth = shape[2];
    int IH = shape[3];
    int IW = shape[4];
    int CH = OW;
    assert(max_bev_buf_range % CH == 0);

    int num_points = num_cams * depth * IH * IW;

    OPoint_S16* geom_xyz_sort = nullptr;
    std::string l_strValidPath = base_path +  "/output/" + filename + "_valid.npy";
    int total_valid_points = npy::LoadArrayFromNumpy(l_strValidPath, (int16_t*)(geom_xyz_sort));
    total_valid_points *= sizeof(int16_t);
    total_valid_points /= sizeof(OPoint_S16);
    geom_xyz_sort = static_cast<OPoint_S16*>(memalign(64, sizeof(OPoint_S16) * total_valid_points));
    npy::LoadArrayFromNumpy(l_strValidPath, (int16_t*)(geom_xyz_sort));

    TileInfo* tile_info = static_cast<TileInfo*>(memalign(64, sizeof(TileInfo) * 1024 * 100)); // 分配空间支持最大分块数量为100K个

    // printf("valid points number: %zu, ratio: %f\n", total_valid_points, 1.0f * total_valid_points / num_points);

    // 对geom_xyz进行排序，优先按照y,x,z排序
    sort(geom_xyz_sort, geom_xyz_sort + total_valid_points,
              [&](OPoint_S16 a, OPoint_S16 b) -> bool {
                        if (a.px == b.px && a.py == b.py && a.pz == b.pz) {
                            if ((((int64_t)a.n << 45) + ((int64_t)a.my << 30) + ((int64_t)a.mx << 15) + ((int64_t)a.d)) <
                                (((int64_t)b.n << 45) + ((int64_t)b.my << 30) + ((int64_t)b.mx << 15) + ((int64_t)b.d))) {
                                return true;
                            }
                            return false;
                        } else 
                        {
                            if ((((int64_t)a.py << 45) + ((int64_t)a.px << 30) + ((int64_t)a.pz << 15)) <
                                (((int64_t)b.py << 45) + ((int64_t)b.px << 30) + ((int64_t)b.pz << 15))) {
                                return true;
                            }
                            return false;
                        }
              });

    int max_l1_tile_geom_range = 0;
    int min_l1_tile_geom_range = 1024*1024*1024;
    int reduced_l1_tile_yxz_range = 0;
    int total_yxz_range = yxz_offset_range(geom_xyz_sort, total_valid_points, OH, OW, OZ, CH);
    // printf("the total_yxz_range is %d\n", total_yxz_range);

    /*---- tile finetune begin ---*/
    int l1_tile_yxz_range = 0;
    int left = 0;
    int right = left + total_valid_points - 1;
    int cur = left;
    int l1_tile_sz = 0;
    size_t total_l1_tile_sz = 0;
    vector<int> l1_tile_sz_vec;
    vector<int> prefix_sum_l1_tile_sz;
    int l1_tile_idx = 0;
    int low_diversity_tile_num = 0;

    while (cur <= right) {
        l1_tile_sz = cur - left + 1;
        if ((int)(yxz_offset_range(geom_xyz_sort + left, l1_tile_sz, OH, OW, OZ, CH)) > CH) {
            int searched_l1_tile_size = l1_tile_sz - 1;
            max_l1_tile_geom_range = max(max_l1_tile_geom_range, searched_l1_tile_size);
            min_l1_tile_geom_range = min(min_l1_tile_geom_range, searched_l1_tile_size);
            l1_tile_yxz_range = yxz_offset_range(geom_xyz_sort + left, searched_l1_tile_size, OH, OW, OZ, CH);

            reduced_l1_tile_yxz_range += l1_tile_yxz_range;
            l1_tile_sz_vec.push_back(searched_l1_tile_size);
            left = cur;
            prefix_sum_l1_tile_sz.push_back(total_l1_tile_sz);
            total_l1_tile_sz += searched_l1_tile_size;
            ++l1_tile_idx;
        } else {
            ++cur;
        }
    }
    if (total_l1_tile_sz < total_valid_points) {
        int tail_l1_tile_sz = total_valid_points - total_l1_tile_sz;
        max_l1_tile_geom_range = max(max_l1_tile_geom_range, tail_l1_tile_sz);
        min_l1_tile_geom_range = max(min_l1_tile_geom_range, tail_l1_tile_sz);
        l1_tile_sz_vec.push_back(tail_l1_tile_sz);
        prefix_sum_l1_tile_sz.push_back(total_l1_tile_sz);
        total_l1_tile_sz += tail_l1_tile_sz;
        l1_tile_yxz_range = yxz_offset_range(geom_xyz_sort + left, tail_l1_tile_sz, OH, OW, OZ, CH);
        reduced_l1_tile_yxz_range += l1_tile_yxz_range;
        ++l1_tile_idx;
    }

    int reduce_finetune_tile_sz = 0;
    int max_finetune_tile_sz = 0;
    int big_nhwd_range_cnt = 0;
    int max_finetune_yxz_range = 0;
    vector<int> fintune_l1_tile_sz_vec;
    vector<int> finetune_nhwd_range_vec;
    vector<int> finetune_nhw_range_vec;
    vector<int> finetune_yxz_range_vec;
    vector<int> prefix_sum_finetune_tile_sz;
    vector<int> prefix_sum_finetune_yxz_range;
    vector<int> finally_tile_yxz_offset;

    l1_tile_num = (OH * OW * OZ * CH * sizeof(int8_t) + max_bev_buf_range - 1) / max_bev_buf_range;
    // printf("the l1_tile_num %d\n", l1_tile_num);
    vector<bool> l1_tile_flag(l1_tile_num, true);

    int finetune_l1_tile_idx = 0;
    for (int i = 0; i < l1_tile_idx;) {
        int start_i = i;
        int tile_sz = 0;
        int left2 = prefix_sum_l1_tile_sz[start_i];
        int finetune_yxz_range = 0;
        int finetune_yxz_offset = 0;
        for (; i < l1_tile_idx; ++i) {
            finetune_yxz_offset = yxz_offset(geom_xyz_sort + left2 + tile_sz, OH, OW, OZ, CH);
            if (finetune_yxz_offset >= (finetune_l1_tile_idx + 1) * max_bev_buf_range) {
                OPoint_S16 tmp_p = *(geom_xyz_sort + left2 + tile_sz);
                break;
            }
            tile_sz += l1_tile_sz_vec[i];
        }
        if (tile_sz == 0) {
            // printf("drop in tile_sz < 0 at finetune_l1_tile_idx %d\n", finetune_l1_tile_idx);
            l1_tile_flag[finetune_l1_tile_idx++] = false;
            continue;
        } else {
            finetune_l1_tile_idx += 1;
        }

        finetune_yxz_range = yxz_offset_range(geom_xyz_sort + left2, tile_sz, OH, OW, OZ, CH);
        max_finetune_yxz_range = max(max_finetune_yxz_range, finetune_yxz_range);
        sort(geom_xyz_sort + left2, geom_xyz_sort + left2 + tile_sz,
            [&](OPoint_S16 a, OPoint_S16 b) -> bool {
                    if ((((int64_t)a.n << 45) + ((int64_t)a.my << 30) + ((int64_t)a.mx << 15) + ((int64_t)a.d)) <
                        (((int64_t)b.n << 45) + ((int64_t)b.my << 30) + ((int64_t)b.mx << 15) + ((int64_t)b.d))) {
                        return true;
                    }
                    return false;
            });
        int nhwd_range = nhwd_offset_range(geom_xyz_sort + left2, tile_sz, IH, IW, depth);
        int nhw_range = nhw_offset_range(geom_xyz_sort + left2, tile_sz, IH, IW, CH);

        finetune_nhwd_range_vec.push_back(nhwd_range); 
        finetune_nhw_range_vec.push_back(nhw_range);
        
        fintune_l1_tile_sz_vec.push_back(tile_sz);
        max_finetune_tile_sz = max(max_finetune_tile_sz, tile_sz);
        reduce_finetune_tile_sz += tile_sz;
    }

    for (int i = 0; i < finetune_l1_tile_idx; ++i) {
        int l1_tile_offset = i * max_bev_buf_range;

        if (l1_tile_flag[i]) {
            finally_tile_yxz_offset.push_back(l1_tile_offset);
            finetune_yxz_range_vec.push_back(i == l1_tile_num -1 ? (OH * OW * OZ * CH * sizeof(int8_t) - l1_tile_offset) : max_bev_buf_range);
        }
    }

    int check_total_finetune_l1_tile_range = 0;
    for (int i = 0; i < finetune_yxz_range_vec.size(); ++i) {
        check_total_finetune_l1_tile_range += finetune_yxz_range_vec[i];
    }
    // printf("finetune_yxz_range_vec sz %zu, check_total_finetune_l1_tile_range %d, and reference bev_s8_sz %zu\n", 
    //         finetune_yxz_range_vec.size(), check_total_finetune_l1_tile_range, OH * OW * OZ * CH * sizeof(int8_t));

    // 懒得加到上面的merge逻辑中，单独起循环来统计
    int prefix_sum_tile_sz = 0;
    int prefix_sum_yxz_range = 0;
    for (size_t i = 0; i < fintune_l1_tile_sz_vec.size(); ++i) {
        prefix_sum_finetune_tile_sz.push_back(prefix_sum_tile_sz);
        prefix_sum_finetune_yxz_range.push_back(prefix_sum_yxz_range);
        prefix_sum_tile_sz += fintune_l1_tile_sz_vec[i];
        prefix_sum_yxz_range += finetune_yxz_range_vec[i];
    }

    // printf("reduce_finetune_tile_sz is %d, and finetune tile num %zu, max_finetune_tile_sz %d\n", 
    //         reduce_finetune_tile_sz,   fintune_l1_tile_sz_vec.size(), max_finetune_tile_sz);
 
    vector<int> split_tile_sz_vec;
    vector<int> prefix_split_tile_sz_vec;
    vector<int> split_tile_nhwd_range_vec;
    vector<int> split_tile_nhw_range_vec;
    int max_tile_sz = 0;
    int min_tile_sz = 1024*1024*1024;
    int prefix_sum_for_fintune_tile = 0;
    int double_check_reduce_split_tile_sz = 0;
    for (size_t i = 0; i < fintune_l1_tile_sz_vec.size(); ++i) {
        int nhwd_range = finetune_nhwd_range_vec[i];
        int nhw_range = finetune_nhw_range_vec[i];
        int cur_tile_sz = fintune_l1_tile_sz_vec[i];
        if (nhwd_range > max_depth_buf_range) { // 如果希望merge tile的粒度越大，这里nhwd range上限就得降得更低
            int left2 = prefix_sum_for_fintune_tile;
            int right2 = left2 + cur_tile_sz - 1;
            int cur2 = left2;
            int finetune_tile_sz = 0;
            int inner_total_tile_sz = 0;
            while (cur2 <= right2) {
                finetune_tile_sz = cur2 - left2 + 1;
                if (nhwd_offset_range(geom_xyz_sort + left2, finetune_tile_sz, IH, IW, depth) > max_depth_buf_range) {
                    int searched_finetune_tile_sz = finetune_tile_sz - 1;
                    max_tile_sz = max(max_tile_sz, searched_finetune_tile_sz);
                    min_tile_sz = min(min_tile_sz, searched_finetune_tile_sz);
                    inner_total_tile_sz += searched_finetune_tile_sz;
                    prefix_split_tile_sz_vec.push_back(double_check_reduce_split_tile_sz);
                    double_check_reduce_split_tile_sz += searched_finetune_tile_sz;
                    split_tile_sz_vec.push_back(searched_finetune_tile_sz);
                    split_tile_nhwd_range_vec.push_back(nhwd_offset_range(geom_xyz_sort + left2, searched_finetune_tile_sz, IH, IW, depth));
                    split_tile_nhw_range_vec.push_back(nhw_offset_range(geom_xyz_sort + left2, searched_finetune_tile_sz, IH, IW, CH));
                    // printf("the split range %zu\n", nhwd_offset_range(geom_xyz_sort + left2, searched_finetune_tile_sz, IH, IW, depth));
                    left2 = cur2;
                } else {
                    ++cur2;
                }
            }
            if (inner_total_tile_sz < cur_tile_sz) {
                int tail_sz = cur_tile_sz - inner_total_tile_sz;
                max_tile_sz = max(max_tile_sz, tail_sz);
                min_tile_sz = min(min_tile_sz, tail_sz);
                prefix_split_tile_sz_vec.push_back(double_check_reduce_split_tile_sz);
                double_check_reduce_split_tile_sz += tail_sz;
                split_tile_sz_vec.push_back(tail_sz);
                split_tile_nhwd_range_vec.push_back(nhwd_offset_range(geom_xyz_sort + left2, tail_sz, IH, IW, depth));
                split_tile_nhw_range_vec.push_back(nhw_offset_range(geom_xyz_sort + left2, tail_sz, IH, IW, CH));
            }
        } else {
            prefix_split_tile_sz_vec.push_back(double_check_reduce_split_tile_sz);
            double_check_reduce_split_tile_sz += cur_tile_sz;
            split_tile_sz_vec.push_back(cur_tile_sz);
            split_tile_nhwd_range_vec.push_back(nhwd_range);
            split_tile_nhw_range_vec.push_back(nhw_range);
            max_tile_sz = max(max_tile_sz, cur_tile_sz);
            min_tile_sz = min(min_tile_sz, cur_tile_sz);
        }
        prefix_sum_for_fintune_tile += cur_tile_sz;
    }
    int little_sz_tile_cnt = 0;
    int big_sz_tile_cnt = 0;
    for (size_t i = 0; i < split_tile_sz_vec.size(); ++i) {
        if (split_tile_sz_vec[i] < 100) {
            ++little_sz_tile_cnt;
        }
        if (split_tile_sz_vec[i] > (max_geom_buf_range / sizeof(OPoint_S16))) {
            ++big_sz_tile_cnt;
        }
    }

    // printf("the little_sz_tile_cnt is %d, and big_sz_tile_cnt %d\n", little_sz_tile_cnt, big_sz_tile_cnt);
    // printf("the split_tile_sz_vec tile num %zu, and double_check_reduce_split_tile_sz %d\n", 
    //             split_tile_sz_vec.size(), double_check_reduce_split_tile_sz);
    // printf("the min_tile_sz is %d, the max_tile_sz is %d, unit is points\n", min_tile_sz, max_tile_sz);

    // split big_sz_tile来减少dsp方案geom input占用片上内存的大小
    vector<int> split_tile_sz_vec2;
    vector<int> prefix_split_tile_sz_vec2;
    vector<int> split_tile_nhwd_range_vec2;
    vector<int> split_tile_nhw_range_vec2;
    vector<int> finally_tile_nhwd_offset;
    vector<int> finally_tile_nhw_offset;

    int max_tile_sz2 = 0;
    int min_tile_sz2 = 1024*1024*1024;
    int prefix_sum_for_fintune_tile2 = 0;
    int double_check_reduce_split_tile_sz2 = 0;
    for (size_t i = 0; i < split_tile_sz_vec.size(); ++i) {
        int nhwd_range = split_tile_nhwd_range_vec[i];
        int nhw_range = split_tile_nhw_range_vec[i];
        int cur_tile_sz = split_tile_sz_vec[i];
        if (cur_tile_sz > (max_geom_buf_range / sizeof(OPoint_S16))) {
            int left2 = prefix_sum_for_fintune_tile2;
            int right2 = left2 + cur_tile_sz - 1;
            int cur2 = left2 + ((max_geom_buf_range / sizeof(OPoint_S16)) - 1);
            int finetune_tile_sz = 0;
            int inner_total_tile_sz = 0;
            while (cur2 <= right2) {
                finetune_tile_sz = cur2 - left2 + 1;
                int searched_finetune_tile_sz = finetune_tile_sz;
                max_tile_sz2 = max(max_tile_sz2, searched_finetune_tile_sz);
                min_tile_sz2 = min(min_tile_sz2, searched_finetune_tile_sz);
                inner_total_tile_sz += searched_finetune_tile_sz;
                prefix_split_tile_sz_vec2.push_back(double_check_reduce_split_tile_sz2);
                double_check_reduce_split_tile_sz2 += searched_finetune_tile_sz;
                split_tile_sz_vec2.push_back(searched_finetune_tile_sz);
                finally_tile_nhwd_offset.push_back(nhwd_offset(geom_xyz_sort + left2, IH, IW, depth));
                finally_tile_nhw_offset.push_back(nhw_offset(geom_xyz_sort + left2, IH, IW, CH));
                split_tile_nhwd_range_vec2.push_back(nhwd_offset_range(geom_xyz_sort + left2, searched_finetune_tile_sz, IH, IW, depth));
                split_tile_nhw_range_vec2.push_back(nhw_offset_range(geom_xyz_sort + left2, searched_finetune_tile_sz, IH, IW, CH));
                ++cur2;
                left2 = cur2;
                cur2 += ((max_geom_buf_range / sizeof(OPoint_S16)) - 1);
            }
            if (inner_total_tile_sz < cur_tile_sz) {
                int tail_sz = cur_tile_sz - inner_total_tile_sz;
                max_tile_sz2 = max(max_tile_sz2, tail_sz);
                min_tile_sz2 = min(min_tile_sz2, tail_sz);
                prefix_split_tile_sz_vec2.push_back(double_check_reduce_split_tile_sz2);
                double_check_reduce_split_tile_sz2 += tail_sz;
                split_tile_sz_vec2.push_back(tail_sz);
                finally_tile_nhwd_offset.push_back(nhwd_offset(geom_xyz_sort + left2, IH, IW, depth));
                finally_tile_nhw_offset.push_back(nhw_offset(geom_xyz_sort + left2, IH, IW, CH));
                split_tile_nhwd_range_vec2.push_back(nhwd_offset_range(geom_xyz_sort + left2, tail_sz, IH, IW, depth));
                split_tile_nhw_range_vec2.push_back(nhw_offset_range(geom_xyz_sort + left2, tail_sz, IH, IW, CH));
            }
        } else {
            prefix_split_tile_sz_vec2.push_back(double_check_reduce_split_tile_sz2);
            finally_tile_nhwd_offset.push_back(nhwd_offset(geom_xyz_sort + double_check_reduce_split_tile_sz2, IH, IW, depth));
            finally_tile_nhw_offset.push_back(nhw_offset(geom_xyz_sort + double_check_reduce_split_tile_sz2, IH, IW, CH));
            double_check_reduce_split_tile_sz2 += cur_tile_sz;
            split_tile_sz_vec2.push_back(cur_tile_sz);
            split_tile_nhwd_range_vec2.push_back(nhwd_range);
            split_tile_nhw_range_vec2.push_back(nhw_range);
            max_tile_sz2 = max(max_tile_sz2, cur_tile_sz);
            min_tile_sz2 = min(min_tile_sz2, cur_tile_sz);
        }
        prefix_sum_for_fintune_tile2 += cur_tile_sz;
    }

    int little_sz_tile_cnt2 = 0;
    int big_sz_tile_cnt2 = 0;
    for (size_t i = 0; i < split_tile_sz_vec2.size(); ++i) {
        if (split_tile_sz_vec2[i] < 100) {
            ++little_sz_tile_cnt2;
        }
        if (split_tile_sz_vec2[i] > (max_geom_buf_range / sizeof(OPoint_S16))) {
            ++big_sz_tile_cnt2;
        }
    }
    // printf("the little_sz_tile_cnt2 is %d, and big_sz_tile_cnt2 %d\n", little_sz_tile_cnt2, big_sz_tile_cnt2);
    // printf("the split_tile_sz_vec2 tile num %zu, and prefix_split_tile_sz_vec2 size %zu, and double_check_reduce_split_tile_sz2 %d\n", 
    //             split_tile_sz_vec2.size(),           prefix_split_tile_sz_vec2.size(),       double_check_reduce_split_tile_sz2);
    // printf("the min_tile_sz2 is %d, the max_tile_sz2 is %d, unit is points\n", min_tile_sz2, max_tile_sz2);
    
    // collect tile_info
    // printf("print some info about tile_info: \n");
    // printf("fintune_l1_tile_sz_vec size %zu, and  split_tile_sz_vec2 size %zu\n", fintune_l1_tile_sz_vec.size(), split_tile_sz_vec2.size());
    int tile_info_idx = 0;
    size_t l2_idx = 0;
    for (size_t l1_idx = 1; l1_idx < fintune_l1_tile_sz_vec.size(); ++l1_idx) {
        for (; l2_idx < split_tile_sz_vec2.size() - 1; ++l2_idx) {
            if (prefix_split_tile_sz_vec2[l2_idx] < prefix_sum_finetune_tile_sz[l1_idx]) {
                TileInfo info;
                info.depth = depth;
                if (prefix_split_tile_sz_vec2[l2_idx] == prefix_sum_finetune_tile_sz[l1_idx - 1] && 
                    prefix_split_tile_sz_vec2[l2_idx + 1] == prefix_sum_finetune_tile_sz[l1_idx]) {
                    info.tile_flag = 3;
                } else if (prefix_split_tile_sz_vec2[l2_idx] == prefix_sum_finetune_tile_sz[l1_idx - 1]) {
                    info.tile_flag = 0;
                } else if (prefix_split_tile_sz_vec2[l2_idx + 1] == prefix_sum_finetune_tile_sz[l1_idx]) {
                    info.tile_flag = 1;
                } else {
                    info.tile_flag = 2;
                }
                info.bev_offset = finally_tile_yxz_offset[l1_idx - 1];
                info.bev_range = finetune_yxz_range_vec[l1_idx - 1];
                info.depth_offset = finally_tile_nhwd_offset[l2_idx];
                info.depth_range = split_tile_nhwd_range_vec2[l2_idx];
                info.image_offset = finally_tile_nhw_offset[l2_idx]; 
                info.image_range = split_tile_nhw_range_vec2[l2_idx];
                info.geom_offset = prefix_split_tile_sz_vec2[l2_idx] * sizeof(OPoint_S16);
                info.geom_range = split_tile_sz_vec2[l2_idx] * sizeof(OPoint_S16);
                tile_info[tile_info_idx++] = info;
            } else {
                break;
            }
        }
    }
    for (; l2_idx < split_tile_sz_vec2.size(); ++l2_idx) {
        int finetune_l1_tile_num = fintune_l1_tile_sz_vec.size();
        TileInfo info;
        info.depth = depth;
        if (prefix_split_tile_sz_vec2[l2_idx] == prefix_sum_finetune_tile_sz[finetune_l1_tile_num - 1] && l2_idx == split_tile_sz_vec2.size() - 1) {
            info.tile_flag = 3; 
        } else if (prefix_split_tile_sz_vec2[l2_idx] == prefix_sum_finetune_tile_sz[finetune_l1_tile_num - 1]) {
            info.tile_flag = 0; 
        } else if (l2_idx == split_tile_sz_vec2.size() - 1) {
            info.tile_flag = 1;
        } else {
            info.tile_flag = 2;
        }
        info.bev_offset = finally_tile_yxz_offset[finetune_l1_tile_num - 1];
        info.bev_range = finetune_yxz_range_vec[finetune_l1_tile_num - 1];
        info.depth_offset = finally_tile_nhwd_offset[l2_idx];
        info.depth_range = split_tile_nhwd_range_vec2[l2_idx];
        info.image_offset = finally_tile_nhw_offset[l2_idx];
        info.image_range = split_tile_nhw_range_vec2[l2_idx];
        info.geom_offset = prefix_split_tile_sz_vec2[l2_idx] * sizeof(OPoint_S16);
        info.geom_range = split_tile_sz_vec2[l2_idx] * sizeof(OPoint_S16);
        tile_info[tile_info_idx++] = info;
    }
    l2_tile_num = split_tile_sz_vec2.size();
    // printf("the tile_info_idx %d vs split_tile_sz_vec2.size() %zu\n", tile_info_idx, split_tile_sz_vec2.size());

    // dump tile_info
    if (split_tile_sz_vec2.size() < 100 * 1024) {
        array<unsigned long, 1> tile_info_shape{split_tile_sz_vec2.size() * sizeof(TileInfo) / sizeof(int32_t)};
        npy::SaveArrayAsNumpy(base_path + "/output/" + filename + "_tile_info.npy", false, tile_info_shape.size(), tile_info_shape.data(), (int32_t*)tile_info);
    }

    // printf("reduced_l1_tile_yxz_range is %d, reference total_yxz_range is %d\n", reduced_l1_tile_yxz_range, total_yxz_range);
    // printf("the total_l1_tile_sz is %zu ref total_valid_points %zu, max_l1_tile_geom_range update to %d, min_l1_tile_geom_range update to %d\n", 
    //             total_l1_tile_sz,           total_valid_points,     max_l1_tile_geom_range,              min_l1_tile_geom_range);
    /*---- tile finetune end ---*/

    // printf("before geom reorg\n");
    int16_t* new_geom_buf = (int16_t*)memalign(64, total_valid_points * sizeof(OPoint_S16));
    int16_t* geom_ptr_base = (int16_t*)geom_xyz_sort;
    int dst_s16_geom_val_idx = 0;
    int src_s16_geom_val_idx_base = 0;
    int src_s16_geom_val_idx = 0;
    int tmp_cnt = 0;
    for (size_t i = 0; i < split_tile_sz_vec2.size(); ++i) {
        int cur_geom_offset = tile_info[i].geom_offset;
        int cur_geom_range = tile_info[i].geom_range;
        int range_step = 32 * sizeof(OPoint_S16);
        src_s16_geom_val_idx_base = (cur_geom_offset / sizeof(OPoint_S16)) * 8;
        for (int j = 0; j < cur_geom_range; j += range_step) {
            int step_points = ((j + range_step) <= cur_geom_range) ? 32 : (cur_geom_range % range_step / (int)(sizeof(OPoint_S16)));
            for (int j2 = 0; j2 < 8; ++j2) {
                src_s16_geom_val_idx = src_s16_geom_val_idx_base + (j2 + 4) % 8;
                for (int j3 = 0; j3 < step_points; ++j3) {
                    new_geom_buf[dst_s16_geom_val_idx++] = geom_ptr_base[src_s16_geom_val_idx];
                    src_s16_geom_val_idx += 8;
                    ++tmp_cnt;
                }
            }
            src_s16_geom_val_idx_base += step_points * 8;
        }
    }

    // dump dump_geom_xyz
    array<unsigned long, 1> geom_xyz_shape{total_valid_points * sizeof(OPoint_S16) / sizeof(int16_t)};
    npy::SaveArrayAsNumpy(base_path + "/output/" + filename + "_dump_geom_xyz.npy", false, geom_xyz_shape.size(), geom_xyz_shape.data(), (int16_t*)new_geom_buf);
    // printf("after dump_geom_xyz\n");

    // free memalign allocated buf
    free(geom_xyz_sort);
    free(tile_info);
    free(new_geom_buf);

    return total_valid_points;
}

initParams test_lvt_origin(const std::string &file_path ,int l1_tile_num, int l2_tile_num_demo53, int l2_tile_num_demo54, int total_valid_points_demo53, int total_valid_points_demo54) {
    long temp[3];
    std::string l_strVoxelPath = file_path + "/input/obstacle/voxel_num.npy";
    npy::LoadArrayFromNumpy(l_strVoxelPath, temp);
    int OW = temp[0];
    int OH = temp[1];
    int OZ = temp[2];
    
    auto front_back_shape = npy::LoadArrayShapeFromNumpy(file_path + "/output/geom_fb.npy");
    int batch = front_back_shape[0];
    int num_cams_demo53 = front_back_shape[1];
    int depth_demo53 = front_back_shape[2];
    int IH = front_back_shape[3];
    int IW = front_back_shape[4];
    int CH = OW;
    auto left_right_shape = npy::LoadArrayShapeFromNumpy(file_path + "/output/geom_lr.npy");
    int num_cams_demo54 = left_right_shape[1];
    int depth_demo54 = left_right_shape[2];
    assert(max_bev_buf_range % CH == 0);

    OPoint_S16* geom_xyz_sort_demo53 = static_cast<OPoint_S16*>(memalign(64, sizeof(OPoint_S16) * total_valid_points_demo53));
    TileInfo* tile_info_demo53 = static_cast<TileInfo*>(memalign(64, sizeof(TileInfo) * l2_tile_num_demo53));

    OPoint_S16* geom_xyz_sort_demo54 = static_cast<OPoint_S16*>(memalign(64, sizeof(OPoint_S16) * total_valid_points_demo54));
    TileInfo* tile_info_demo54 = static_cast<TileInfo*>(memalign(64, sizeof(TileInfo) * l2_tile_num_demo54));

    npy::LoadArrayFromNumpy(file_path + "/output/fb_dump_geom_xyz.npy", (int16_t*)geom_xyz_sort_demo53);
    npy::LoadArrayFromNumpy(file_path + "/output/fb_tile_info.npy", (int32_t*)tile_info_demo53);

    npy::LoadArrayFromNumpy(file_path + "/output/lr_dump_geom_xyz.npy", (int16_t*)geom_xyz_sort_demo54);
    npy::LoadArrayFromNumpy(file_path + "/output/lr_tile_info.npy", (int32_t*)tile_info_demo54);

    // 构建fuse_tile_info, Note: fuse_tile_info也是离线逻辑，不用每一帧重新构建一次fuse_tile_info
    int finally_tile_num = l2_tile_num_demo53 + l2_tile_num_demo54;
    TileInfo* fuse_tile_info = static_cast<TileInfo*>(memalign(64, sizeof(TileInfo) * finally_tile_num));
    // 调整demo54所有tileinfo的image_offset/depth_offset/geom_offset
    for (size_t i = 0; i < l2_tile_num_demo54; ++i) {
        tile_info_demo54[i].image_offset += num_cams_demo53 * IH * IW * CH * sizeof(int8_t);
        tile_info_demo54[i].depth_offset += num_cams_demo53 * IH * IW * depth_demo53 * sizeof(int8_t);
        tile_info_demo54[i].geom_offset += total_valid_points_demo53 * sizeof(OPoint_S16);
    }

    int tile_idx_0 = 0;
    int tile_idx_1 = 0;
    int fuse_tile_idx = 0;
    int fuse_l1_tile_idx = 0;
    int l2_tile_num_core0 = finally_tile_num;
    int l2_tile_num_core1 = 0;
    int l2_tile_num_core2 = 0;
    int l2_tile_num_core3 = 0;
    int reduce_point_num = 0;
    int work_load_per_core = (total_valid_points_demo53 + total_valid_points_demo54) / dsp_core_num;
    //int work_load_per_core = finally_tile_num / dsp_core_num;
    bool flag0 = true;
    bool flag1 = true;
    bool flag2 = true;
    bool flag3 = true;
    for (fuse_l1_tile_idx = 0; fuse_l1_tile_idx < l1_tile_num;) {
        //int reduce_work_load = fuse_tile_idx;
        int reduce_work_load = reduce_point_num;
        if (dsp_core_num == 2) {
            if (reduce_work_load >= work_load_per_core && flag0) {
                l2_tile_num_core0 = fuse_tile_idx;
                l2_tile_num_core1 = finally_tile_num - l2_tile_num_core0;
                flag0 = false;
                printf("l2_tile_num_core0 %d, l2_tile_num_core1 %d\n", l2_tile_num_core0, l2_tile_num_core1);
            }
        }
        if (dsp_core_num == 3) {
            if (reduce_work_load >= work_load_per_core && flag0) {
                l2_tile_num_core0 = fuse_tile_idx;
                flag0 = false;
            }
            if (reduce_work_load >= work_load_per_core * 2 && flag1) {
                l2_tile_num_core1 = fuse_tile_idx - l2_tile_num_core0;
                l2_tile_num_core2 = finally_tile_num - l2_tile_num_core1 - l2_tile_num_core0;
                flag1 = false;
                printf("l2_tile_num_core0 %d, l2_tile_num_core1 %d, l2_tile_num_core2 %d\n", 
                        l2_tile_num_core0, l2_tile_num_core1, l2_tile_num_core2);
            }
        }
        if (dsp_core_num == 4) {
            if (reduce_work_load >= work_load_per_core && flag0) {
                l2_tile_num_core0 = fuse_tile_idx;
                flag0 = false;
            }
            if (reduce_work_load >= work_load_per_core * 2 && flag1) {
                l2_tile_num_core1 = fuse_tile_idx - l2_tile_num_core0;
                flag1 = false;
            }
            if (reduce_work_load >= work_load_per_core * 3 && flag2) {
                l2_tile_num_core2 = fuse_tile_idx - l2_tile_num_core1 - l2_tile_num_core0;
                l2_tile_num_core3 = finally_tile_num - l2_tile_num_core2 - l2_tile_num_core1 - l2_tile_num_core0;
                flag2 = false;
                // printf("l2_tile_num_core0 = %d;\nl2_tile_num_core1 = %d;\nl2_tile_num_core2 = %d;\nl2_tile_num_core3 = %d;\n", 
                //         l2_tile_num_core0,       l2_tile_num_core1,       l2_tile_num_core2,       l2_tile_num_core3);
            }
        }

        int cur_l1_tile_offset = fuse_l1_tile_idx * max_bev_buf_range;
        int back_tile_idx_0 = tile_idx_0;
        while (tile_info_demo53[tile_idx_0].bev_offset == cur_l1_tile_offset && fuse_tile_idx < finally_tile_num) {
            //printf("tile_idx_0 %d, and fuse_tile_idx %d ", tile_idx_0, fuse_tile_idx);
            if (tile_info_demo53[tile_idx_0].tile_flag == 3) {
                if (tile_info_demo54[tile_idx_1].bev_offset == cur_l1_tile_offset) {
                    tile_info_demo53[tile_idx_0].tile_flag = 0;
                }
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo53[tile_idx_0++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
                break;
            } else if (tile_info_demo53[tile_idx_0].tile_flag == 1) {
                if (tile_info_demo54[tile_idx_1].bev_offset == cur_l1_tile_offset) {
                    tile_info_demo53[tile_idx_0].tile_flag = 2;
                }
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo53[tile_idx_0++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
                break;
            } else {
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo53[tile_idx_0++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
            }
        }
        while (tile_info_demo54[tile_idx_1].bev_offset == cur_l1_tile_offset && fuse_tile_idx < finally_tile_num) {
            //printf("tile_idx_1 %d, and fuse_tile_idx %d ", tile_idx_1, fuse_tile_idx);
            if (tile_info_demo54[tile_idx_1].tile_flag == 3) {
                if (tile_info_demo53[back_tile_idx_0].bev_offset == cur_l1_tile_offset) {
                    tile_info_demo54[tile_idx_1].tile_flag = 1;
                }
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo54[tile_idx_1++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
                break;
            } else if (tile_info_demo54[tile_idx_1].tile_flag == 0) {
                if (tile_info_demo53[back_tile_idx_0].bev_offset == cur_l1_tile_offset) {
                    tile_info_demo54[tile_idx_1].tile_flag = 2;
                }
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo54[tile_idx_1++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
            } else if (tile_info_demo54[tile_idx_1].tile_flag == 1) {
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo54[tile_idx_1++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
                break;
            } else {
                fuse_tile_info[fuse_tile_idx++] = tile_info_demo54[tile_idx_1++];
                reduce_point_num += fuse_tile_info[fuse_tile_idx - 1].geom_range / sizeof(OPoint_S16);
            }
        }
        ++fuse_l1_tile_idx;
    }

    assert(fuse_tile_idx == finally_tile_num);

    initParams params;
    params.finally_tile_num = finally_tile_num;
    params.fuse_total_image_feat_sz = num_cams_demo53 * IH * IW * CH * sizeof(int8_t) + num_cams_demo54 * IH * IW * CH * sizeof(int8_t);;
    params.fuse_total_depth_feat_sz = num_cams_demo53 * IH * IW * depth_demo53 * sizeof(int8_t) + num_cams_demo54 * IH * IW * depth_demo54 * sizeof(int8_t);;
    params.fuse_total_valid_points = total_valid_points_demo53 + total_valid_points_demo54;
    params.fuse_total_tile_info_range = finally_tile_num * sizeof(TileInfo);;
    params.total_valid_points_front_back = total_valid_points_demo53;
    params.total_valid_points_left_right = total_valid_points_demo54;
    params.l2_tile_num[0] = l2_tile_num_core0;
    params.l2_tile_num[1] = l2_tile_num_core1;
    params.l2_tile_num[2] = l2_tile_num_core2;
    params.l2_tile_num[3] = l2_tile_num_core3;
    
    // cout<<"struct initParams size : " << sizeof(initParams) << endl;
    // save bin
    // ofstream ofile("obstacleLSS.bin", std::ios::binary);
    // ofile.write((char*)&params, sizeof(initParams));
    // ofile.write((char*)geom_xyz_sort_demo53, total_valid_points_demo53 * sizeof(OPoint_S16));
    // ofile.write((char*)geom_xyz_sort_demo54, total_valid_points_demo54 * sizeof(OPoint_S16));
    // ofile.write((char*)fuse_tile_info, finally_tile_num * sizeof(TileInfo));
    // old version
    ofstream ofile1(file_path + "/output/obstacle_bin/geom_xyz_sort_demo53.bin", std::ios::binary);
    ofile1.write((char*)geom_xyz_sort_demo53, total_valid_points_demo53 * sizeof(OPoint_S16));
    ofstream ofile2(file_path + "/output/obstacle_bin/geom_xyz_sort_demo54.bin", std::ios::binary);
    ofile2.write((char*)geom_xyz_sort_demo54, total_valid_points_demo54 * sizeof(OPoint_S16));
    ofstream ofile3(file_path + "/output/obstacle_bin/fuse_tile_info.bin", std::ios::binary);
    ofile3.write((char*)fuse_tile_info, finally_tile_num * sizeof(TileInfo));

    return params;
}
// void GenObstacle(const std::string &base_path)
// {
//     int depth_demo53 = 160;
//     int depth_demo54 = 112;
//     assert((max_image_buf_range / (max_depth_buf_range * 1.0f))  >= 
//            (min(depth_demo53, depth_demo54)/ (max(depth_demo53, depth_demo54)/ 1.0f)));

//     // 离线预处理逻辑
//     int l1_tile_num = 0;
//     int l2_tile_num_demo53 = 0;
//     int l2_tile_num_demo54 = 0;
//     int total_valid_points_demo53 = geom_preprocess(base_path , 0, l1_tile_num, l2_tile_num_demo53); // 调用demo53 geom_xyz的预处理
//     int total_valid_points_demo54 = geom_preprocess(base_path , 1, l1_tile_num, l2_tile_num_demo54); // 调用demo54 geom_xyz的预处理
//     printf("Change these codes in devastator : \n");
//     printf("\033[47m\033[34mfinally_tile_num = %d + %d;\n", l2_tile_num_demo53, l2_tile_num_demo54);

//     // dsp 调用逻辑
//     // printf("total_valid_points_demo53 %d, total_valid_points_demo54 %d\n", total_valid_points_demo53, total_valid_points_demo54);
//     test_lvt_origin(base_path ,l1_tile_num, l2_tile_num_demo53, l2_tile_num_demo54, total_valid_points_demo53, total_valid_points_demo54);
//     printf("\033[0mBin generation finished.\n\033[0m");
// }
void UpdateScalingParameters(const std::string &config_path , const int &finally_tile_num , const initParams &params)
{
    std::cout<<"config dst path is "<<config_path<<std::endl;
    std::string l_strTomlPath = config_path + "/allinone.toml"; 
    auto l_tmData = toml::parse(l_strTomlPath);
    auto &l_tmScalingParams = toml::find(l_tmData, "scaling_parameters");
    auto &l_tmFinallyTileNum = toml::find(l_tmScalingParams, "finally_tile_num"); 
    auto &l_tmL2Tile_NumCore0 = toml::find(l_tmScalingParams, "l2_tile_num_core0"); 
    auto &l_tmL2Tile_NumCore1 = toml::find(l_tmScalingParams, "l2_tile_num_core1"); 
    auto &l_tmL2Tile_NumCore2 = toml::find(l_tmScalingParams, "l2_tile_num_core2"); 
    auto &l_tmL2Tile_NumCore3 = toml::find(l_tmScalingParams, "l2_tile_num_core3"); 
    l_tmFinallyTileNum = finally_tile_num;
    l_tmL2Tile_NumCore0 = params.l2_tile_num[0];
    l_tmL2Tile_NumCore1 = params.l2_tile_num[1];
    l_tmL2Tile_NumCore2 = params.l2_tile_num[2];
    l_tmL2Tile_NumCore3 = params.l2_tile_num[3];
    std::ofstream l_outStream(l_strTomlPath);
    l_outStream << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << l_tmData;
    l_outStream.close();
}
int main(int argc, char* argv[]) {    
    if (argc != 3)
    {
        std::cout<<"please input path , dst config path"<<std::endl;
    }
    std::string l_strBasePath = argv[1];
    std::string l_strConfigPath = argv[2];
    int depth_demo53 = 160;
    int depth_demo54 = 112;
    assert((max_image_buf_range / (max_depth_buf_range * 1.0f))  >= 
           (min(depth_demo53, depth_demo54)/ (max(depth_demo53, depth_demo54)/ 1.0f)));

    // 离线预处理逻辑
    int l1_tile_num = 0;
    int l2_tile_num_demo53 = 0;
    int l2_tile_num_demo54 = 0;
    int total_valid_points_demo53 = geom_preprocess(l_strBasePath , 0, l1_tile_num, l2_tile_num_demo53); // 调用demo53 geom_xyz的预处理
    int total_valid_points_demo54 = geom_preprocess(l_strBasePath , 1, l1_tile_num, l2_tile_num_demo54); // 调用demo54 geom_xyz的预处理
    // printf("Change these codes in devastator : \n");
    // printf("\033[47m\033[34mfinally_tile_num = %d + %d;\n", l2_tile_num_demo53, l2_tile_num_demo54);

    // dsp 调用逻辑
    // printf("total_valid_points_demo53 %d, total_valid_points_demo54 %d\n", total_valid_points_demo53, total_valid_points_demo54);
    initParams params = test_lvt_origin(l_strBasePath ,l1_tile_num, l2_tile_num_demo53, l2_tile_num_demo54, total_valid_points_demo53, total_valid_points_demo54);
    // printf("\033[0mBin generation finished.\n\033[0m");
    UpdateScalingParameters(l_strConfigPath,l2_tile_num_demo53+l2_tile_num_demo54,params);
    return 0;
}
