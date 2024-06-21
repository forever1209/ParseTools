#!/bin/bash
declare -A VehicleMap
VehicleMap["car602"]="car_00602"
VehicleMap["car201"]="car_byd_han_00001"
VehicleMap["car202"]="car_byd_han_00002"
vehicle_id=""
dst_config_path=""
models_path=""
pkl_path=""
type=""
break_models=False
cwd=$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}" )" &> /dev/null && pwd )
export LD_LIBRARY_PATH="$cwd/../lib:$LD_LIBRARY_PATH"
oss() {
    aws --endpoint-url=http://oss-internal.i.brainpp.cn/ s3 "$@"
}  
DownloadModels()
{
    src_url=$1
    dst_path=$cwd"/../ChildrenPath/models/"
    mkdir -p $dst_path
    str_cmd="oss cp s3://$src_url $dst_path --recursive --exclude 'quant_onnx/*' "
    echo $str_cmd
    eval "$str_cmd"
}
DownFovsIntrinsicPkl()
{
    src_url=$1
    dst_path=$cwd"/../input/"
    str_cmd="oss cp s3://$src_url $dst_path "
    rm -rf $dst_path/*.pkl
    echo $str_cmd
    eval "$str_cmd"
    find $dst_path -type f -name "*.pkl" -exec mv {} $dst_path/fovs_intrinsic.pkl \;
}
UpdateConfigFile()
{
    car_id=$1
    config_path=$2
    echo "update config"
    parse_tool_bin_path=$cwd"/../CalibArgsAutoUpdateTool"
    conflate_models_cmd="$parse_tool_bin_path -m $type -i $car_id -d $config_path"
    echo $conflate_models_cmd
    eval "$conflate_models_cmd"
    
    # obstacle_tool_bin_path=$cwd"/../ObstacleTool"
    se_tool_bin_path=$cwd"/../SeTool"
    se_bin_cmd="$se_tool_bin_path $cwd/../ChildrenPath/models/inputs"
    echo $se_bin_cmd
    eval "$se_bin_cmd"

    config_update_cmd="$parse_tool_bin_path -d $config_path  -i $car_id"
    echo $config_update_cmd
    eval "$config_update_cmd"

    camera_bin_cmd="$parse_tool_bin_path -c -i $car_id"
    echo $camera_bin_cmd
    eval "$camera_bin_cmd"
}
UpdateModelsPath()
{
    local models_path=$1
    local new_model_path=$2
    local config_allinone=$dst_config_path/allinone.toml
    local ModelPathTool=$cwd"/../ModelTool"
    path_update_cmd="$ModelPathTool $config_allinone $models_path $new_model_path"
    echo $path_update_cmd
    eval "$path_update_cmd"
}
CopyFile()
{
    car_id=$1
    config_path=$2
    models_time=$3
    local calib_path=$cwd"/../ChildrenPath/calibresult/${VehicleMap[$car_id]}"
    local latest_folder=""
    for folder in $calib_path/*; do
        # 检查是否是文件夹
        if [ -d "$folder" ]; then
            # 提取文件夹名中的日期部分
            folder_date=$(basename "$folder")
            if [[ $folder_date =~ ^[0-9]{8}$ ]]; then
                # 如果日期格式正确，比较日期
                if [ -z "$latest_folder" ] || [[ $folder_date -gt $latest_folder ]]; then
                    latest_folder=$folder_date
                fi
            fi
        fi
    done
    # echo "calib_path is $calib_path latest_folder is $latest_folder"
    local update_models_time=""
    if [[ $models_time =~ ([0-9]{8}) ]]; then
        update_models_time=${BASH_REMATCH[1]}  # 使用BASH_REMATCH数组获取匹配的部分
        echo "The extracted date is: $update_models_time"
    else
        echo "Date not found in the $models_time."
        exit 1
    fi
    local models_path=$config_path/../../models/$car_id/merge_obstacle_lane_bev/a1000/model$update_models_time"_nv12calib$latest_folder"
    # echo "now begin copy all file models_path is $models_path"
    mkdir -p $models_path
    rm -rf $models_path/*
    local src_path=$cwd"/../ChildrenPath/models/"
    local src_devastator=$cwd"/../output/"
    for folder in "$src_path"/*; do
        # 检查是否是文件夹且名称包含lanedet_
        if [ -d "$folder" ] && [[ $(basename "$folder") == lanedet_* ]]; then
            # 提取新的文件夹名称
            new_folder_name=$(basename "$folder" | sed 's/^lanedet_//')
            new_folder_path="$models_path/$new_folder_name"
            mv $folder $new_folder_path
        fi
    done
    mkdir -p "$models_path/devastator_bin"
    cp $src_devastator/lane_bin/* "$models_path/devastator_bin/"
    cp $src_devastator/obstacle_bin/* "$models_path/devastator_bin/"
    mv $src_devastator/se_bin "$models_path/devastator_bin/se"
    UpdateModelsPath $config_path/model model$update_models_time"_nv12calib$latest_folder"
}
usage()
{
    echo "Usage: ./update.sh [options]"
    echo "Options:"
    echo "  -h      Display this help message"
    echo "  -v      Set vehicle id"
    echo "  -d      Set dst config path"
    echo "  -m      Set Models path"
    echo "  -p      Set PKL path"
    echo "  -b      Break download path"
    echo "  -t      Set Type [BYD/HS]"
    echo "Example :"
    echo "bash update.sh -v car602 
    -d /home/liulei10/LL/ParseJson/Test/deva-dev-config-car602/config/car602 
    -m lane-det/share/20240527hs/ 
    -p camera-perceptron/resources/calib/fovs_intrinsic_k_dict_tmp.pkl
    -t HS "
}

# DownloadModels "lane-det/share/20240527hs/"

while getopts "v:d:m:p:t:hb" opt; do
  case $opt in
    v)
        vehicle_id=$OPTARG
    ;;
    d)
        dst_config_path=$OPTARG
    ;;
    m)
        models_path=$OPTARG
    ;;
    p)
        pkl_path=$OPTARG
    ;;
    t)
        type=$OPTARG
    ;;
    h)
        usage
        exit 1
    ;;
    b)
        break_models=True
    ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        exit 1
    ;;
  esac
done

# echo "car id is $vehicle_id"
# echo "dst path is $dst_config_path"
# echo "models path is $models_path"
# echo "pkl_path is $pkl_path"
# echo "type is $type"
if [ -z "$vehicle_id" ] || [ -z "$dst_config_path" ] || [ -z "$models_path" ] || [ -z "$pkl_path" ] || [ -z "$type" ]; then
    echo "Error: One or more variables are empty."
    usage
    exit 1
fi
echo -e "\n ---------------------------------------------------\n"
DownFovsIntrinsicPkl $pkl_path
echo -e "\n ---------------------------------------------------\n DownFovsIntrinsicPkl done"
if [ "$break_models" = True ]; then
    echo "Breaking models..."
else
    echo -e "\n ---------------------------------------------------\n"
    DownloadModels $models_path
    echo -e "\n ---------------------------------------------------\n DownloadModels done"
fi
echo -e "\n ---------------------------------------------------\n"
UpdateConfigFile $vehicle_id $dst_config_path
echo -e "\n ---------------------------------------------------\n UpdateConfigFile done"
echo -e "\n ---------------------------------------------------\n"
CopyFile $vehicle_id $dst_config_path $models_path
echo -e "\n ---------------------------------------------------\n CopyFile done"