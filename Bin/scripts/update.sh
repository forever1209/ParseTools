#!/bin/bash
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
    parse_tool_bin_path=$cwd"/../ParseTool"
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
CopyFile()
{
    echo "now copy all file"
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

DownFovsIntrinsicPkl $pkl_path
echo -e "\n ---------------------------------------------------\n DownFovsIntrinsicPkl done"
if [ "$break_models" = True ]; then
    echo "Breaking models..."
else
    DownloadModels $models_path
    echo -e "\n ---------------------------------------------------\n DownloadModels done"
fi
UpdateConfigFile $vehicle_id $dst_config_path
echo -e "\n ---------------------------------------------------\n UpdateConfigFile done"
CopyFile
echo -e "\n ---------------------------------------------------\n CopyFile done"