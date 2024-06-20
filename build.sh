#!/bin/bash
cwd=$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}" )" &> /dev/null && pwd )
rm -rf $cwd/Bin/lib/*
build_project()
{
    #build pytool
    cd $cwd/PythonTool
    source_dir=$cwd/PythonTool/build/
    target_dir=$cwd/Bin/lib
    python3 setup.py build_ext
    find "$source_dir" -type f -name "*.so" -exec mv {} "$target_dir" \;
    #build project
    cd $cwd
    mkdir -p build && cd build
    cmake .. && make -j$1
    LIBS=$(ldd $cwd/Bin/ParseTool | grep "=>" | awk '{print $3}')
    for lib in $LIBS; do
        if [ -f $lib ]; then  # 确保文件存在
            cp $lib $target_dir
            echo "Copied $lib to $target_dir"
        else
            echo "Error: $lib does not exist or is not a regular file."
        fi
    done
}
usage() {
    echo "Usage: ./build.sh [options]"
    echo "Options:"
    echo "  -h      Display this help message"
    echo "  -b      Build Project use build params"
    echo "  -r      Rebuild Project use build params"
    echo "Example :"
    echo "./build.sh -h "
    echo "./build.sh -b 6  "
}

while getopts "b:r:h" opt; do
  case $opt in
    b)
        build_params=$OPTARG
        build_project $build_params
    ;;
    r)
        rm -rf $cwd/build/
        rm -rf $cwd/PythonTool/build/
        build_params=$OPTARG
        build_project $build_params
    ;;
    h)
        usage
        exit 1
    ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        exit 1
    ;;
  esac
done


