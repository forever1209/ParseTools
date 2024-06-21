#!/bin/bash
cwd=$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}" )" &> /dev/null && pwd )

CleanBin()
{
    rm -rf "$cwd/../ChildrenPath/"
    rm -rf "$cwd/../lib/"
    rm -rf "$cwd/../ObstacleTool"
    rm -rf "$cwd/../CalibArgsAutoUpdateTool"
    rm -rf "$cwd/../ModelTool"
    rm -rf "$cwd/../SeTool"
    mkdir -p "$cwd/../lib/"
}
CleanOutput()
{
    rm -rf "$cwd/../output/"
}
CleanBuild()
{
    rm -rf "$cwd/../../build/"
}
CleanBin
CleanOutput
CleanBuild
