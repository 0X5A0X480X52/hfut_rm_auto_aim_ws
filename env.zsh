#COLCON编译配置
export COLCON_DEFAULTS_FILE=$PWD/config/.colcon_config.yaml

#RMW配置

#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
#export RMW_IMPLEMENTATION=rmw_iceoryx_cpp

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_FASTRTPS_USE_QOS_FROM_XML=1
# export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_profile.xml

#mimalloc内存分配器哦，位置有时可能需要修订
export LD_PRELOAD=/usr/local/lib/libmimalloc.so:$LD_PRELOAD

#gcc-14库路径
export LD_LIBRARY_PATH=/usr/local/gcc-14/lib64:$LD_LIBRARY_PATH

#调试信息库
export DEBUGINFOD_URLS="https://debuginfod.ubuntu.com"

#控制台输出格式
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name}]: {message}"

#输出屏幕设定
export DISPLAY=:0

#Intel MKL数学优化库
#source /opt/intel/oneapi/setvars.sh
#colcon 函数包装器

colcon() {
    local ONEAPI_VARS="/opt/intel/oneapi/setvars.sh"
    
    if [[ -f "$ONEAPI_VARS" ]]; then
        source "$ONEAPI_VARS" intel64>/dev/null 2>&1
    else
        echo "[Warning]: $ONEAPI_VARS not found. Running colcon without oneAPI."
    fi
    command colcon "$@"
}
