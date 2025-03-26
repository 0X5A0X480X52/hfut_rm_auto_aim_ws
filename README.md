# 基于中南 FYT2024_vision 修改的自瞄方案

## 环境配置

- fmt库
  ```bash
  sudo apt install libfmt-dev
  ```
- abseil-cpp (Ceres-solver依赖)
    ```bash
    wget https://github.com/abseil/abseil-cpp/archive/refs/tags/20250127.1.zip
    unzip 20250127.1.zip
    cd 20250127.1
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```
- googletest (Ceres-solver依赖)
    ```bash
    sudo apt update
    sudo apt install -y libgtest-dev googletest
    cd /usr/src/googletest
    mkdir build && cd build
    sudo cmake -DBUILD_GMOCK=ON ..
    sudo make -j
    sudo make install
    ```
- ceres-solver (Sophus库依赖)
    ```bash
    git clone https://github.com/ceres-solver/ceres-solver
    cd ceres-solver
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```
    如果出现报错：
    ```bash
    /root/Robomaster/Libs/ceres-solver/internal/ceres/test_util.cc: In function ‘std::string ceres::internal::TestFileAbsolutePath(const string&)’:
    /root/Robomaster/Libs/ceres-solver/internal/ceres/test_util.cc:136:30: error: ‘SrcDir’ is not a member of ‘testing’
    136 |   return JoinPath(::testing::SrcDir() + CERES_TEST_SRCDIR_SUFFIX, filename);
        |                              ^~~~~~
    ```
    可将源码进行以下更改再进行编译：
    ```C++
    std::string TestFileAbsolutePath(const std::string& filename) {
    // return JoinPath(::testing::SrcDir() + CERES_TEST_SRCDIR_SUFFIX, filename);
    return JoinPath(::testing::UnitTest::GetInstance()->original_working_dir(), filename);
    }
    ```
- Sophus库 (G2O库依赖)
   ```bash
   git clone https://github.com/strasdat/Sophus
   cd Sophus
   mkdir build && cd build
   cmake ..
   make -j
   sudo make install
   ```
- G2O库 (优化装甲板Yaw角度)
    ```bash
    sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```

- 出现类似以下缺少 ros2 包的问题
    ```bash
    CMake Error at /root/Robomaster/hfut_rm_auto_aim_ws/install/models/share/models/cmake/ament_cmake_export_include_directories-extras.cmake:8 (find_package):
    By not providing "Findament_cmake_core.cmake" in CMAKE_MODULE_PATH this
    project has asked CMake to find a package configuration file provided by
    "ament_cmake_core", but CMake did not find one.

    Could not find a package configuration file provided by "ament_cmake_core"
    with any of the following names:

        ament_cmake_coreConfig.cmake
        ament_cmake_core-config.cmake

    Add the installation prefix of "ament_cmake_core" to CMAKE_PREFIX_PATH or
    set "ament_cmake_core_DIR" to a directory containing one of the above
    files.  If "ament_cmake_core" provides a separate development package or
    SDK, be sure it has been installed.
    Call Stack (most recent call first):
    /root/Robomaster/hfut_rm_auto_aim_ws/install/models/share/models/cmake/modelsConfig.cmake:41 (include)
    CMakeLists.txt:19 (find_package)
    ```
    首先尝试尝试安装缺少的ros2包：
    ```
    sudo apt update
    sudo apt install ros-humble-ament-package
    ```
    如果已存在，运行ros2脚本配置环境变量：
    ```
    source /opt/ros/humble/setup.bash
    ```
    

## 参考项目：
- [中南 FYT2024_vision 开源](https://github.com/CSU-FYT-Vision/FYT2024_vision.git)