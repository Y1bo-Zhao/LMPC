# Read Me

本仓库记录了如何在 Linux 系统中安装 [OSQP](https://github.com/oxfordcontrol/osqp) 和它的 C++ 封装器 [osqp-eigen](https://github.com/robotology/osqp-eigen)。并基于OSQP解决LMPC问题

>  `osqp-eigen` 目前与 OSQP 的较新版本可能不兼容。为避免安装失败，建议使用 `OSQP release-0.6.3` 版本。

---

##  安装 OSQP


```bash
安装osqp：（由于osqp-eigen更新较慢，下载osqp源码时要指定老版本）
git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
cd osqp
mkdir build && cd build
cmake .. 
make
sudo make install

测试 OSQP 是否安装成功
cd ~/osqp/build/out
./osqp_demo
应该会看到一些输出结果，表示一个小的二次规划问题被成功求解。

卸载 OSQP（可选）
如果需要卸载 OSQP，可以执行以下命令：
sudo make uninstall

安装 OSQP-Eigen：
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake .. 
make
sudo make install

运行：
cd xxx_ws/
source install/setup.bash 
colcon build
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run xxx xxx （自定义）
