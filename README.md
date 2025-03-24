# LMPC
安装osqp、及其c++包装器osqp-eigen
安装osqp（由于osqp-eigen更新较慢，下载osqp源码时要指定老版本）
git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
cd osqp
mkdir build && cd build
cmake .. 
make
sudo make install
测试
cd ~/osqp/build/out
./osqp_demo
卸载
sudo make uninstall
安装osqp-eigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake .. 
make
sudo make install
