# 🌿 OSQP 与 OSQP-Eigen 安装指南

本仓库记录了如何在 Linux 系统中安装 [OSQP](https://github.com/oxfordcontrol/osqp) 和它的 C++ 封装器 [osqp-eigen](https://github.com/robotology/osqp-eigen)。

> 📌 **温馨提示：** `osqp-eigen` 目前与 OSQP 的较新版本可能不兼容。为避免安装失败，我们建议使用 `OSQP release-0.6.3` 版本。

---

## 📦 安装 OSQP

### 🔧 步骤一：克隆并切换版本

```bash
git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
🔧 步骤二：构建并安装
bash
复制
编辑
cd osqp
mkdir build && cd build
cmake ..
make
sudo make install
✅ 测试 OSQP 是否安装成功
bash
复制
编辑
cd ~/osqp/build/out
./osqp_demo
你应该会看到一些输出结果，表示一个小的二次规划问题被成功求解。

🧼 卸载 OSQP（可选）
如果需要卸载 OSQP，可以执行以下命令：

bash
复制
编辑
sudo make uninstall
⚙️ 安装 OSQP-Eigen
🔧 步骤一：克隆源码
bash
复制
编辑
git clone https://github.com/robotology/osqp-eigen.git
🔧 步骤二：构建并安装
bash
复制
编辑
cd osqp-eigen
mkdir build && cd build
cmake ..
make
sudo make install
