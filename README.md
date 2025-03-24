# OSQP 与 OSQP-Eigen 安装指南 ✨

本仓库记录了如何安装 [OSQP](https://github.com/oxfordcontrol/osqp) 以及它的 C++ 封装器 [osqp-eigen](https://github.com/robotology/osqp-eigen)。

> 📌 **注意：**由于 `osqp-eigen` 对新版本 OSQP 的支持存在延迟，本教程安装的是 OSQP 的旧版本 `release-0.6.3`。

---

## 🧩 安装 OSQP

```bash
# 克隆 OSQP 源码（指定旧版本）
git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git

# 编译并安装
cd osqp
mkdir build && cd build
cmake ..
make
sudo make install
