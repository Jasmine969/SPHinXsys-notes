# 安装依赖库

## 安装OpenBLAS

已经有了，略过。

添加环境变量：

```bash
export LD_LIBRARY_PATH=/opt/OpenBLAS/lib:$LD_LIBRARY_PATH
```

## 安装LAPACK-3.8.0

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/lapack-3.8.0 ..
make -j
make install

# 环境变量
export LD_LIBRARY_PATH=/opt/sphinxsys-soft/lapack-3.8.0/lib64:$LD_LIBRARY_PATH
```

## 安装binutils-2.41

安装oneTBB-2022.3.0需要。

```bash
# 编译安装
./configure --prefix=/opt/binutils-2.41
make -j
make install

# 将新汇编器加入 PATH
export PATH=/opt/binutils-2.41/bin:$PATH
export LD_LIBRARY_PATH=/opt/binutils-2.41/lib:$LD_LIBRARY_PATH
```

## 安装oneTBB-2022.3.0

因为我的GCC是13.4版本，启用了更严格的安全检查，而 oneTBB-2022.3.0 的代码可能没有完全适配这些新检查。所以添加额外的编译选项：

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/oneTBB-2022.3.0 \
      -DCMAKE_BUILD_TYPE=RELEASE \
      -DCMAKE_CXX_FLAGS="-Wno-stringop-truncation -Wno-stringop-overflow -Wno-error=stringop-truncation -Wno-error=stringop-overflow" \
      -DCMAKE_C_FLAGS="-Wno-stringop-truncation -Wno-stringop-overflow -Wno-error=stringop-truncation -Wno-error=stringop-overflow" ..
make -j
sudo make install
```

## 安装googletest-1.17.0

```bash
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/googletest-1.17.0 ..
make -j
make install
```

## 安装boost-1.88.0

```bash
./bootstrap.sh --prefix=/opt/sphinxsys-soft/Boost-1.88.0 --with-libraries=all
./b2 install -j40
```

## 安装Simbody-3.7

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/Simbody-3.7 ..
make -j
make install
```

## 安装pybind11

我已经用pip install安装在venv中，可用以下命令获取路径：

```python
import pybind11
print(pybind11.get_include())
```

## eigen-3.4.0

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/eigen-3.4.0 ..
make -j
make install
```

## spdlog-1.16.0

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/sphinxsys-soft/spdlog-1.16.0 ..
make -j
make install
```

# 安装SPHinXsys

配置环境变量（我把这些写在了`/opt/sphinxsys-soft/sphinxsys.env.sh`）：

```bash
export LD_LIBRARY_PATH=/opt/OpenBLAS/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/sphinxsys-soft/lapack-3.8.0/lib64:$LD_LIBRARY_PATH
export TBB_DIR=/opt/sphinxsys-soft/oneTBB-2022.3.0/lib64/cmake/TBB
export GTest_DIR=/opt/sphinxsys-soft/googletest-1.17.0
export Boost_DIR=/opt/sphinxsys-soft/Boost-1.88.0
export Simbody_DIR=/opt/sphinxsys-soft/Simbody-3.7
export pybind11_DIR=/data/zhuh_software/pyenteric/lib/python3.10/site-packages/pybind11
export Eigen3_DIR=/opt/sphinxsys-soft/eigen-3.4.0
export BLAS_DIR=/opt/OpenBLAS
export LAPACK_DIR=/opt/sphinxsys-soft/lapack-3.8.0
export spdlog_DIR=/opt/sphinxsys-soft/spdlog-1.16.0
```

```bash
source /opt/sphinxsys-soft/sphinxsys.env.sh
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_C_COMPILER_LAUNCHER=ccache -D CMAKE_CXX_COMPILER_LAUNCHER=ccache -S ..
make -j40 # 不建议占用全部核心
```

## 测试

```bash
# run the whole test suite
# we are now in the build dir
/usr/local/cmake/bin/ctest -j 1
# test a specific case
cd tests/2d_examples/test_2d_dambreak/bin
./test_2d_dambreak
```

