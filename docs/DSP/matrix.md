# DSP 矩阵模块（matrix.hpp）

## 原理

该模块提供编译期确定维度的矩阵与向量类型，底层运算对接 ARM CMSIS-DSP 库（`arm_mat_*_f32` / `arm_mat_*_f64`），同时通过 CRTP 基类 `base_matrix` 统一提供行列式、逆矩阵、转置、范数等纯 C++ 实现的高层接口。所有维度信息和类型检查均在编译期完成，运行期零开销。

## 核心设计

### CRTP 基类 `base_matrix<Derived, T, Rows, Cols>`

所有矩阵类继承自此基类。基类通过 CRTP 调用派生类的 `*_impl` 函数，实现静态多态，避免虚函数开销。

| 功能 | 方法 | 说明 |
|------|------|------|
| 加法 | `add(other)` / `+` | 同形矩阵逐元素相加 |
| 减法 | `sub(other)` / `-` | 同形矩阵逐元素相减 |
| 矩阵乘法 | `mult(other)` / `*` | `(M×K) * (K×N) → (M×N)` |
| 标量乘法 | `mult(scalar)` / `*` | 矩阵每个元素乘以标量 |
| 行列式 | `det()` | 仅方阵，见下方算法说明 |
| 可逆判断 | `is_invertible()` | `|det| > epsilon` |
| 逆矩阵 | `inverse()` | 仅方阵，底层调用 CMSIS-DSP |
| 转置 | `transpose()` | 返回 `(Cols×Rows)` 新矩阵 |
| 范数 | `norm()` | Frobenius 范数 $\|\mathbf{A}\|_F = \sqrt{\sum a_{ij}^2}$ |
| 归一化 | `normalized()` | 返回 `this / norm()` 的新矩阵（按 Frobenius 范数归一化），`norm() > epsilon` 时执行除法，否则返回原矩阵副本 |
| 单位矩阵 | `identity()` | 静态方法，仅方阵 |
| 元素访问 | `operator[](i, j)` | C++23 多维下标（行，列），零起始 |
| 原始指针 | `get()` | 返回行优先存储的数据指针 |
| CMSIS 句柄 | `get_handle()` | 返回 `arm_matrix_instance_f32/f64` |

### `matrix<T, Rows, Cols>`

具体矩阵类，目前特化了两种浮点类型：

- `matrix<float, Rows, Cols>`：底层调用 `arm_mat_*_f32`
- `matrix<double, Rows, Cols>`：底层调用 `arm_mat_*_f64`

数据以行优先（row-major）顺序存储于栈上数组 `m_data[Rows * Cols]`，零初始化。

### `vector<T, Rows>`

```cpp
template<typename T, std::size_t Rows>
using vector = matrix<T, Rows, 1>;
```

列向量是 `matrix<T, Rows, 1>` 的别名。对于列向量，可使用单下标访问 `v[i]`（等同于 `v[i, 0]`）。

### 行列式算法

| 维度 | 算法 |
|------|------|
| 1×1 | 直接返回元素 |
| 2×2 | $ad - bc$ |
| 3×3 | Sarrus 展开式 |
| N×N (N≥4) | 带部分主元选取的 LU 分解，时间复杂度 $O(N^3)$ |

## 向量运算

```cpp
// 点积（返回标量）
T dot(const vector<T, N>& a, const vector<T, N>& b);
T operator*(const vector<T, N>& a, const vector<T, N>& b);  // 同上

// 叉积（仅 3D 向量）
vector<T, 3> cross(const vector<T, 3>& a, const vector<T, 3>& b);
```

## 3D 变换辅助函数（齐次坐标，4×4）

所有变换函数返回 `matrix<T, 4, 4>`，用于齐次坐标下的仿射变换，可直接右乘 `vector<T, 4>`（最后分量为 `1`）。

| 函数 | 说明 |
|------|------|
| `make_scale<T>(s)` | 均匀缩放，前三个对角元素为 `s`，`[3,3]` 为 `1` |
| `make_translate(vec)` | 平移，基于单位矩阵，`[0..2, 3]` 填入 `vec` 分量 |
| `make_rotate(axis, angle)` | 绕轴 `axis` 旋转 `angle` 弧度（Rodrigues 公式），函数内部自动对 `axis` 归一化 |

变换矩阵组合遵循**右乘**规则：

$$\mathbf{v}' = M_{\text{translate}} \cdot M_{\text{rotate}} \cdot M_{\text{scale}} \cdot \mathbf{v}$$

## 如何使用

### 基本矩阵运算

```cpp
#include "matrix.hpp"

// 构造 3×4 矩阵（行优先初始化列表）
gdut::dsp::matrix<float, 3, 4> a{1, 2, 3, 4,
                                  5, 6, 7, 8,
                                  9,10,11,12};

// 矩阵乘法（编译期验证维度）
gdut::dsp::matrix<float, 4, 2> b{1,2,3,4,5,6,7,8};
auto c = a * b;  // 结果类型：matrix<float, 3, 2>

// 标量乘法
auto d = c * 2.0f;
auto e = 2.0f * c;

// 加减法
auto f = c + d;
auto g = c - d;
```

### 方阵操作

```cpp
auto m = gdut::dsp::matrix<float, 3, 3>{1, 2, 3,
                                         0, 1, 4,
                                         5, 6, 0};

// 行列式
float d = m.det();

// 可逆性检查
if (m.is_invertible()) {
    auto inv = m.inverse();
    auto eye = m * inv;  // 近似单位矩阵
}

// 转置
auto t = m.transpose();  // 类型：matrix<float, 3, 3>

// 单位矩阵
auto i = gdut::dsp::matrix<float, 4, 4>::identity();
```

### 元素访问

```cpp
auto m = gdut::dsp::matrix<float, 2, 3>{1,2,3,4,5,6};

// 二维下标（C++23 多维 operator[]）
float val = m[0, 2];   // 第0行第2列 → 3
m[1, 0] = 10.0f;

// const 访问
const auto& cm = m;
float cval = cm[1, 2];  // → 6
```

### 向量与叉积

```cpp
using vec3f = gdut::dsp::vector<float, 3>;

vec3f a{1.0f, 0.0f, 0.0f};
vec3f b{0.0f, 1.0f, 0.0f};

float dp = gdut::dsp::dot(a, b);   // 0.0f
vec3f cp = gdut::dsp::cross(a, b); // {0, 0, 1}

float norm = a.norm();  // 1.0f
```

### 3D 变换

```cpp
#include <numbers>
using namespace gdut::dsp;

// 缩放 5 倍
auto s = make_scale<float>(5.0f);

// 平移 (1, 2, 3)
auto t = make_translate(vector<float, 3>{1.0f, 2.0f, 3.0f});

// 绕 Y 轴旋转 π/2 弧度（轴向量无需预先归一化）
auto r = make_rotate(vector<float, 3>{0.0f, 1.0f, 0.0f},
                     std::numbers::pi_v<float> / 2.0f);

// 组合变换：先缩放，再旋转，再平移
vector<float, 4> v{1.0f, 1.0f, 1.0f, 1.0f};
auto result = t * r * s * v;
```

### 与 CMSIS-DSP 互操作

```cpp
auto m = gdut::dsp::matrix<float, 3, 3>::identity();

// 获取 arm_matrix_instance_f32，可传入其他 CMSIS 函数
auto handle = m.get_handle();
// handle.pData 指向内部数组，handle.numRows/numCols 已设置

// 获取原始数据指针
float* data = m.get();
```

### 使用 double 精度

```cpp
auto dm = gdut::dsp::matrix<double, 4, 4>::identity();
auto dinv = dm.inverse();  // 调用 arm_mat_inverse_f64
```

## 实现注意事项

- **支持类型**：目前仅特化 `float` 和 `double`。对其他类型实例化会得到空的 `matrix<T, R, C>` 类（无任何成员）。
- **维度检查**：矩阵乘法 `operator*` 在编译期通过 `static_assert` 检查 `A.cols == B.rows` 及两矩阵元素类型一致。
- **内存布局**：行优先（row-major）存储，与 CMSIS-DSP 约定一致。
- **`make_rotate` 轴向量自动归一化**：函数内部会调用 `normalized()` 对传入的 `axis` 进行归一化，因此传入任意非零方向向量均可，但传入零向量将导致除以零（结果未定义）。
- **`inverse()` 精度**：`float` 精度较低，建议在精度敏感场景使用 `double` 版本。
- **`det()` for N≥4**：使用带部分主元的 LU 分解，当主元绝对值 ≤ `epsilon` 时提前返回 `0`（奇异矩阵）。

## 类型特征工具

模块内提供以下辅助元函数，可用于泛型约束：

```cpp
// 检查类型是否为标量（整型或浮点型）
gdut::dsp::is_scalar_v<T>

// 检查类型是否为 matrix 家族
gdut::dsp::is_matrix_v<Mat>

// 将 matrix<T, R, C> 的维度替换为 (NewR, NewC)
gdut::dsp::redefine_matrix_t<Mat, NewRows, NewCols>
```

相关源码：[Middlewares/GDUT_RC_Library/DSP/matrix.hpp](../../Middlewares/GDUT_RC_Library/DSP/matrix.hpp)
