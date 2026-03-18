# VS Code 开发环境配置指南

本文档详细说明如何配置VS Code开发环境，包括OpenOCD调试器的安装和环境变量设置。

## 目录
- [VS Code 扩展与插件](#vs-code-扩展与插件)
- [OpenOCD 安装](#openocd-安装)
- [环境变量配置](#环境变量配置)
- [VS Code 调试配置](#vs-code-调试配置)
- [常见问题](#常见问题)

---

## VS Code 扩展与插件

### 必需的扩展

为了获得完整的 STM32 开发体验，需要安装以下 VS Code 扩展：

| 扩展 | 功能 | 下载链接 |
|------|------|--------|
| **C/C++** | C/C++ 代码编辑、调试 | [Microsoft C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) |
| **Cortex-Debug** | ARM Cortex-M 调试工具 | [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) |
| **CMake** | CMake 构建系统支持 | [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake) |
| **CMake Tools** | CMake 集成工具 | [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) |
| **Clang-Format** | 代码格式化工具 | [Clang-Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) |

### 安装方法

#### 方法一：VS Code 扩展市场（推荐）

1. 打开 VS Code
2. 按 `Ctrl + Shift + X` 打开扩展市场
3. 搜索上表中的扩展名
4. 点击 "Install" 按钮安装

#### 方法二：命令行安装

```powershell
# 安装 C/C++ 扩展
code --install-extension ms-vscode.cpptools

# 安装 Cortex-Debug 扩展
code --install-extension marus25.cortex-debug

# 安装 CMake 扩展
code --install-extension twxs.cmake

# 安装 CMake Tools 扩展
code --install-extension ms-vscode.cmake-tools

# 安装 Clang-Format 扩展
code --install-extension xaver.clang-format
```

### Cortex-Debug 插件详细说明

**Cortex-Debug** 是专为 ARM Cortex-M 微控制器设计的调试工具，提供更强大的硬件调试功能。

#### 核心功能

- 🎯 **SVD 文件支持**：寄存器窗口显示外设和寄存器信息
- 📊 **变量和内存查看**：实时查看变量值和内存内容
- 🔍 **条件断点**：设置复杂的断点条件
- 📈 **性能分析**：用于代码性能优化
- 🛠️ **外设模拟**：模拟硬件外设行为

#### 与本项目的集成

本项目已在 `debug/` 文件夹提供了 Cortex-Debug 的配置：

- **launch.json** - 如果使用 Cortex-Debug，需要在配置中指定：
  ```json
  {
      "name": "STM32 Debug",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/build/robocon11-second-group-framework.elf",
      "svdFile": "${workspaceFolder}/debug/stm32f407.svd",
      "device": "STM32F407VE",
      "interface": "swd",
      "configFiles": [
          "interface/stlink.cfg",
          "target/stm32f4x.cfg"
      ]
  }
  ```

#### 配置步骤

1. **安装 Cortex-Debug 扩展**（见上方安装方法）

2. **确保 OpenOCD 已安装**（见下方 OpenOCD 安装段落）

3. **使用 SVD 文件**（可选但推荐）
   - 项目已包含 `debug/stm32f407.svd` 文件
   - 该文件提供 STM32F407 的外设定义
   - 在调试时可以查看和修改寄存器值

4. **修改 launch.json**
   - 如使用 Cortex-Debug，需将上述配置添加到 `.vscode/launch.json`
   - 确保 `executable` 路径正确指向编译后的 ELF 文件

#### 使用 Cortex-Debug 调试

1. **启动调试**
   - 按 `Ctrl + Shift + D` 打开调试面板
   - 从下拉菜单选择 "Cortex Debug"
   - 按 `F5` 或点击 "Start Debugging" 按钮

2. **查看外设寄存器**
   - 调试时在左侧 "Cortex-Debug" 面板中可查看
   - 展开 "Peripherals" 部分查看 GPIO、UART 等外设

3. **查看内存**
   - 在调试面板中右键选择 "View Memory"
   - 输入地址查看特定内存位置的值

---

## OpenOCD 安装

### 什么是 OpenOCD？
OpenOCD (Open On-Chip Debugger) 是一个开源的调试工具，用于与STM32等微控制器通信，支持在线编程和调试。

### Windows 下载和安装

#### 第一步：下载 OpenOCD
1. 打开浏览器访问：https://github.com/openocd-org/openocd/releases
2. 找到最新稳定版本（通常标记为 `Latest release`）
3. 下载 Windows 预编译版本，文件名通常为：
   - `OpenOCD-[版本号]-WIN64.zip` (64位)
   - `OpenOCD-[版本号]-WIN32.zip` (32位)
   
   **推荐下载 64 位版本**

#### 第二步：解压文件
1. 右键点击下载的 `.zip` 文件
2. 选择"解压到..."
3. **建议解压路径**：`C:\OpenOCD` 或 `D:\tools\OpenOCD`
   
   **注意**：避免路径中含有中文和空格

#### 第三步：验证安装
打开 PowerShell（Windows键 + X，选择 Windows PowerShell），执行：
```powershell
C:\OpenOCD\bin\openocd.exe --version
```
如果显示版本号，说明安装成功。

---

## 环境变量配置

### 什么是环境变量？
环境变量是系统级或用户级的配置，可以让你在任何位置通过命令行直接运行程序，而不需要输入完整路径。

### Windows 环境变量设置步骤

#### 方法一：图形界面（推荐）

1. **打开环境变量编辑器**
   - 按 `Windows 键 + R`
   - 输入 `sysdm.cpl`，按 Enter
   - 点击"高级"标签页
   - 点击"环境变量"按钮

2. **添加用户环境变量**
   - 在"用户变量"框中，点击"新建"
   - 变量名：`OPENOCD_HOME`
   - 变量值：`C:\OpenOCD`（根据实际安装路径修改）
   - 点击"确定"

3. **修改 PATH 环境变量**
   - 在"用户变量"中找到 `Path`（没有则新建）
   - 点击"编辑"
   - 点击"新建"，输入：`%OPENOCD_HOME%\bin`
   - 或直接输入：`C:\OpenOCD\bin`（根据实际路径修改）
   - 点击"确定"

4. **应用更改**
   - 点击"确定"关闭所有窗口
   - **重启 VS Code 或 PowerShell** 使更改生效

#### 方法二：PowerShell 命令行（高级用户）

```powershell
# 设置 OPENOCD_HOME 环境变量
[Environment]::SetEnvironmentVariable("OPENOCD_HOME", "C:\OpenOCD", "User")

# 添加到 PATH（如果还没有）
$currentPath = [Environment]::GetEnvironmentVariable("Path", "User")
if ($currentPath -notlike "*OpenOCD*") {
    $newPath = $currentPath + ";C:\OpenOCD\bin"
    [Environment]::SetEnvironmentVariable("Path", $newPath, "User")
}
```

### 验证环境变量设置

在 PowerShell 中执行：
```powershell
# 检查 OPENOCD_HOME 变量
Write-Host $env:OPENOCD_HOME

# 直接运行 openocd
openocd.exe --version
```

如果两条命令都成功执行，说明环境变量配置正确。

---

## VS Code 调试配置

### 项目的配置模板

本项目在 `debug/` 文件夹中提供了标准的 VS Code 配置模板，包括：

| 文件 | 用途 |
|------|------|
| [debug/launch.json](../debug/launch.json) | GDB 调试启动配置 |
| [debug/settings.json](../debug/settings.json) | VS Code 编辑器设置和扩展配置 |
| [debug/stm32f407.svd](../debug/stm32f407.svd) | STM32F407 芯片的调试符号文件 |

### 使用配置模板

1. **首次设置**：将 `debug/` 文件夹中的配置文件复制到 `.vscode/` 目录：
   ```powershell
   # 如果还没有 .vscode 文件夹，先创建
   New-Item -ItemType Directory -Path .vscode -Force
   
   # 复制配置文件
   Copy-Item -Path debug/launch.json -Destination .vscode/launch.json -Force
   Copy-Item -Path debug/settings.json -Destination .vscode/settings.json -Force
   ```

2. **检查配置**：打开 `.vscode/launch.json`，检查以下参数是否正确：
   - `executable`: ELF 文件路径
   - `servertype`: 调试服务器类型（应为 `openocd`）
   - `configFiles`: OpenOCD 配置文件路径
   - `runToEntryPoint`: 启动后停止的入口函数

### launch.json 关键配置说明

**主要配置参数**：

| 参数 | 说明 | 示例值 |
|------|------|--------|
| `name` | 配置名称，显示在调试菜单中 | `STM32 Debug` |
| `type` | 调试器类型，固定为 Cortex-Debug 插件 | `cortex-debug` |
| `servertype` | 调试服务器类型 | `openocd` |
| `executable` | ELF 文件路径（编译后生成） | `${workspaceFolder}/build/robocon11-second-group-framework.elf` |
| `svdFile` | SVD 文件路径，提供外设寄存器视图 | `${workspaceFolder}/debug/stm32f407.svd` |
| `configFiles` | OpenOCD 配置文件（调试器 + 目标芯片） | `["interface/stlink.cfg", "target/stm32f4x.cfg"]` |
| `runToEntryPoint` | 启动后自动运行到指定函数 | `main` |

具体的配置内容请查看 [debug/launch.json](../debug/launch.json) 文件。

### settings.json 编辑器配置

[debug/settings.json](../debug/settings.json) 目前主要包含对 C/C++ 语言服务器（`clangd`）的参数配置（例如 `clangd.arguments`），用于优化代码补全和跳转体验。

你可以在此基础上根据个人需求，手动补充 clang-format、Pylance、文件关联等其他 VS Code 设置。

使用此配置可获得更好的代码提示和编辑体验。

### 调试工作流

#### 第一次调试前的准备

1. **确保 `.vscode/` 文件夹存在**：若不存在，需将 `debug/` 文件夹中的文件复制到 `.vscode/`

2. **构建项目**：
   ```powershell
   cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
   cmake --build build
   ```

3. **启动 OpenOCD 调试服务器**（在终端中）：
   ```powershell
   openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
   ```
   OpenOCD 应显示 "Info : Listening on port 3333 for gdb connections"。

#### 启动调试

1. **打开调试面板**：按 `Ctrl + Shift + D`
2. **选择配置**：从下拉菜单选择 "Debug with OpenOCD"
3. **启动调试**：点击绿色 "▶ Start Debugging" 按钮或按 `F5`
4. **GDB 应连接成功**：若显示 "hit breakpoint"，说明调试正常

#### 使用调试工具

| 功能 | 快捷键 | 说明 |
|------|--------|------|
| 继续执行 | F5 | 运行到下一个断点 |
| 单步执行 | F10 | 执行当前行（不进入函数） |
| 单步进入 | F11 | 进入函数内部执行 |
| 单步返回 | Shift + F11 | 执行到函数返回 |
| 添加断点 | 点击行号左侧 | 在代码行处添加/移除断点 |
| 查看变量 | 左侧 "Variables" 面板 | 实时查看变量值 |
| 查看调用栈 | 左侧 "Call Stack" 面板 | 跟踪函数调用关系 |

---

## OpenOCD 配置文件

OpenOCD 需要配置文件来识别你的开发板和调试器。

### 常见的配置文件位置
```
C:\OpenOCD\share\openocd\scripts\
├── board/               # 开发板配置
├── interface/           # 调试器接口配置
└── target/              # 目标芯片配置
```

### STM32F407 的典型配置

**interface/stlink.cfg** - 用于 ST-Link 调试器
**target/stm32f4x.cfg** - 用于 STM32F4 系列芯片

### 创建项目级配置文件（可选）

在项目根目录创建 `openocd.cfg`：

```cfg
# OpenOCD 配置文件示例

# 1. 指定调试器接口
source [find interface/stlink.cfg]

# 2. 指定目标芯片
source [find target/stm32f4x.cfg]

# 3. 连接参数
set WORKAREASIZE 0x50000
set FLASH_WRITESIZE 256
```

使用此配置启动 OpenOCD：
```powershell
openocd -f openocd.cfg
```

---

## 常见问题

### Q1: VS Code 扩展无法安装？
**A**:
- 确保网络连接正常
- 尝试切换 VS Code 扩展市场源（设置中的 Extension Gallery）
- 可使用命令行安装：`code --install-extension <extension-id>`
- 重启 VS Code 后重试

### Q2: Cortex-Debug 无法连接调试器？
**A**:
- 确认已安装 OpenOCD（见上方 OpenOCD 安装段落）
- 检查 `launch.json` 中的 `servertype` 是否设置为 `openocd`
- 启动 OpenOCD 服务器：`openocd -f interface/stlink.cfg -f target/stm32f4x.cfg`
- 查看 OpenOCD 是否显示 "Listening on port 3333"
- 检查 USB 调试器是否正确连接

### Q3: OpenOCD 命令找不到？
**A**: 
- 确认已添加环境变量到 PATH
- 重启 VS Code 和 PowerShell
- 在 PowerShell 中执行：`refreshenv` 刷新环境变量（在 Windows 10 之前可能需要）

### Q4: 连接调试器失败？
**A**:
- 确保 USB 调试器已连接到电脑
- 检查设备管理器中是否识别了设备
- 确认使用了正确的配置文件（`interface` 和 `target`）
- 尝试在命令行直接运行 `openocd -f openocd.cfg` 排查问题

### Q5: 编译失败说找不到工具链？
**A**:
- 确认已安装 GCC ARM Embedded Toolchain
- 检查 CMake 配置文件中的工具链路径是否正确
- 执行 `cmake --build build --target clean` 清除缓存后重新编译

### Q6: 调试时 GDB 连接超时？
**A**:
- 确保 OpenOCD 已启动且运行正常
- 检查防火墙是否阻止了 GDB 端口（默认为 3333）
- 尝试在 VS Code 中增加超时时间：
  ```json
  "stopAtEntry": true,
  "timeout": 60000
  ```

---

## 快速参考命令

| 操作 | 命令 |
|------|------|
| 验证 OpenOCD 安装 | `openocd.exe --version` |
| 启动 OpenOCD（使用配置文件） | `openocd -f openocd.cfg` |
| 启动 OpenOCD（ST-Link + STM32F4） | `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg` |
| 构建项目 | `cmake --build build` |
| 清除构建文件 | `cmake --build build --target clean` |
| 格式化代码 | `.\format_all.ps1` |

---

## 相关文档和文件

### 文档
- [README.md](../README.md) - 项目总体说明和构建指南
- [代码规范.md](./代码规范.md) - 编码规范和开发规范

### VS Code 配置模板（在 debug/ 目录）
- [debug/launch.json](../debug/launch.json) - GDB 调试启动配置
- [debug/settings.json](../debug/settings.json) - 编辑器设置和扩展配置
- [debug/stm32f407.svd](../debug/stm32f407.svd) - STM32F407 调试符号定义

### CMake 相关
- [CMakeLists.txt](../CMakeLists.txt) - 项目构建配置
- [cmake/gcc-arm-none-eabi.cmake](../cmake/gcc-arm-none-eabi.cmake) - ARM 工具链配置

---

**最后更新**：2026年2月
**维护者**：开发团队
