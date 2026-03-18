# BSP GPIO 引脚模块（bsp_gpio_pin.hpp）

## 原理
该模块对 STM32 HAL 的 GPIO 初始化与操作进行编译期封装，通过模板参数在编译期确定端口与配置，运行期只执行必要的 HAL 调用。

## 实现思想
- `gpio_pin_tag` 作为编译期配置标签，保存端口与初始化结构体。
- `gpio_pin` 在构造时调用 `HAL_GPIO_Init`，析构时自动 `HAL_GPIO_DeInit`。
- 通过 `bsp_type_traits` 中的端口枚举和转换函数保证类型安全。

## 如何使用

```cpp
#include "bsp_gpio_pin.hpp"

// 定义 LED 引脚（以 PA5 为例）
using led_pin = gdut::gpio_pin<gdut::gpio_port::A,
                                GPIO_InitTypeDef{.Pin = GPIO_PIN_5,
                                                .Mode = GPIO_MODE_OUTPUT_PP,
                                                .Pull = GPIO_NOPULL,
                                                .Speed = GPIO_SPEED_FREQ_LOW}>;

void led_task() {
    led_pin led; // 构造时初始化 GPIO

    led.write(true);
    led.toggle();
    bool state = led.read();
    (void)state;
} // 退出作用域自动反初始化 GPIO
```

## 与代码规范的对应
- 采用强类型枚举 `gpio_port`，减少魔法数。
- RAII 方式自动初始化与反初始化，符合资源管理规范。
- 命名采用蛇形与 `m_` 前缀风格。

## 注意事项/坑点
- 使用前必须确保对应 GPIO 时钟已开启。
- `GPIO_InitTypeDef` 需完整初始化，避免未定义字段。
- 对象析构会反初始化 GPIO，注意生命周期不要短于使用范围。

相关源码：[Middlewares/GDUT_RC_Library/BSP/bsp_gpio_pin.hpp](../../Middlewares/GDUT_RC_Library/BSP/bsp_gpio_pin.hpp)
