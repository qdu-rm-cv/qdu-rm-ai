# Style Guide

编码风格主要参考Google C++ 风格指南([中文版](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/)/[英文原版](https://google.github.io/styleguide/cppguide.html))，并在此基础上添加了部分战队特有风格

本文件为对以上材料的补充和改写如果本文件与参考指南冲突，则以本文件为准

所有不符合此规范的代码都不能提交，鼓励对已有的不规范的代码进行更改

- 使用C++14作为标准，部分情况下允许使用C++17
- 提交前**必须**使用Google风格的`clang-formatter`进行格式化
- 如果使用VS Code编辑代码，建议打开`Format On Save`

## 头文件

### #pragma once 保护

所有头文件都应该使用 `#pragma once` 来防止头文件被多重包含.

### #include的顺序

1. 本源代码文件对应的头文件本文件为`armor.cpp`则对应 `armor.hpp`

2. C++标准库，需要使用`#include <>`

3. 其他文件夹内或本文件夹内的 `.hpp` 文件，使用`#include ""`，按字典序排序

## 面向对象编程

- 所有的代码均运用面向对象的编程思维

- 数据类型：
  类和结构体分别使用，具有抽象意义的使用自定义类作为基础类型；
  仅储存数据使用的数据类型，使用结构体

- 清楚分析 `public` 、 `private` 和 `protected` 的区别

- 不使用 `using namespace` 的用法

## 函数

### 参数

1. 函数的参数应该尽可能地用`const`保护起来，并且使用引用`&`防止意外修改

2. 应以`const`类型作为函数数据类型

## 命名约定

1. 函数命名方式

   - 大驼峰命名法
   - 常见的缩略词首字母大写，其余均小写，如`Usb`

2. 文件命名方式

   - 全小写的下划线命名法
   - 头文件使用`.hpp`作为扩展名，源文件使用`.cpp`作为扩展名，特殊情况下使用`.cc`

3. 变量命名方式

   1. 统一使用全小写下划线命名方式

      - 使用全小写和下划线组合的方式，如`temp_var`
      - 常见的缩略词首字母大写，其余均小写，如`var_Usb`

   2. 类的内部成员变量需要在一般变量的基础上，在变量名后面加`_`，如：

      ```C++
      class Armor{
       private:
        Euler euler_;
      };
      ```

   3. 常量在一般变量的基础上，原名称变为全大写拼写，并在前面加`k`

      1. 单独文件使用的常量使用匿名空间保护，如：

         ```C++
         namespace{
         const double kARMOR_WIDTH = 125.;
         }
         ```

      2. `Enum`内变量名同常量对待

   4. 宏命名

      - 全大写和下划线

4. 类命名

   - 大驼峰命名法
   - 常见的缩略词首字母大写，其余均小写，如`Usb`

5. 函数命名

   - 大驼峰命名法
   - 常见的缩略词首字母大写，其余均小写，如`FindUsb`

6. 作用域

   - 有名`namespace` 不使用缩略名

## 注释

使用`/* 内容 */`格式的注释(注意空格)

## 格式

类的内部函数成员，排序按照 `private` 到 `public` 排序，按照语义到字典的优先级排序，并且在 `.hpp` 和  `.cpp` 顺序对应

## 其他

``` C++
/* 当有代码不需要起作用但要保留时，请使用以下方法 */
#if 0 // or #ifndef or #ifdef

FunctionOne();
FunctionTwo();

#endif
```
