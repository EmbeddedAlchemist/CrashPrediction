# Read Me

## BRIEF

这是“路侧运算中心”的源码。



## 编译

- **开发环境：** 
  - 集成开发环境： Visual Studio 2022
  - 编译器： MSVC
  - C++标准：ISO C++17
- **外部依赖库**
  - Box2D

> 备注：开发时使用了vcpkg包管理器来安装Box2D库，若希望编译运行，您需要正确安装vcpkg，并编译安装Box2D库，之后可能需要在 Visual Studio 中配置vcpkg路径。

## 说明

程序中目前使用0.0.0.0:8080作为数据的获取和回报接口，使用0.0.0.0:8081作为物理引擎的调试绘图接口。

你可以在main.cpp中更改。

