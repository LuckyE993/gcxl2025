# gcxl2025

这是一个Python应用程序的标准项目模板。

## 项目结构

```
gcxl2025/
│
├── src/                    # 源代码目录
│   ├── __init__.py         # 使src成为一个Python包
│   ├── main.py             # 应用程序入口点
│   ├── core/               # 核心功能模块
│   │   └── __init__.py
│   ├── utils/              # 实用工具函数
│   │   └── __init__.py
│   └── config/             # 配置文件和配置加载逻辑
│       └── __init__.py
│
├── tests/                  # 测试目录
│   ├── __init__.py
│   ├── test_main.py        # 主模块测试
│   ├── unit/               # 单元测试
│   │   └── __init__.py
│   └── integration/        # 集成测试
│       └── __init__.py
│
├── docs/                   # 文档目录
│   └── README.md           # 详细文档
│
├── data/                   # 数据文件目录
│   ├── input/              # 输入数据
│   └── output/             # 输出数据
│
├── scripts/                # 部署、构建和实用脚本
│   └── setup.sh
│
├── requirements.txt        # 项目依赖
├── setup.py                # 包安装和分发设置
├── .gitignore              # Git忽略文件列表
├── .env.example            # 环境变量示例文件
├── README.md               # 项目说明文档（本文件）
└── LICENSE                 # 许可证文件
```

## 文件夹功能描述

- **src/**: 包含所有源代码，采用模块化结构组织
  - **core/**: 应用程序的核心功能和业务逻辑
  - **utils/**: 通用工具函数和帮助类
  - **config/**: 配置文件处理和应用程序配置管理

- **tests/**: 包含所有测试文件
  - **unit/**: 单元测试，测试单个函数和类
  - **integration/**: 集成测试，测试组件之间的交互

- **docs/**: 详细的项目文档，包括API文档、使用指南等

- **data/**: 应用程序使用的数据文件
  - **input/**: 输入数据存储
  - **output/**: 输出数据存储

- **scripts/**: 用于自动化部署、构建和其他操作的脚本

## 安装和使用

1. 克隆仓库
2. 创建并激活虚拟环境（推荐）：
   ```bash
   python -m venv venv
   # Windows
   venv\Scripts\activate
   # Linux/MacOS
   source venv/bin/activate
   ```
3. 安装依赖：`pip install -r requirements.txt`
4. 运行应用的几种方式：

   - 作为模块运行主程序：
     ```bash
     python -m src.main
     ```
   
   - 如果您已安装此包（通过 `pip install -e .`），可以直接运行：
     ```bash
     python src/main.py
     ```
   
   - 如果项目已配置为可执行包，可能还可以使用：
     ```bash
     gcxl2025
     ```

5. 运行测试：
   ```bash
   pytest
   # 或者运行特定测试
   pytest tests/unit/
   pytest tests/integration/
   ```

## 开发指南

- 遵循PEP 8编码规范
- 为所有功能编写测试
- 在提交代码前运行测试：`pytest`