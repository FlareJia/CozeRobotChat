# CozeRobotChat - 智能语音对话机器人

## 📖 项目简介

CozeRobotChat 是一个基于 Coze API 的智能语音对话机器人系统，支持语音唤醒、实时对话、流式响应和音频播放等功能。该项目采用模块化架构设计，支持依赖注入和多环境配置，适用于智能助手、客服机器人等应用场景。

## ✨ 核心功能

### 🎤 语音交互
- **语音唤醒**：支持自定义唤醒词检测
- **语音识别**：实时语音转文字功能
- **语音合成**：文字转语音播放
- **音频管理**：支持多种音频格式和播放控制

### 🤖 智能对话
- **Coze API 集成**：基于先进的 AI 对话模型
- **流式响应**：支持实时流式对话体验
- **上下文管理**：维护对话上下文和状态
- **多Bot支持**：可配置不同的对话机器人

### 🏗️ 系统架构
- **依赖注入**：基于接口的松耦合设计
- **模块化架构**：清晰的分层结构
- **配置管理**：支持多环境配置
- **错误处理**：完善的异常处理机制

### 🎮 交互方式
- **键盘控制**：支持快捷键操作
- **语音控制**：语音唤醒和结束
- **ROS集成**：支持机器人操作系统

## 🚀 快速开始

### 环境要求

- Python 3.8+
- macOS/Linux (推荐)
- 音频设备（麦克风和扬声器）

### 安装步骤

1. **克隆项目**
```bash
git clone <repository-url>
cd CozeRobotChat
```

2. **安装依赖**
```bash
# 使用提供的安装脚本
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh

# 或手动安装
pip install -r requirements.txt
```

3. **配置环境变量**
```bash
# 复制环境变量模板
cp .env.example .env

# 编辑配置文件
vim .env
```

4. **配置 Coze API**
在 `.env` 文件中设置：
```env
BEARER_TOKEN=your_coze_api_token
BOT_ID=your_bot_id
ENVIRONMENT=development
```

### 运行应用

```bash
# 赋予执行权限
chmod +x main.py

# 启动应用
python main.py

# 或直接运行
python app.py
```

## 📁 项目结构

```
CozeRobotChat/
├── app.py                 # 应用主入口
├── main.py               # 简化入口文件
├── config/               # 配置模块
│   ├── __init__.py
│   ├── environment.py    # 环境配置
│   ├── api_config.py     # API配置
│   ├── audio_config.py   # 音频配置
│   ├── di_config.py      # 依赖注入配置
│   ├── feature_config.py # 功能配置
│   └── path_config.py    # 路径配置
├── core/                 # 核心模块
│   ├── interfaces/       # 接口定义
│   ├── di_container.py   # 依赖注入容器
│   ├── service_registry.py # 服务注册
│   ├── conversation_manager.py # 对话管理
│   ├── streaming_processor.py # 流式处理
│   └── wake_word_detector.py # 唤醒词检测
├── services/             # 服务层
│   ├── api_client.py     # API客户端
│   ├── audio_service.py  # 音频服务
│   ├── chat_processor.py # 聊天处理
│   ├── error_handler.py  # 错误处理
│   ├── keyboard_service2.py # 键盘服务
│   └── streaming/        # 流式处理模块
├── managers/             # 管理器
│   ├── audio_manager.py  # 音频文件管理
│   └── resource_manager.py # 资源管理
├── hardware/             # 硬件接口
│   └── audio_interface.py # 音频硬件接口
├── utils/                # 工具模块
├── tests/                # 测试文件
├── outputs/              # 输出文件
├── records/              # 录音文件
└── ros_ws/               # ROS工作空间
```

## 🎯 使用指南

### 基本操作

1. **启动应用**：运行 `python3 main.py`
2. **语音唤醒**：说出配置的唤醒词（默认："你好，伯乐"）
3. **开始对话**：唤醒后直接说话进行对话
4. **结束对话**：说出结束词（默认："再见，伯乐"）或按 `Ctrl+C`

### 键盘快捷键

- `Ctrl+1/2`：播放预设音频
- `Alt+1/2/3/4`：播放预设音频
- `Ctrl+C`：退出应用

### 配置说明

主要配置文件位于 `config/` 目录：

- **环境配置**：`environment.py` - 设置运行环境
- **API配置**：`api_config.py` - Coze API相关配置
- **音频配置**：`audio_config.py` - 音频参数和文件路径
- **功能配置**：`feature_config.py` - 功能开关

## 🔧 开发指南

### 架构设计

项目采用分层架构和依赖注入模式：

1. **接口层**：`core/interfaces/` - 定义系统接口
2. **核心层**：`core/` - 核心业务逻辑
3. **服务层**：`services/` - 具体服务实现
4. **管理层**：`managers/` - 资源和状态管理
5. **硬件层**：`hardware/` - 硬件接口实现

### 扩展开发

1. **添加新服务**：实现对应接口并注册到 DI 容器
2. **自定义音频处理**：继承 `IAudioDevice` 接口
3. **集成新的 AI 模型**：实现 `IAPIClient` 接口
4. **添加新的交互方式**：扩展输入处理模块

### 测试

```bash
# 运行所有测试
python tests/run_tests.py

# 运行特定测试
python -m pytest tests/test_service_registry.py -v
```

## 📋 依赖说明

### 核心依赖
- `requests` - HTTP请求库
- `pyaudio` - 音频处理
- `python-dotenv` - 环境变量管理
- `retrying` - 重试机制
- `apscheduler` - 定时任务

### 系统依赖
- `portaudio` - 音频底层库
- `filelock` - 文件锁
- `psutil` - 系统工具
- `playsound` - 音频播放

## 🚨 常见问题

### 音频问题

**Q: 无法录音或播放音频**
A: 检查音频设备权限，确保麦克风和扬声器正常工作

**Q: pyaudio 安装失败**
A: 先安装 portaudio：`brew install portaudio`（macOS）

### API问题

**Q: Coze API 调用失败**
A: 检查 BEARER_TOKEN 和 BOT_ID 配置是否正确

**Q: 网络连接超时**
A: 检查网络连接，可能需要配置代理

### 运行问题

**Q: 依赖注入错误**
A: 检查服务注册配置，确保所有接口都有对应实现

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支：`git checkout -b feature/new-feature`
3. 提交更改：`git commit -am 'Add new feature'`
4. 推送分支：`git push origin feature/new-feature`
5. 提交 Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 📞 联系方式

如有问题或建议，请通过以下方式联系：

- 提交 Issue
- 发送邮件
- 项目讨论区

---

**注意**：使用前请确保已正确配置 Coze API 密钥和相关权限。
