# 开发者指南

## 概述

本文档为 CozeRobotChat 项目的开发者提供详细的开发指南，包括环境搭建、代码规范、测试指南、贡献流程等内容。

## 开发环境搭建

### 系统要求

- **操作系统**: macOS 10.15+, Ubuntu 18.04+, Windows 10+
- **Python版本**: 3.8+
- **内存**: 最少 4GB RAM
- **存储**: 最少 2GB 可用空间
- **音频设备**: 麦克风和扬声器（用于语音交互）

### 环境准备

#### 1. 克隆项目

```bash
git clone https://github.com/your-username/CozeRobotChat.git
cd CozeRobotChat
```

#### 2. 创建虚拟环境

```bash
# 使用 venv
python -m venv venv

# 激活虚拟环境
# macOS/Linux
source venv/bin/activate
# Windows
venv\Scripts\activate
```

#### 3. 安装依赖

```bash
# 安装生产依赖
pip install -r requirements.txt

# 安装开发依赖（可选）
pip install -r requirements-dev.txt
```

#### 4. 配置环境变量

创建 `.env` 文件：

```bash
cp .env.example .env
```

编辑 `.env` 文件，填入必要的配置：

```env
# API配置
BEARER_TOKEN=your_coze_api_token
BOT_ID=your_bot_id
USER_ID=your_user_id

# 环境配置
ENVIRONMENT=development

# 音频配置
VOICE_ID=your_voice_id

# 日志配置
LOG_LEVEL=DEBUG
```

#### 5. 验证安装

```bash
# 运行测试
python -m pytest tests/ -v

# 运行应用
python main.py
```

### IDE配置

#### PyCharm配置

1. **项目解释器设置**
   - File → Settings → Project → Python Interpreter
   - 选择虚拟环境中的Python解释器

2. **代码风格配置**
   - File → Settings → Editor → Code Style → Python
   - 设置缩进为4个空格
   - 启用PEP 8代码检查

3. **运行配置**
   - Run → Edit Configurations
   - 添加Python配置，脚本路径设为 `main.py`
   - 设置工作目录为项目根目录

#### VS Code配置

创建 `.vscode/settings.json`：

```json
{
    "python.defaultInterpreterPath": "./venv/bin/python",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "python.formatting.provider": "black",
    "python.testing.pytestEnabled": true,
    "python.testing.pytestArgs": [
        "tests"
    ],
    "files.exclude": {
        "**/__pycache__": true,
        "**/*.pyc": true
    }
}
```

## 项目结构详解

```
CozeRobotChat/
├── app.py                    # 应用主入口
├── main.py                   # 简化启动入口
├── requirements.txt          # 生产依赖
├── requirements-dev.txt      # 开发依赖
├── .env                      # 环境变量配置
├── .env.example             # 环境变量模板
├── .gitignore               # Git忽略文件
├── README.md                # 项目说明
│
├── config/                  # 配置模块
│   ├── __init__.py         # 主配置入口
│   ├── environment.py      # 环境配置
│   ├── api_config.py       # API配置
│   ├── audio_config.py     # 音频配置
│   ├── di_config.py        # 依赖注入配置
│   ├── logging_config.py   # 日志配置
│   └── paths.py           # 路径配置
│
├── core/                   # 核心模块
│   ├── __init__.py
│   ├── interfaces/         # 接口定义
│   │   ├── __init__.py
│   │   └── unified_interfaces.py
│   ├── conversation_manager.py  # 对话管理器
│   ├── di_container.py     # 依赖注入容器
│   ├── service_registry.py # 服务注册表
│   ├── streaming_processor.py # 流式处理器
│   └── wake_word_detector.py  # 唤醒词检测器
│
├── services/               # 服务层
│   ├── __init__.py
│   ├── api_client.py       # API客户端
│   ├── audio_service.py    # 音频服务
│   ├── chat_processor.py   # 聊天处理器
│   ├── error_handler.py    # 错误处理器
│   ├── exceptions.py       # 自定义异常
│   └── scheduler.py        # 调度器
│
├── managers/               # 管理层
│   ├── __init__.py
│   ├── audio_manager.py    # 音频管理器
│   └── resource_manager.py # 资源管理器
│
├── hardware/               # 硬件接口
│   ├── __init__.py
│   └── audio_interface.py  # 音频硬件接口
│
├── utils/                  # 工具模块
│   ├── __init__.py
│   ├── file_transfer.py    # 文件传输工具
│   └── string_utils.py     # 字符串工具
│
├── tests/                  # 测试模块
│   ├── __init__.py
│   ├── mocks.py           # Mock对象
│   ├── run_tests.py       # 测试运行器
│   ├── test_*.py          # 单元测试
│   └── integration/       # 集成测试
│
├── docs/                   # 文档
│   ├── API.md             # API文档
│   ├── ARCHITECTURE.md    # 架构文档
│   └── DEVELOPMENT.md     # 开发指南
│
├── logs/                   # 日志文件
├── audio/                  # 音频文件
└── temp/                   # 临时文件
```

## 代码规范

### Python代码风格

项目遵循 [PEP 8](https://www.python.org/dev/peps/pep-0008/) 代码风格指南。

#### 命名规范

```python
# 类名：使用PascalCase
class AudioService:
    pass

# 函数和变量名：使用snake_case
def process_audio_data():
    user_input = "hello"
    return user_input

# 常量：使用UPPER_CASE
MAX_RETRY_COUNT = 3
API_TIMEOUT = 30

# 私有方法和属性：使用单下划线前缀
class MyClass:
    def _private_method(self):
        pass
    
    def __init__(self):
        self._private_attr = None
```

#### 文档字符串

使用Google风格的文档字符串：

```python
def send_chat_request(self, bot_id: str, user_id: str, query: str) -> Optional[Dict[str, Any]]:
    """发送聊天请求到Coze API。
    
    Args:
        bot_id: 机器人ID
        user_id: 用户ID  
        query: 用户查询文本
        
    Returns:
        API响应数据字典，失败时返回None
        
    Raises:
        APIError: 当API调用失败时抛出
        
    Example:
        >>> client = EnhancedCozeAPIClient(token)
        >>> response = client.send_chat_request("bot123", "user456", "Hello")
        >>> print(response['status'])
        'success'
    """
    pass
```

#### 类型注解

使用类型注解提高代码可读性：

```python
from typing import Optional, Dict, List, Any, Union

class AudioService:
    def __init__(self, audio_device: IAudioDevice, audio_manager: AudioFileManager) -> None:
        self.audio_device = audio_device
        self.audio_manager = audio_manager
        self.is_recording: bool = False
    
    def record_audio(self, duration: Optional[int] = None) -> Optional[str]:
        """录制音频文件"""
        pass
    
    def get_audio_files(self) -> List[str]:
        """获取音频文件列表"""
        pass
```

### 接口设计规范

#### 接口定义

```python
from abc import ABC, abstractmethod
from typing import Optional, Any

class IService(ABC):
    """服务接口基类"""
    
    @abstractmethod
    def initialize(self) -> bool:
        """初始化服务
        
        Returns:
            初始化是否成功
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """清理资源"""
        pass
```

#### 实现类规范

```python
class ConcreteService(IService):
    """具体服务实现"""
    
    def __init__(self, dependency: IDependency) -> None:
        self.dependency = dependency
        self._initialized = False
    
    def initialize(self) -> bool:
        """初始化服务实现"""
        try:
            # 初始化逻辑
            self._initialized = True
            return True
        except Exception as e:
            logger.error(f"服务初始化失败: {e}")
            return False
    
    def cleanup(self) -> None:
        """清理资源实现"""
        if self._initialized:
            # 清理逻辑
            self._initialized = False
```

### 错误处理规范

#### 异常定义

```python
# services/exceptions.py
class CozeRobotError(Exception):
    """项目基础异常类"""
    pass

class APIError(CozeRobotError):
    """API相关异常"""
    def __init__(self, message: str, status_code: Optional[int] = None):
        super().__init__(message)
        self.status_code = status_code

class AudioError(CozeRobotError):
    """音频相关异常"""
    pass
```

#### 异常处理

```python
def api_operation():
    """API操作示例"""
    try:
        result = some_api_call()
        return result
    except requests.RequestException as e:
        logger.error(f"网络请求失败: {e}")
        raise APIError(f"API调用失败: {e}") from e
    except ValueError as e:
        logger.error(f"数据格式错误: {e}")
        raise APIError(f"响应数据格式错误: {e}") from e
    except Exception as e:
        logger.error(f"未知错误: {e}")
        raise CozeRobotError(f"操作失败: {e}") from e
```

## 测试指南

### 测试结构

```
tests/
├── __init__.py
├── mocks.py                 # Mock对象定义
├── run_tests.py            # 测试运行器
├── conftest.py             # pytest配置
├── test_unit/              # 单元测试
│   ├── test_audio_service.py
│   ├── test_api_client.py
│   └── test_*.py
├── test_integration/       # 集成测试
│   ├── test_conversation_flow.py
│   └── test_*.py
└── test_e2e/              # 端到端测试
    └── test_full_workflow.py
```

### 单元测试

#### 测试类结构

```python
import pytest
from unittest.mock import Mock, patch
from services.audio_service import AudioService
from tests.mocks import MockAudioDevice, MockAudioManager

class TestAudioService:
    """音频服务单元测试"""
    
    def setup_method(self):
        """测试前置设置"""
        self.mock_audio_device = MockAudioDevice()
        self.mock_audio_manager = MockAudioManager()
        self.audio_service = AudioService(
            self.mock_audio_device,
            self.mock_audio_manager
        )
    
    def teardown_method(self):
        """测试后置清理"""
        self.audio_service.cleanup()
    
    def test_start_conversation_success(self):
        """测试成功开始对话"""
        # Arrange
        self.mock_audio_device.detect_wake_word.return_value = True
        
        # Act
        self.audio_service.start_conversation()
        
        # Assert
        assert self.audio_service.is_in_conversation() is True
        self.mock_audio_device.play_hello_audio.assert_called_once()
    
    def test_record_and_transcribe_success(self):
        """测试成功录音和转录"""
        # Arrange
        expected_text = "Hello, world!"
        self.mock_audio_device.record_audio.return_value = "/path/to/audio.wav"
        
        with patch('services.api_client.EnhancedCozeAPIClient') as mock_api:
            mock_api.return_value.speech_to_text.return_value = expected_text
            
            # Act
            result = self.audio_service.record_and_transcribe()
            
            # Assert
            assert result == expected_text
            self.mock_audio_device.record_audio.assert_called_once()
    
    @pytest.mark.parametrize("duration,expected_calls", [
        (5, 1),
        (10, 1),
        (None, 1)
    ])
    def test_record_audio_with_different_durations(self, duration, expected_calls):
        """参数化测试：不同录音时长"""
        # Act
        self.audio_service.record_audio(duration)
        
        # Assert
        assert self.mock_audio_device.record_audio.call_count == expected_calls
```

#### Mock对象

```python
# tests/mocks.py
from unittest.mock import Mock
from core.interfaces.unified_interfaces import IAudioDevice, IAPIClient

class MockAudioDevice(IAudioDevice):
    """音频设备Mock对象"""
    
    def __init__(self):
        self.detect_wake_word = Mock(return_value=True)
        self.detect_bye_word = Mock(return_value=False)
        self.record_audio = Mock(return_value="/mock/audio.wav")
        self.play_audio = Mock(return_value=True)
        self.is_playing = Mock(return_value=False)
        self.stop_playing = Mock()

class MockAPIClient(IAPIClient):
    """API客户端Mock对象"""
    
    def __init__(self):
        self.send_chat_request = Mock(return_value={"status": "success"})
        self.get_chat_status = Mock(return_value={"status": "completed"})
        self.text_to_speech = Mock(return_value="/mock/tts.wav")
        self.speech_to_text = Mock(return_value="Mock transcription")
        self.create_streaming_chat = Mock(return_value=Mock())
```

### 集成测试

```python
# tests/test_integration/test_conversation_flow.py
import pytest
from core.di_container import DIContainer
from config.di_config import DIConfig
from core.service_registry import ServiceRegistry

class TestConversationFlow:
    """对话流程集成测试"""
    
    def setup_method(self):
        """设置测试环境"""
        self.container = DIContainer()
        self.di_config = DIConfig()
        self.service_registry = ServiceRegistry(self.container, self.di_config)
        self.service_registry.register_services()
    
    def test_full_conversation_cycle(self):
        """测试完整对话周期"""
        # Arrange
        conversation_manager = self.container.get_service('IConversationManager')
        
        # Act & Assert
        # 这里会测试真实的服务交互
        conversation_manager.start_conversation()
        
        # 验证各个服务的状态
        audio_service = self.container.get_service('IAudioService')
        assert audio_service.is_in_conversation() is True
```

### 测试运行

```bash
# 运行所有测试
python -m pytest tests/ -v

# 运行特定测试文件
python -m pytest tests/test_audio_service.py -v

# 运行特定测试方法
python -m pytest tests/test_audio_service.py::TestAudioService::test_start_conversation -v

# 生成覆盖率报告
python -m pytest tests/ --cov=. --cov-report=html

# 运行性能测试
python -m pytest tests/ -m performance
```

### 测试配置

```python
# tests/conftest.py
import pytest
import os
from config.environment import Environment

@pytest.fixture(scope="session")
def test_environment():
    """设置测试环境"""
    os.environ['ENVIRONMENT'] = Environment.TESTING.value
    yield
    # 清理

@pytest.fixture
def mock_api_token():
    """Mock API Token"""
    return "mock_token_12345"

@pytest.fixture
def temp_audio_file(tmp_path):
    """创建临时音频文件"""
    audio_file = tmp_path / "test_audio.wav"
    audio_file.write_bytes(b"mock audio data")
    return str(audio_file)
```

## 调试指南

### 日志配置

```python
# config/logging_config.py
import logging
from config.environment import Environment

def setup_logging():
    """配置日志系统"""
    env = Environment.get_current()
    
    if env == Environment.DEVELOPMENT:
        level = logging.DEBUG
    elif env == Environment.TESTING:
        level = logging.WARNING
    else:
        level = logging.INFO
    
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('logs/app.log'),
            logging.StreamHandler()
        ]
    )
```

### 调试技巧

#### 使用断点调试

```python
# 在代码中设置断点
def process_audio():
    import pdb; pdb.set_trace()  # 设置断点
    # 调试代码
    result = some_operation()
    return result
```

#### 日志调试

```python
import logging

logger = logging.getLogger(__name__)

def debug_function():
    logger.debug("开始处理音频")
    
    try:
        result = process_audio()
        logger.info(f"音频处理成功: {result}")
        return result
    except Exception as e:
        logger.error(f"音频处理失败: {e}", exc_info=True)
        raise
```

#### 性能调试

```python
import time
from functools import wraps

def timing_decorator(func):
    """性能计时装饰器"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(f"{func.__name__} 执行时间: {end_time - start_time:.4f}秒")
        return result
    return wrapper

@timing_decorator
def slow_function():
    # 需要性能分析的函数
    pass
```

## 贡献指南

### 贡献流程

1. **Fork项目**
   ```bash
   # 在GitHub上Fork项目
   # 克隆你的Fork
   git clone https://github.com/your-username/CozeRobotChat.git
   ```

2. **创建功能分支**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **开发和测试**
   ```bash
   # 编写代码
   # 运行测试
   python -m pytest tests/ -v
   # 检查代码风格
   flake8 .
   black .
   ```

4. **提交更改**
   ```bash
   git add .
   git commit -m "feat: 添加新功能描述"
   ```

5. **推送分支**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **创建Pull Request**
   - 在GitHub上创建PR
   - 填写详细的PR描述
   - 等待代码审查

### 提交信息规范

使用[Conventional Commits](https://www.conventionalcommits.org/)规范：

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

**类型说明**：
- `feat`: 新功能
- `fix`: 修复bug
- `docs`: 文档更新
- `style`: 代码格式化
- `refactor`: 代码重构
- `test`: 测试相关
- `chore`: 构建过程或辅助工具的变动

**示例**：
```
feat(audio): 添加流式音频处理功能

- 实现实时音频流处理
- 添加音频缓冲机制
- 优化音频质量

Closes #123
```

### 代码审查清单

#### 功能性
- [ ] 功能是否按预期工作
- [ ] 是否处理了边界情况
- [ ] 错误处理是否完善
- [ ] 是否有足够的测试覆盖

#### 代码质量
- [ ] 代码是否遵循项目规范
- [ ] 是否有适当的注释和文档
- [ ] 变量和函数命名是否清晰
- [ ] 是否有代码重复

#### 性能
- [ ] 是否有性能问题
- [ ] 内存使用是否合理
- [ ] 是否有不必要的计算

#### 安全性
- [ ] 是否有安全漏洞
- [ ] 敏感信息是否正确处理
- [ ] 输入验证是否充分

## 最佳实践

### 依赖注入最佳实践

```python
# 好的做法：通过接口依赖
class AudioService:
    def __init__(self, audio_device: IAudioDevice, api_client: IAPIClient):
        self.audio_device = audio_device
        self.api_client = api_client

# 避免：直接依赖具体实现
class AudioService:
    def __init__(self):
        self.audio_device = RobotAudioInterface()  # 不好
        self.api_client = EnhancedCozeAPIClient()  # 不好
```

### 错误处理最佳实践

```python
# 好的做法：具体的异常处理
def api_call():
    try:
        return make_api_request()
    except requests.ConnectionError as e:
        logger.error(f"网络连接失败: {e}")
        raise APIError("无法连接到服务器") from e
    except requests.Timeout as e:
        logger.error(f"请求超时: {e}")
        raise APIError("请求超时，请稍后重试") from e

# 避免：捕获所有异常
def api_call():
    try:
        return make_api_request()
    except Exception as e:  # 不好
        logger.error(f"出错了: {e}")
        return None
```

### 配置管理最佳实践

```python
# 好的做法：分层配置
class Config:
    def __init__(self):
        self.api_config = APIConfig()
        self.audio_config = AudioConfig()
        self.logging_config = LoggingConfig()

# 好的做法：环境变量优先
class APIConfig:
    def __init__(self):
        self.bearer_token = os.getenv('BEARER_TOKEN') or self.get_default_token()
        self.base_url = os.getenv('API_BASE_URL', 'https://api.coze.com')
```

### 测试最佳实践

```python
# 好的做法：测试行为而非实现
def test_audio_service_starts_conversation():
    # Arrange
    audio_service = AudioService(mock_device, mock_manager)
    
    # Act
    audio_service.start_conversation()
    
    # Assert
    assert audio_service.is_in_conversation()  # 测试行为

# 避免：测试内部实现
def test_audio_service_internal_state():
    audio_service = AudioService(mock_device, mock_manager)
    audio_service.start_conversation()
    
    assert audio_service._internal_flag is True  # 不好：测试内部状态
```

## 性能优化

### 内存管理

```python
# 及时清理资源
class AudioProcessor:
    def __init__(self):
        self.temp_files = []
    
    def process_audio(self, audio_path):
        try:
            # 处理音频
            temp_file = create_temp_file()
            self.temp_files.append(temp_file)
            return process(temp_file)
        finally:
            self.cleanup_temp_files()
    
    def cleanup_temp_files(self):
        for file_path in self.temp_files:
            if os.path.exists(file_path):
                os.remove(file_path)
        self.temp_files.clear()
```

### 异步处理

```python
import asyncio
from typing import List

class AsyncAudioProcessor:
    async def process_multiple_audio_files(self, file_paths: List[str]):
        """并行处理多个音频文件"""
        tasks = [self.process_single_file(path) for path in file_paths]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        return results
    
    async def process_single_file(self, file_path: str):
        """处理单个音频文件"""
        # 异步音频处理逻辑
        pass
```

### 缓存策略

```python
from functools import lru_cache
import time

class CachedAPIClient:
    def __init__(self):
        self.cache = {}
        self.cache_ttl = 300  # 5分钟
    
    @lru_cache(maxsize=128)
    def get_bot_info(self, bot_id: str):
        """缓存机器人信息"""
        return self.fetch_bot_info(bot_id)
    
    def get_cached_response(self, cache_key: str):
        """获取缓存的响应"""
        if cache_key in self.cache:
            cached_data, timestamp = self.cache[cache_key]
            if time.time() - timestamp < self.cache_ttl:
                return cached_data
            else:
                del self.cache[cache_key]
        return None
```

## 部署指南

### 本地部署

```bash
# 1. 准备环境
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 2. 配置环境变量
cp .env.example .env
# 编辑 .env 文件

# 3. 运行应用
python main.py
```

### Docker部署

```dockerfile
# Dockerfile
FROM python:3.9-slim

WORKDIR /app

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 复制应用代码
COPY . .

# 创建必要目录
RUN mkdir -p logs audio temp

# 设置权限
RUN chmod +x main.py

# 暴露端口（如果需要）
EXPOSE 8000

# 启动应用
CMD ["python", "main.py"]
```

```yaml
# docker-compose.yml
version: '3.8'

services:
  coze-robot:
    build: .
    environment:
      - ENVIRONMENT=production
      - BEARER_TOKEN=${BEARER_TOKEN}
      - BOT_ID=${BOT_ID}
    volumes:
      - ./logs:/app/logs
      - ./audio:/app/audio
      - ./temp:/app/temp
    restart: unless-stopped
    devices:
      - "/dev/snd:/dev/snd"  # 音频设备访问
```

### 生产环境部署

```bash
# 1. 构建Docker镜像
docker build -t coze-robot-chat .

# 2. 运行容器
docker-compose up -d

# 3. 查看日志
docker-compose logs -f

# 4. 健康检查
docker-compose ps
```

## 故障排除

### 常见问题

#### 音频设备问题

```bash
# 检查音频设备
arecord -l  # 列出录音设备
aplay -l   # 列出播放设备

# 测试录音
arecord -d 5 test.wav
aplay test.wav
```

#### API连接问题

```python
# 测试API连接
import requests

def test_api_connection():
    try:
        response = requests.get('https://api.coze.com/health', timeout=10)
        print(f"API状态: {response.status_code}")
    except requests.RequestException as e:
        print(f"API连接失败: {e}")
```

#### 依赖问题

```bash
# 检查依赖版本
pip list

# 重新安装依赖
pip install --force-reinstall -r requirements.txt

# 清理缓存
pip cache purge
```

### 日志分析

```bash
# 查看错误日志
grep "ERROR" logs/app.log

# 查看最近的日志
tail -f logs/app.log

# 按时间过滤日志
grep "2024-01-20" logs/app.log
```

## 社区和支持

### 获取帮助

- **GitHub Issues**: 报告bug和功能请求
- **Discussions**: 技术讨论和问答
- **Wiki**: 详细文档和教程

### 贡献方式

- 提交bug报告
- 提出功能建议
- 贡献代码
- 改进文档
- 分享使用经验

---

感谢您对CozeRobotChat项目的贡献！如有任何问题，请随时联系项目维护者。