# 架构文档

## 概述

CozeRobotChat 是一个基于接口驱动的智能对话机器人系统，采用分层架构和依赖注入模式，具有高度的模块化和可扩展性。系统支持语音交互、文本处理、流式对话等功能。

## 架构原则

### 1. 接口驱动设计 (Interface-Driven Design)
- 所有核心功能都通过接口进行抽象
- 实现与接口分离，便于测试和扩展
- 支持多种实现方式的切换

### 2. 依赖注入 (Dependency Injection)
- 使用DIContainer管理对象生命周期
- 通过配置文件控制依赖关系
- 支持不同环境下的不同实现

### 3. 分层架构 (Layered Architecture)
- 清晰的职责分离
- 单向依赖关系
- 易于维护和扩展

### 4. 领域驱动设计 (Domain-Driven Design)
- 核心业务逻辑集中在领域层
- 基础设施与业务逻辑分离
- 明确的边界上下文

## 系统架构图

```
┌─────────────────────────────────────────────────────────────┐
│                        应用层 (Application)                  │
├─────────────────────────────────────────────────────────────┤
│  main.py  │  app.py  │  ConversationManager                 │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                        服务层 (Services)                     │
├─────────────────────────────────────────────────────────────┤
│  AudioService  │  ChatProcessor  │  StreamingProcessor      │
│  APIClient     │  ErrorHandler   │  Scheduler               │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                        管理层 (Managers)                     │
├─────────────────────────────────────────────────────────────┤
│  AudioManager  │  ResourceManager                           │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                        核心层 (Core)                         │
├─────────────────────────────────────────────────────────────┤
│  Interfaces    │  ServiceRegistry  │  DIContainer           │
│  WakeWordDetector  │  StreamingProcessor                    │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                      基础设施层 (Infrastructure)              │
├─────────────────────────────────────────────────────────────┤
│  Hardware      │  Utils        │  Config                    │
│  AudioInterface│  FileTransfer │  Environment               │
└─────────────────────────────────────────────────────────────┘
```

## 核心组件

### 1. 应用层 (Application Layer)

#### 主要职责
- 应用程序入口点
- 系统初始化和配置
- 对话流程控制

#### 核心组件

**main.py**
- 应用程序入口
- 简化的启动逻辑

**app.py**
- 系统初始化
- 依赖注入配置
- 服务注册

**ConversationManager**
- 对话流程控制
- 协调各个服务
- 性能监控

### 2. 服务层 (Service Layer)

#### 主要职责
- 业务逻辑实现
- 外部API集成
- 数据处理和转换

#### 核心服务

**AudioService**
```python
class AudioService(IAudioService):
    """音频服务实现"""
    - 对话流程管理
    - 音频录制和播放
    - 语音转录
```

**EnhancedCozeAPIClient**
```python
class EnhancedCozeAPIClient(IAPIClient):
    """增强的Coze API客户端"""
    - HTTP请求处理
    - 错误重试机制
    - 响应数据解析
```

**ChatProcessor**
```python
class ChatProcessor(IChatProcessor):
    """聊天处理器"""
    - 用户查询处理
    - 响应生成
    - 上下文管理
```

**StreamingProcessor**
```python
class StreamingProcessor(IStreamingProcessor):
    """流式处理器"""
    - 实时对话处理
    - 流式响应处理
    - 状态管理
```

**AdvancedErrorHandler**
```python
class AdvancedErrorHandler(IErrorHandler):
    """高级错误处理器"""
    - 分类错误处理
    - 错误恢复策略
    - 错误历史记录
```

### 3. 管理层 (Manager Layer)

#### 主要职责
- 资源管理
- 文件操作
- 配置管理

#### 核心管理器

**AudioFileManager**
```python
class AudioFileManager:
    """音频文件管理器"""
    - 音频文件存储
    - 文件路径管理
    - 临时文件清理
```

**ResourceManager**
```python
class ResourceManager:
    """资源管理器"""
    - 系统资源监控
    - 内存管理
    - 性能优化
```

### 4. 核心层 (Core Layer)

#### 主要职责
- 接口定义
- 依赖注入
- 服务注册
- 核心算法

#### 核心组件

**统一接口 (Unified Interfaces)**
```python
# core/interfaces/unified_interfaces.py
- IAudioDevice: 音频设备接口
- IAudioService: 音频服务接口
- IAPIClient: API客户端接口
- IConversationManager: 对话管理接口
- IChatProcessor: 聊天处理接口
- IStreamingProcessor: 流式处理接口
- IErrorHandler: 错误处理接口
```

**依赖注入容器 (DI Container)**
```python
class DIContainer:
    """依赖注入容器"""
    - 服务实例管理
    - 依赖关系解析
    - 生命周期控制
```

**服务注册表 (Service Registry)**
```python
class ServiceRegistry:
    """服务注册表"""
    - 服务注册和发现
    - 接口与实现映射
    - 环境相关配置
```

**唤醒词检测器 (Wake Word Detector)**
```python
class WakeWordDetector:
    """唤醒词检测器"""
    - 实时音频监听
    - 关键词识别
    - 状态管理
```

### 5. 基础设施层 (Infrastructure Layer)

#### 主要职责
- 硬件接口
- 工具函数
- 配置管理
- 外部系统集成

#### 核心组件

**硬件接口 (Hardware)**
```python
class RobotAudioInterface(IAudioDevice):
    """机器人音频接口实现"""
    - 麦克风控制
    - 扬声器控制
    - 音频处理
```

**配置管理 (Configuration)**
```python
# 配置文件结构
config/
├── __init__.py          # 主配置入口
├── environment.py       # 环境配置
├── api_config.py        # API配置
├── audio_config.py      # 音频配置
├── di_config.py         # 依赖注入配置
├── logging_config.py    # 日志配置
└── paths.py            # 路径配置
```

**工具函数 (Utils)**
```python
utils/
├── file_transfer.py     # 文件传输工具
├── string_utils.py      # 字符串处理工具
└── ...
```

## 数据流图

### 完整对话流程

```
用户语音输入
     │
     ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│ AudioDevice │───▶│ AudioService │───▶│ APIClient   │
│ 音频设备     │    │ 音频服务      │    │ API客户端    │
└─────────────┘    └──────────────┘    └─────────────┘
     │                     │                   │
     ▼                     ▼                   ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│ 唤醒词检测   │    │ 语音转文本    │    │ 文本处理     │
└─────────────┘    └──────────────┘    └─────────────┘
                           │                   │
                           ▼                   ▼
                  ┌──────────────┐    ┌─────────────┐
                  │ ChatProcessor│    │ 响应生成     │
                  │ 聊天处理器    │    └─────────────┘
                  └──────────────┘           │
                           │                   ▼
                           ▼            ┌─────────────┐
                  ┌──────────────┐    │ 文本转语音   │
                  │ 响应处理      │    └─────────────┘
                  └──────────────┘           │
                                            ▼
                                   ┌─────────────┐
                                   │ 音频播放     │
                                   └─────────────┘
```

### 流式处理流程

```
实时音频流
     │
     ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│ 音频缓冲     │───▶│ 实时转录      │───▶│ 流式处理     │
└─────────────┘    └──────────────┘    └─────────────┘
     │                     │                   │
     ▼                     ▼                   ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│ 音频分片     │    │ 文本片段      │    │ 实时响应     │
└─────────────┘    └──────────────┘    └─────────────┘
```

## 设计模式

### 1. 策略模式 (Strategy Pattern)

不同环境下使用不同的服务实现：

```python
# 生产环境
production_services = {
    'IAPIClient': 'EnhancedCozeAPIClient',
    'IAudioService': 'AudioService'
}

# 测试环境
testing_services = {
    'IAPIClient': 'MockAPIClient',
    'IAudioService': 'MockAudioService'
}
```

### 2. 工厂模式 (Factory Pattern)

服务实例的创建和管理：

```python
class ServiceRegistry:
    def create_service(self, interface_name: str):
        implementation_class = self.get_implementation(interface_name)
        return self.container.create_instance(implementation_class)
```

### 3. 观察者模式 (Observer Pattern)

错误处理和事件通知：

```python
class ErrorHandler:
    def __init__(self):
        self.observers = []
    
    def notify_observers(self, error):
        for observer in self.observers:
            observer.on_error(error)
```

### 4. 装饰器模式 (Decorator Pattern)

性能监控和日志记录：

```python
@time_recorder("API调用")
def send_chat_request(self, ...):
    # API调用逻辑
    pass
```

### 5. 适配器模式 (Adapter Pattern)

不同API的统一接口：

```python
class CozeAPIAdapter(IAPIClient):
    def __init__(self, coze_client):
        self.client = coze_client
    
    def send_chat_request(self, ...):
        # 适配Coze API调用
        return self.client.chat(...)
```

## 配置管理架构

### 配置层次结构

```
配置优先级（从高到低）：
1. 环境变量
2. .env 文件
3. 配置文件默认值
4. 系统默认值
```

### 配置模块化

```python
# 配置模块结构
config/
├── __init__.py          # 统一配置入口
├── environment.py       # 环境检测和切换
├── api_config.py        # API相关配置
├── audio_config.py      # 音频相关配置
├── di_config.py         # 依赖注入配置
├── logging_config.py    # 日志配置
└── paths.py            # 路径配置
```

### 环境配置

```python
class Environment(Enum):
    DEVELOPMENT = "development"
    TESTING = "testing"
    PRODUCTION = "production"
    
    @classmethod
    def get_current(cls) -> 'Environment':
        # 环境检测逻辑
        pass
```

## 错误处理架构

### 错误分类

```python
class ErrorCategory(Enum):
    API = "api"          # API调用错误
    AUDIO = "audio"      # 音频处理错误
    SYSTEM = "system"    # 系统错误
    NETWORK = "network"  # 网络错误
    FILE = "file"        # 文件操作错误
    UNKNOWN = "unknown"  # 未知错误
```

### 错误处理策略

```python
class ErrorHandlingStrategy:
    """错误处理策略"""
    
    def handle_api_error(self, error):
        # API错误处理：重试、降级、缓存
        pass
    
    def handle_audio_error(self, error):
        # 音频错误处理：设备重置、格式转换
        pass
    
    def handle_system_error(self, error):
        # 系统错误处理：资源清理、状态重置
        pass
```

## 测试架构

### 测试层次

```
测试金字塔：
┌─────────────────┐
│   E2E Tests     │  ← 端到端测试
├─────────────────┤
│ Integration     │  ← 集成测试
│    Tests        │
├─────────────────┤
│   Unit Tests    │  ← 单元测试
└─────────────────┘
```

### Mock服务架构

```python
# Mock服务实现
tests/mocks.py:
- MockAudioService
- MockAPIClient
- MockAudioDevice
- MockErrorHandler
```

### 测试配置

```python
# 测试环境配置
class TestingDIConfig(DIConfig):
    def get_service_implementations(self):
        return {
            'IAPIClient': 'MockAPIClient',
            'IAudioService': 'MockAudioService',
            # ... 其他Mock实现
        }
```

## 性能优化架构

### 缓存策略

```python
# API响应缓存
class CachedAPIClient(IAPIClient):
    def __init__(self, client, cache):
        self.client = client
        self.cache = cache
    
    def send_chat_request(self, ...):
        cache_key = self.generate_cache_key(...)
        if cache_key in self.cache:
            return self.cache[cache_key]
        
        result = self.client.send_chat_request(...)
        self.cache[cache_key] = result
        return result
```

### 异步处理

```python
# 异步音频处理
class AsyncAudioProcessor:
    async def process_audio_stream(self, audio_stream):
        # 异步音频处理逻辑
        pass
```

### 资源管理

```python
# 资源监控和管理
class ResourceManager:
    def monitor_memory_usage(self):
        # 内存使用监控
        pass
    
    def cleanup_temp_files(self):
        # 临时文件清理
        pass
```

## 扩展性设计

### 插件架构

```python
# 插件接口
class IPlugin(ABC):
    @abstractmethod
    def initialize(self, context):
        pass
    
    @abstractmethod
    def execute(self, input_data):
        pass

# 插件管理器
class PluginManager:
    def load_plugins(self, plugin_dir):
        # 动态加载插件
        pass
```

### 服务扩展

```python
# 新服务接口
class INewService(ABC):
    @abstractmethod
    def new_method(self):
        pass

# 服务注册
service_registry.register_service('INewService', 'NewServiceImpl')
```

## 安全架构

### 配置安全

```python
# 敏感信息加密
class SecureConfig:
    def __init__(self):
        self.cipher = Fernet(self.get_encryption_key())
    
    def get_secure_value(self, key):
        encrypted_value = os.getenv(key)
        return self.cipher.decrypt(encrypted_value.encode()).decode()
```

### API安全

```python
# API调用安全
class SecureAPIClient(IAPIClient):
    def __init__(self, token_manager):
        self.token_manager = token_manager
    
    def send_chat_request(self, ...):
        token = self.token_manager.get_valid_token()
        # 使用安全token进行API调用
        pass
```

## 监控和日志架构

### 日志层次

```python
# 日志配置
logging_config = {
    'version': 1,
    'handlers': {
        'file': {
            'class': 'logging.FileHandler',
            'filename': 'app.log',
            'level': 'INFO'
        },
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'DEBUG'
        }
    },
    'loggers': {
        'audio': {'level': 'DEBUG'},
        'api': {'level': 'INFO'},
        'error': {'level': 'ERROR'}
    }
}
```

### 性能监控

```python
# 性能指标收集
class PerformanceMonitor:
    def record_execution_time(self, operation, duration):
        # 记录执行时间
        pass
    
    def record_memory_usage(self, component, usage):
        # 记录内存使用
        pass
```

## 部署架构

### 容器化

```dockerfile
# Dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .
CMD ["python", "main.py"]
```

### 配置管理

```yaml
# docker-compose.yml
version: '3.8'
services:
  coze-robot:
    build: .
    environment:
      - ENVIRONMENT=production
      - BEARER_TOKEN=${BEARER_TOKEN}
    volumes:
      - ./logs:/app/logs
      - ./audio:/app/audio
```

---

这个架构设计确保了系统的可维护性、可扩展性和可测试性，同时提供了清晰的职责分离和灵活的配置管理。