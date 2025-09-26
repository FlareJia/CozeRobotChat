# API 文档

## 概述

CozeRobotChat 项目采用基于接口的架构设计，所有核心功能都通过定义良好的接口进行抽象。本文档详细介绍了项目中的主要接口和服务的使用方法。

## 核心接口

### 音频相关接口

#### IAudioDevice

音频设备接口，定义了与音频硬件交互的基本方法。

```python
from core.interfaces.unified_interfaces import IAudioDevice

class IAudioDevice(ABC):
    @abstractmethod
    def detect_wake_word(self) -> bool:
        """检测唤醒词"""
        pass
    
    @abstractmethod
    def detect_bye_word(self) -> bool:
        """检测结束词"""
        pass
    
    @abstractmethod
    def record_audio(self, duration: int = None) -> str:
        """录制音频
        
        Args:
            duration: 录制时长（秒），None表示手动停止
            
        Returns:
            录制的音频文件路径
        """
        pass
    
    @abstractmethod
    def play_audio(self, audio_path: str) -> bool:
        """播放音频
        
        Args:
            audio_path: 音频文件路径
            
        Returns:
            播放是否成功
        """
        pass
    
    @abstractmethod
    def is_playing(self) -> bool:
        """检查是否正在播放"""
        pass
    
    @abstractmethod
    def stop_playing(self) -> None:
        """停止播放"""
        pass
```

**实现类**: `RobotAudioInterface`

**使用示例**:
```python
from hardware.audio_interface import RobotAudioInterface

audio_device = RobotAudioInterface()

# 检测唤醒词
if audio_device.detect_wake_word():
    print("检测到唤醒词")

# 录制音频
audio_path = audio_device.record_audio(duration=5)
print(f"录制完成: {audio_path}")

# 播放音频
success = audio_device.play_audio(audio_path)
if success:
    print("播放成功")
```

#### IAudioService

音频服务接口，提供高级音频操作功能。

```python
class IAudioService(ABC):
    @abstractmethod
    def start_conversation(self) -> None:
        """开始对话流程"""
        pass
    
    @abstractmethod
    def end_conversation(self) -> None:
        """结束对话流程"""
        pass
    
    @abstractmethod
    def is_in_conversation(self) -> bool:
        """检查是否在对话流程中"""
        pass
    
    @abstractmethod
    def play_wait_audio(self) -> None:
        """播放等待音频"""
        pass
    
    @abstractmethod
    def play_hello_audio(self) -> None:
        """播放问候音频"""
        pass
    
    @abstractmethod
    def play_bye_audio(self) -> None:
        """播放告别音频"""
        pass
    
    @abstractmethod
    def record_and_transcribe(self) -> Optional[str]:
        """录音并转录
        
        Returns:
            转录的文本，失败时返回None
        """
        pass
    
    @abstractmethod
    def play_result_audio(self, audio_path: str) -> bool:
        """播放结果音频
        
        Args:
            audio_path: 音频文件路径
            
        Returns:
            播放是否成功
        """
        pass
```

**实现类**: `AudioService`

**使用示例**:
```python
from services.audio_service import AudioService
from hardware.audio_interface import RobotAudioInterface
from managers.audio_manager import AudioFileManager

audio_interface = RobotAudioInterface()
audio_manager = AudioFileManager()
audio_service = AudioService(audio_interface, audio_manager)

# 开始对话
audio_service.start_conversation()

# 录音并转录
text = audio_service.record_and_transcribe()
if text:
    print(f"用户说: {text}")

# 结束对话
audio_service.end_conversation()
```

### API相关接口

#### IAPIClient

API客户端接口，定义了与Coze API交互的方法。

```python
class IAPIClient(ABC):
    @abstractmethod
    def send_chat_request(self, bot_id: str, user_id: str, query: str) -> Optional[Dict[str, Any]]:
        """发送聊天请求
        
        Args:
            bot_id: 机器人ID
            user_id: 用户ID
            query: 用户查询
            
        Returns:
            API响应数据
        """
        pass
    
    @abstractmethod
    def get_chat_status(self, conversation_id: str, chat_id: str) -> Optional[Dict[str, Any]]:
        """获取聊天状态
        
        Args:
            conversation_id: 对话ID
            chat_id: 聊天ID
            
        Returns:
            聊天状态数据
        """
        pass
    
    @abstractmethod
    def text_to_speech(self, text: str, voice_id: str = None) -> Optional[str]:
        """文本转语音
        
        Args:
            text: 要转换的文本
            voice_id: 语音ID
            
        Returns:
            生成的音频文件路径
        """
        pass
    
    @abstractmethod
    def speech_to_text(self, audio_path: str) -> Optional[str]:
        """语音转文本
        
        Args:
            audio_path: 音频文件路径
            
        Returns:
            转录的文本
        """
        pass
    
    @abstractmethod
    def create_streaming_chat(self, bot_id: str, user_id: str, query: str) -> Optional[Any]:
        """创建流式聊天
        
        Args:
            bot_id: 机器人ID
            user_id: 用户ID
            query: 用户查询
            
        Returns:
            流式聊天对象
        """
        pass
```

**实现类**: `EnhancedCozeAPIClient`

**使用示例**:
```python
from services.api_client import EnhancedCozeAPIClient
from config import Config

api_client = EnhancedCozeAPIClient(Config.BEARER_TOKEN)

# 发送聊天请求
response = api_client.send_chat_request(
    bot_id="your_bot_id",
    user_id="user123",
    query="你好，请介绍一下自己"
)

if response:
    print(f"API响应: {response}")

# 文本转语音
audio_path = api_client.text_to_speech("你好，我是智能助手")
if audio_path:
    print(f"生成音频: {audio_path}")

# 语音转文本
text = api_client.speech_to_text("/path/to/audio.wav")
if text:
    print(f"转录结果: {text}")
```

### 对话管理接口

#### IConversationManager

对话管理接口，负责整个对话流程的控制。

```python
class IConversationManager(ABC):
    @abstractmethod
    def start_conversation(self) -> None:
        """开始对话"""
        pass
```

**实现类**: `ConversationManager`

**使用示例**:
```python
from core.conversation_manager import ConversationManager

# 通过依赖注入获取实例
conversation_manager = container.get_service('IConversationManager')

# 开始对话
conversation_manager.start_conversation()
```

#### IChatProcessor

聊天处理接口，处理用户查询。

```python
class IChatProcessor(ABC):
    @abstractmethod
    def process_query(self, query: str) -> Optional[str]:
        """处理用户查询
        
        Args:
            query: 用户查询文本
            
        Returns:
            处理结果，失败时返回None
        """
        pass
```

**实现类**: `ChatProcessor`

#### IStreamingProcessor

流式处理接口，处理流式对话。

```python
class IStreamingProcessor(ABC):
    @abstractmethod
    def process(self, transcript: str) -> bool:
        """处理流式对话
        
        Args:
            transcript: 转录文本
            
        Returns:
            处理是否成功
        """
        pass
```

**实现类**: `StreamingProcessor`

### 错误处理接口

#### IErrorHandler

错误处理接口，提供统一的错误处理机制。

```python
class ErrorCategory(Enum):
    """错误类别枚举"""
    API = "api"
    AUDIO = "audio"
    SYSTEM = "system"
    NETWORK = "network"
    FILE = "file"
    UNKNOWN = "unknown"

class IErrorHandler(ABC):
    @abstractmethod
    def handle_error(self, error: Exception, category: ErrorCategory = ErrorCategory.UNKNOWN) -> None:
        """处理错误
        
        Args:
            error: 异常对象
            category: 错误类别
        """
        pass
    
    @abstractmethod
    def handle_api_error(self, context: Any) -> None:
        """处理API错误"""
        pass
    
    @abstractmethod
    def handle_audio_error(self, context: Any) -> None:
        """处理音频错误"""
        pass
    
    @abstractmethod
    def get_error_history(self) -> List[Any]:
        """获取错误历史"""
        pass
```

**实现类**: `AdvancedErrorHandler`

**使用示例**:
```python
from services.error_handler import AdvancedErrorHandler
from core.interfaces.unified_interfaces import ErrorCategory

error_handler = AdvancedErrorHandler()

try:
    # 一些可能出错的操作
    pass
except Exception as e:
    error_handler.handle_error(e, ErrorCategory.API)
```

## 服务注册与依赖注入

### 服务注册

项目使用 `ServiceRegistry` 来管理服务的注册和获取：

```python
from core.service_registry import ServiceRegistry
from core.di_container import DIContainer
from config import DIConfig

# 初始化
container = DIContainer()
di_config = DIConfig()
service_registry = ServiceRegistry(container, di_config)

# 注册服务
service_registry.register_services()

# 获取服务
audio_service = container.get_service('IAudioService')
api_client = container.get_service('IAPIClient')
```

### 依赖注入配置

在 `config/di_config.py` 中配置服务映射：

```python
class DIConfig:
    def get_service_implementations(self) -> Dict[str, str]:
        """获取服务实现映射"""
        if self.environment == Environment.TESTING:
            return {
                'IAPIClient': 'MockAPIClient',
                'IAudioService': 'MockAudioService',
                # ... 其他测试实现
            }
        else:
            return {
                'IAPIClient': 'EnhancedCozeAPIClient',
                'IAudioService': 'AudioService',
                # ... 其他生产实现
            }
```

## 配置管理

### 环境配置

```python
from config import Environment, Config

# 获取当前环境
current_env = Environment.get_current()

# 根据环境获取配置
config = Config()
api_token = config.BEARER_TOKEN
bot_id = config.BOT_ID
```

### 音频配置

```python
from config import AudioConfig

audio_config = AudioConfig()

# 获取音频参数
voice_id = audio_config.VOICE_ID
record_settings = audio_config.RECORD_SETTINGS
audio_names = audio_config.AUDIO_NAMES
```

## 错误处理

### 异常类型

项目定义了多种异常类型：

```python
from services.exceptions import AudioError, APIError

# 音频相关异常
raise AudioError("音频设备初始化失败")

# API相关异常
raise APIError("API调用失败", status_code=500)
```

### 错误处理最佳实践

```python
from core.interfaces.unified_interfaces import IErrorHandler, ErrorCategory

def some_api_operation():
    try:
        # API操作
        result = api_client.send_chat_request(...)
        return result
    except APIError as e:
        error_handler.handle_error(e, ErrorCategory.API)
        return None
    except Exception as e:
        error_handler.handle_error(e, ErrorCategory.UNKNOWN)
        return None
```

## 测试

### Mock服务

项目提供了完整的Mock服务用于测试：

```python
from tests.mocks import MockAudioService, MockAPIClient

# 在测试中使用Mock服务
mock_audio = MockAudioService()
mock_api = MockAPIClient()

# 设置Mock行为
mock_api.send_chat_request.return_value = {"status": "success"}
```

### 集成测试

```python
from tests.test_integration import IntegrationTest

# 运行集成测试
test = IntegrationTest()
test.test_full_conversation_flow()
```

## 扩展开发

### 添加新的接口

1. 在 `core/interfaces/unified_interfaces.py` 中定义接口
2. 创建具体实现类
3. 在 `ServiceRegistry` 中注册服务
4. 更新依赖注入配置

### 自定义音频处理

```python
from core.interfaces.unified_interfaces import IAudioDevice

class CustomAudioDevice(IAudioDevice):
    def detect_wake_word(self) -> bool:
        # 自定义唤醒词检测逻辑
        pass
    
    def record_audio(self, duration: int = None) -> str:
        # 自定义录音逻辑
        pass
    
    # 实现其他抽象方法...
```

### 集成新的AI模型

```python
from core.interfaces.unified_interfaces import IAPIClient

class CustomAPIClient(IAPIClient):
    def send_chat_request(self, bot_id: str, user_id: str, query: str) -> Optional[Dict[str, Any]]:
        # 自定义API调用逻辑
        pass
    
    # 实现其他抽象方法...
```

## 性能监控

项目内置了性能监控功能：

```python
from core.conversation_manager import time_recorder

with time_recorder("API调用"):
    result = api_client.send_chat_request(...)
    # 自动记录执行时间
```

## 日志记录

```python
import logging

logger = logging.getLogger(__name__)

# 记录不同级别的日志
logger.info("操作成功")
logger.warning("警告信息")
logger.error("错误信息")
logger.debug("调试信息")
```

---

更多详细信息请参考源代码和单元测试。