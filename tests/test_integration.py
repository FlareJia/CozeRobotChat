# tests/test_integration.py
import unittest
from unittest.mock import Mock, patch
import sys
import os

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 模拟外部依赖模块以避免导入错误
import unittest.mock
sys.modules['pyaudio'] = unittest.mock.MagicMock()
sys.modules['dotenv'] = unittest.mock.MagicMock()
sys.modules['requests'] = unittest.mock.MagicMock()
sys.modules['pygame'] = unittest.mock.MagicMock()
sys.modules['pydub'] = unittest.mock.MagicMock()
sys.modules['speech_recognition'] = unittest.mock.MagicMock()
sys.modules['retrying'] = unittest.mock.MagicMock()
sys.modules['filelock'] = unittest.mock.MagicMock()
sys.modules['psutil'] = unittest.mock.MagicMock()
sys.modules['audioop'] = unittest.mock.MagicMock()
sys.modules['apscheduler'] = unittest.mock.MagicMock()
sys.modules['apscheduler.schedulers'] = unittest.mock.MagicMock()
sys.modules['apscheduler.schedulers.background'] = unittest.mock.MagicMock()
sys.modules['pynput'] = unittest.mock.MagicMock()
sys.modules['pynput.keyboard'] = unittest.mock.MagicMock()

from core.di_container import DIContainer
from core.service_registry import ServiceRegistry
from config import DIConfig, Environment
from core.interfaces.unified_interfaces import (
    IAudioDevice, IAudioService, IAPIClient, IConversationManager,
    IWakeWordDetector, IChatProcessor, IStreamingProcessor, IErrorHandler
)
from app import Application


class TestDependencyInjectionIntegration(unittest.TestCase):
    """依赖注入系统集成测试"""
    
    def setUp(self):
        """测试前准备"""
        self.config = DIConfig()
        self.config.environment = Environment.TESTING
        self.container = DIContainer()
        self.registry = ServiceRegistry(self.container, self.config)
    
    def tearDown(self):
        """测试后清理"""
        self.container.cleanup()
    
    @patch('hardware.audio_interface.RobotAudioInterface')
    @patch('services.api_client.EnhancedCozeAPIClient')
    @patch('services.audio_service.AudioService')
    def test_full_service_registration_and_resolution(self, mock_audio_service, mock_api_client, mock_audio_device):
        """测试完整的服务注册和解析流程"""
        # 配置模拟对象
        mock_audio_device.return_value = Mock()
        mock_api_client.return_value = Mock()
        mock_audio_service.return_value = Mock()
        
        # 注册所有服务
        self.registry.register_all_services()
        
        # 验证核心服务可以被解析
        audio_device = self.container.resolve(IAudioDevice)
        api_client = self.container.resolve(IAPIClient)
        audio_service = self.container.resolve(IAudioService)
        
        self.assertIsNotNone(audio_device)
        self.assertIsNotNone(api_client)
        self.assertIsNotNone(audio_service)
    
    @patch('hardware.audio_interface.RobotAudioInterface')
    @patch('services.api_client.EnhancedCozeAPIClient')
    def test_singleton_behavior(self, mock_api_client, mock_audio_device):
        """测试单例服务行为"""
        # 配置模拟对象
        mock_audio_device.return_value = Mock()
        mock_api_client.return_value = Mock()
        
        # 注册服务
        self.registry.register_all_services()
        
        # 多次解析同一个单例服务
        device1 = self.container.resolve(IAudioDevice)
        device2 = self.container.resolve(IAudioDevice)
        
        # 应该是同一个实例
        self.assertIs(device1, device2)
    
    def test_application_initialization_with_di(self):
        """测试应用程序使用依赖注入的初始化"""
        # 创建应用程序实例
        app = Application()
        
        # 验证依赖注入组件已初始化
        self.assertIsNotNone(app.di_config)
        self.assertIsNotNone(app.container)
        self.assertIsNotNone(app.service_registry)
    
    def test_service_dependency_injection(self):
        """测试服务间的依赖注入"""
        # 创建模拟服务
        mock_audio_device = Mock()
        mock_api_client = Mock()
        
        # 手动注册服务
        self.container.register_singleton(IAudioDevice, lambda: mock_audio_device)
        self.container.register_singleton(IAPIClient, lambda: mock_api_client)
        
        # 创建需要依赖的服务
        class TestService:
            def __init__(self, audio_device: IAudioDevice, api_client: IAPIClient):
                self.audio_device = audio_device
                self.api_client = api_client
        
        # 注册测试服务
        self.container.register_transient(
            TestService,
            lambda: TestService(
                self.container.resolve(IAudioDevice),
                self.container.resolve(IAPIClient)
            )
        )
        
        # 解析测试服务
        test_service = self.container.resolve(TestService)
        
        # 验证依赖已正确注入
        self.assertIs(test_service.audio_device, mock_audio_device)
        self.assertIs(test_service.api_client, mock_api_client)
    
    def test_environment_specific_service_resolution(self):
        """测试环境特定的服务解析"""
        # 测试开发环境
        self.config.environment = Environment.DEVELOPMENT
        dev_impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(dev_impl, 'EnhancedCozeAPIClient')
        
        # 测试测试环境
        self.config.environment = Environment.TESTING
        test_impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(test_impl, 'MockAPIClient')
        
        # 测试生产环境
        self.config.environment = Environment.PRODUCTION
        prod_impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(prod_impl, 'EnhancedCozeAPIClient')
    
    def test_service_configuration_injection(self):
        """测试服务配置注入"""
        # 获取音频服务配置
        audio_config = self.config.get_service_config('audio_service')
        
        # 验证配置内容
        self.assertEqual(audio_config['buffer_size'], 1024)
        self.assertEqual(audio_config['sample_rate'], 16000)
        self.assertEqual(audio_config['channels'], 1)
        
        # 获取API客户端配置
        api_config = self.config.get_service_config('api_client')
        
        # 验证配置内容
        self.assertEqual(api_config['timeout'], 30)
        self.assertEqual(api_config['max_retries'], 3)
    
    def test_container_cleanup(self):
        """测试容器清理功能"""
        # 注册一些服务
        self.container.register_singleton(IAudioDevice, Mock)
        self.container.register_transient(IAPIClient, Mock)
        
        # 创建一些实例
        self.container.resolve(IAudioDevice)
        
        # 验证服务已注册
        services = self.container.get_registered_services()
        self.assertGreater(len(services), 0)
        
        # 清理容器
        self.container.cleanup()
        
        # 验证服务已清理
        services = self.container.get_registered_services()
        self.assertEqual(len(services), 0)
    
    def test_error_handling_in_service_registration(self):
        """测试服务注册中的错误处理"""
        # 创建一个新的注册表实例用于测试
        test_registry = ServiceRegistry(self.container, self.config)
        
        # 模拟logger
        with patch.object(test_registry, 'logger') as mock_logger:
            # 模拟服务注册失败
            with patch.object(test_registry, '_register_infrastructure_services', side_effect=Exception("注册失败")):
                with self.assertRaises(Exception):
                    test_registry.register_all_services()
                
                # 验证错误被记录
                mock_logger.error.assert_called_with("服务注册失败: 注册失败")


if __name__ == '__main__':
    unittest.main()