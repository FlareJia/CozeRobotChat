# tests/test_service_registry.py
import unittest
from unittest.mock import Mock, patch, MagicMock
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

from core.di_container import DIContainer
from core.service_registry import ServiceRegistry
from config import DIConfig, Environment
from core.interfaces.unified_interfaces import (
    IAudioDevice, IAudioService, IAPIClient, IConversationManager,
    IWakeWordDetector, IChatProcessor, IStreamingProcessor, IErrorHandler
)


class TestServiceRegistry(unittest.TestCase):
    """服务注册器测试"""
    
    def setUp(self):
        """测试前准备"""
        self.container = DIContainer()
        self.config = DIConfig()
        self.registry = ServiceRegistry(self.container, self.config)
    
    def tearDown(self):
        """测试后清理"""
        self.container.cleanup()
    
    def test_service_registry_initialization(self):
        """测试服务注册器初始化"""
        self.assertIsNotNone(self.registry.container)
        self.assertIsNotNone(self.registry.config)
        self.assertIsInstance(self.registry.service_implementations, dict)
    
    def test_get_service_implementation_development(self):
        """测试开发环境下的服务实现获取"""
        # 设置为开发环境
        self.config.environment = Environment.DEVELOPMENT
        
        # 获取服务实现
        impl = self.registry._get_service_implementation('IAPIClient')
        
        self.assertIsNotNone(impl)
        self.assertEqual(impl.__name__, 'EnhancedCozeAPIClient')
    
    def test_get_service_implementation_testing(self):
        """测试测试环境下的服务实现获取"""
        # 设置为测试环境
        self.config.environment = Environment.TESTING
        
        # 获取服务实现（应该获取到Mock实现）
        impl = self.registry._get_service_implementation('IAPIClient')
        
        self.assertIsNotNone(impl)
        # 在测试环境下，应该获取到Mock实现
        self.assertEqual(impl.__name__, 'MockAPIClient')
    
    def test_get_service_implementation_unknown_interface(self):
        """测试未知接口的服务实现获取"""
        impl = self.registry._get_service_implementation('IUnknownService')
        self.assertIsNone(impl)
    
    @patch('core.service_registry.logger')
    def test_register_all_services_success(self, mock_logger):
        """测试成功注册所有服务"""
        # 模拟服务注册方法
        self.registry._register_infrastructure_services = Mock()
        self.registry._register_business_services = Mock()
        self.registry._register_core_services = Mock()
        
        # 执行注册
        self.registry.register_all_services()
        
        # 验证所有注册方法都被调用
        self.registry._register_infrastructure_services.assert_called_once()
        self.registry._register_business_services.assert_called_once()
        self.registry._register_core_services.assert_called_once()
    
    @patch('core.service_registry.logger')
    def test_register_all_services_failure(self, mock_logger):
        """测试服务注册失败"""
        # 模拟注册方法抛出异常
        self.registry._register_infrastructure_services = Mock(side_effect=Exception("注册失败"))
        
        # 执行注册，应该抛出异常
        with self.assertRaises(Exception):
            self.registry.register_all_services()
    
    def test_register_mock_services(self):
        """测试模拟服务注册"""
        # 设置为使用模拟服务
        self.config.environment = Environment.TESTING
        
        # 模拟注册方法
        self.registry._register_infrastructure_services = Mock()
        self.registry._register_business_services = Mock()
        self.registry._register_core_services = Mock()
        
        # 执行模拟服务注册
        self.registry._register_mock_services()
        
        # 验证注册方法被调用
        self.registry._register_infrastructure_services.assert_called_once()
        self.registry._register_business_services.assert_called_once()
        self.registry._register_core_services.assert_called_once()
    
    def test_should_use_mock_services(self):
        """测试是否应该使用模拟服务"""
        # 测试环境应该使用模拟服务
        self.config.environment = Environment.TESTING
        should_use_mock = self.config.should_use_mock_services()
        self.assertTrue(should_use_mock)
        
        # 开发环境不应该使用模拟服务
        self.config.environment = Environment.DEVELOPMENT
        should_use_mock = self.config.should_use_mock_services()
        self.assertFalse(should_use_mock)


if __name__ == '__main__':
    unittest.main()