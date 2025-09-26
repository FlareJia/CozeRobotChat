# tests/test_di_config.py
import unittest
from unittest.mock import patch
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

from config import DIConfig, Environment


class TestDIConfig(unittest.TestCase):
    """依赖注入配置测试"""
    
    def setUp(self):
        """测试前准备"""
        self.config = DIConfig()
    
    def test_default_environment(self):
        """测试默认环境"""
        self.assertEqual(self.config.environment, Environment.DEVELOPMENT)
    
    def test_set_environment(self):
        """测试设置环境"""
        self.config.environment = Environment.PRODUCTION
        self.assertEqual(self.config.environment, Environment.PRODUCTION)
    
    def test_get_service_config_audio_service(self):
        """测试获取音频服务配置"""
        config = self.config.get_service_config('audio_service')
        
        self.assertIsInstance(config, dict)
        self.assertIn('buffer_size', config)
        self.assertIn('sample_rate', config)
        self.assertIn('channels', config)
        self.assertEqual(config['buffer_size'], 1024)
        self.assertEqual(config['sample_rate'], 16000)
        self.assertEqual(config['channels'], 1)
    
    def test_get_service_config_api_client(self):
        """测试获取API客户端配置"""
        config = self.config.get_service_config('api_client')
        
        self.assertIsInstance(config, dict)
        self.assertIn('timeout', config)
        self.assertIn('max_retries', config)
        self.assertIn('base_url', config)
        self.assertEqual(config['timeout'], 30)
        self.assertEqual(config['max_retries'], 3)
    
    def test_get_service_config_conversation(self):
        """测试获取对话配置"""
        config = self.config.get_service_config('conversation')
        
        self.assertIsInstance(config, dict)
        self.assertIn('max_history', config)
        self.assertIn('context_window', config)
        self.assertEqual(config['max_history'], 10)
        self.assertEqual(config['context_window'], 4000)
    
    def test_get_service_config_unknown(self):
        """测试获取未知服务配置"""
        config = self.config.get_service_config('unknown_service')
        self.assertEqual(config, {})
    
    def test_get_service_implementation_development(self):
        """测试开发环境服务实现"""
        self.config.environment = Environment.DEVELOPMENT
        
        # 测试音频设备实现
        impl = self.config.get_service_implementation('IAudioDevice')
        self.assertEqual(impl, 'RobotAudioInterface')
        
        # 测试API客户端实现
        impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(impl, 'EnhancedCozeAPIClient')
    
    def test_get_service_implementation_testing(self):
        """测试测试环境服务实现"""
        self.config.environment = Environment.TESTING
        
        # 测试音频设备实现（应该使用Mock）
        impl = self.config.get_service_implementation('IAudioDevice')
        self.assertEqual(impl, 'MockAudioDevice')
        
        # 测试API客户端实现（应该使用Mock）
        impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(impl, 'MockAPIClient')
    
    def test_get_service_implementation_production(self):
        """测试生产环境服务实现"""
        self.config.environment = Environment.PRODUCTION
        
        # 测试音频设备实现
        impl = self.config.get_service_implementation('IAudioDevice')
        self.assertEqual(impl, 'RobotAudioInterface')
        
        # 测试API客户端实现
        impl = self.config.get_service_implementation('IAPIClient')
        self.assertEqual(impl, 'EnhancedCozeAPIClient')
    
    def test_get_service_implementation_unknown(self):
        """测试未知接口实现"""
        impl = self.config.get_service_implementation('IUnknownInterface')
        self.assertIsNone(impl)
    
    def test_should_use_mock_services_development(self):
        """测试开发环境是否使用模拟服务"""
        self.config.environment = Environment.DEVELOPMENT
        self.assertFalse(self.config.should_use_mock_services())
    
    def test_should_use_mock_services_testing(self):
        """测试测试环境是否使用模拟服务"""
        self.config.environment = Environment.TESTING
        self.assertTrue(self.config.should_use_mock_services())
    
    def test_should_use_mock_services_production(self):
        """测试生产环境是否使用模拟服务"""
        self.config.environment = Environment.PRODUCTION
        self.assertFalse(self.config.should_use_mock_services())
    
    @patch.dict('os.environ', {'ENVIRONMENT': 'testing'})
    def test_environment_from_env_var(self):
        """测试从环境变量读取环境配置"""
        # 重新创建配置对象以读取环境变量
        config = DIConfig()
        # 注意：这里需要在DIConfig中实现从环境变量读取的逻辑
        # 目前的实现中没有这个功能，这是一个潜在的改进点
        pass


if __name__ == '__main__':
    unittest.main()