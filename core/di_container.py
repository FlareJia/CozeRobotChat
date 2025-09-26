# core/di_container.py
import logging
import threading
from typing import Dict, Any, Type, Callable, Optional, TypeVar, Generic
from enum import Enum
from dataclasses import dataclass
from datetime import datetime

logger = logging.getLogger(__name__)

T = TypeVar('T')


class ServiceLifetime(Enum):
    """服务生命周期枚举"""
    SINGLETON = "singleton"  # 单例模式
    TRANSIENT = "transient"  # 每次创建新实例
    SCOPED = "scoped"       # 作用域内单例


@dataclass
class ServiceDescriptor:
    """服务描述符"""
    service_type: Type
    implementation_type: Optional[Type] = None
    factory: Optional[Callable] = None
    instance: Optional[Any] = None
    lifetime: ServiceLifetime = ServiceLifetime.TRANSIENT
    dependencies: Optional[list] = None
    created_at: Optional[datetime] = None


class DIContainer:
    """依赖注入容器"""
    
    def __init__(self):
        self._services: Dict[Type, ServiceDescriptor] = {}
        self._instances: Dict[Type, Any] = {}
        self._lock = threading.RLock()
        self._building_stack = set()  # 防止循环依赖
        
    def register_singleton(self, service_type: Type[T], implementation_type: Type[T] = None, 
                          factory: Callable[[], T] = None, instance: T = None) -> 'DIContainer':
        """注册单例服务"""
        return self._register_service(service_type, implementation_type, factory, instance, ServiceLifetime.SINGLETON)
    
    def register_transient(self, service_type: Type[T], implementation_type: Type[T] = None, 
                          factory: Callable[[], T] = None) -> 'DIContainer':
        """注册瞬态服务"""
        return self._register_service(service_type, implementation_type, factory, None, ServiceLifetime.TRANSIENT)
    
    def register_scoped(self, service_type: Type[T], implementation_type: Type[T] = None, 
                       factory: Callable[[], T] = None) -> 'DIContainer':
        """注册作用域服务"""
        return self._register_service(service_type, implementation_type, factory, None, ServiceLifetime.SCOPED)
    
    def _register_service(self, service_type: Type[T], implementation_type: Type[T] = None, 
                         factory: Callable[[], T] = None, instance: T = None, 
                         lifetime: ServiceLifetime = ServiceLifetime.TRANSIENT) -> 'DIContainer':
        """注册服务的内部方法"""
        with self._lock:
            if service_type in self._services:
                logger.warning(f"服务 {service_type.__name__} 已存在，将被覆盖")
            
            # 验证注册参数
            if sum(bool(x) for x in [implementation_type, factory, instance]) != 1:
                raise ValueError("必须且只能指定 implementation_type、factory 或 instance 中的一个")
            
            descriptor = ServiceDescriptor(
                service_type=service_type,
                implementation_type=implementation_type,
                factory=factory,
                instance=instance,
                lifetime=lifetime,
                created_at=datetime.now()
            )
            
            self._services[service_type] = descriptor
            
            # 如果是单例且提供了实例，直接存储
            if lifetime == ServiceLifetime.SINGLETON and instance is not None:
                self._instances[service_type] = instance
                
            logger.info(f"注册服务: {service_type.__name__} -> {lifetime.value}")
            return self
    
    def resolve(self, service_type: Type[T]) -> T:
        """解析服务"""
        with self._lock:
            return self._resolve_service(service_type)
    
    def _resolve_service(self, service_type: Type[T]) -> T:
        """解析服务的内部方法"""
        # 检查循环依赖
        if service_type in self._building_stack:
            raise RuntimeError(f"检测到循环依赖: {service_type.__name__}")
        
        # 检查服务是否已注册
        if service_type not in self._services:
            raise ValueError(f"服务 {service_type.__name__} 未注册")
        
        descriptor = self._services[service_type]
        
        # 单例模式：检查是否已有实例
        if descriptor.lifetime == ServiceLifetime.SINGLETON:
            if service_type in self._instances:
                return self._instances[service_type]
        
        # 创建实例
        self._building_stack.add(service_type)
        try:
            instance = self._create_instance(descriptor)
            
            # 存储单例实例
            if descriptor.lifetime == ServiceLifetime.SINGLETON:
                self._instances[service_type] = instance
                
            return instance
        finally:
            self._building_stack.discard(service_type)
    
    def _create_instance(self, descriptor: ServiceDescriptor) -> Any:
        """创建服务实例"""
        try:
            # 如果有预设实例，直接返回
            if descriptor.instance is not None:
                return descriptor.instance
            
            # 如果有工厂方法，使用工厂方法创建
            if descriptor.factory is not None:
                return descriptor.factory()
            
            # 使用实现类型创建实例
            if descriptor.implementation_type is not None:
                return self._create_with_dependencies(descriptor.implementation_type)
            
            raise ValueError(f"无法创建服务实例: {descriptor.service_type.__name__}")
            
        except Exception as e:
            logger.error(f"创建服务实例失败 {descriptor.service_type.__name__}: {str(e)}")
            raise
    
    def _create_with_dependencies(self, implementation_type: Type) -> Any:
        """根据依赖创建实例"""
        # 获取构造函数参数
        import inspect
        
        try:
            signature = inspect.signature(implementation_type.__init__)
            parameters = list(signature.parameters.values())[1:]  # 跳过 self 参数
            
            # 解析依赖
            dependencies = []
            for param in parameters:
                if param.annotation != inspect.Parameter.empty:
                    dependency = self._resolve_service(param.annotation)
                    dependencies.append(dependency)
                else:
                    logger.warning(f"参数 {param.name} 缺少类型注解，跳过依赖注入")
            
            return implementation_type(*dependencies)
            
        except Exception as e:
            logger.error(f"依赖注入创建实例失败 {implementation_type.__name__}: {str(e)}")
            # 尝试无参数创建
            try:
                return implementation_type()
            except Exception as fallback_error:
                logger.error(f"无参数创建也失败: {str(fallback_error)}")
                raise e
    
    def is_registered(self, service_type: Type) -> bool:
        """检查服务是否已注册"""
        return service_type in self._services
    
    def get_registered_services(self) -> Dict[str, Dict[str, Any]]:
        """获取已注册的服务信息"""
        with self._lock:
            result = {}
            for service_type, descriptor in self._services.items():
                result[service_type.__name__] = {
                    'lifetime': descriptor.lifetime.value,
                    'has_instance': service_type in self._instances,
                    'created_at': descriptor.created_at.isoformat() if descriptor.created_at else None,
                    'implementation': descriptor.implementation_type.__name__ if descriptor.implementation_type else 'Factory/Instance'
                }
            return result
    
    def clear(self) -> None:
        """清除所有注册的服务"""
        with self._lock:
            # 清理实例
            for instance in self._instances.values():
                if hasattr(instance, 'cleanup') and callable(getattr(instance, 'cleanup')):
                    try:
                        instance.cleanup()
                    except Exception as e:
                        logger.error(f"清理实例时出错: {str(e)}")
            
            self._services.clear()
            self._instances.clear()
            self._building_stack.clear()
            logger.info("DI容器已清除")
    
    def cleanup(self) -> None:
        """清理容器（clear方法的别名）"""
        self.clear()
    
    def __del__(self):
        """析构函数"""
        try:
            self.clear()
        except Exception:
            pass  # 忽略析构时的错误