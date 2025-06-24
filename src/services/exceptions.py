from typing import Optional, Dict, Any

class BaseError(Exception):
    """基础异常类
    
    所有自定义异常的基类，提供统一的错误信息格式和错误码支持。
    """
    def __init__(self, message: str, error_code: Optional[str] = None, details: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}

class AudioError(BaseError):
    """音频相关错误
    
    用于处理音频设备初始化、播放、录制等相关操作时发生的错误。
    """
    def __init__(self, message: str, error_code: Optional[str] = None, device_info: Optional[Dict[str, Any]] = None):
        details = {"device_info": device_info} if device_info else {}
        super().__init__(message, error_code, details)

class APIError(BaseError):
    """API相关错误
    
    用于处理API请求过程中发生的错误，包括网络请求、响应解析等。
    """
    def __init__(self, message: str, status_code: Optional[int] = None, error_code: Optional[str] = None):
        details = {"status_code": status_code} if status_code else {}
        super().__init__(message, error_code, details)
        self.status_code = status_code

class SystemError(BaseError):
    """系统相关错误
    
    用于处理系统资源、进程、环境等相关的错误。
    """
    def __init__(self, message: str, error_code: Optional[str] = None, system_info: Optional[Dict[str, Any]] = None):
        details = {"system_info": system_info} if system_info else {}
        super().__init__(message, error_code, details)

class FileError(BaseError):
    """文件系统相关错误
    
    用于处理文件读写、权限、存储空间等相关的错误。
    """
    def __init__(self, message: str, error_code: Optional[str] = None, file_info: Optional[Dict[str, Any]] = None):
        details = {"file_info": file_info} if file_info else {}
        super().__init__(message, error_code, details)

class NetworkError(BaseError):
    """网络相关错误
    
    用于处理网络连接、DNS解析、超时等相关的错误。
    """
    def __init__(self, message: str, error_code: Optional[str] = None, network_info: Optional[Dict[str, Any]] = None):
        details = {"network_info": network_info} if network_info else {}
        super().__init__(message, error_code, details)