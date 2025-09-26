#!/usr/bin/env python3
# tests/run_tests.py
"""测试运行脚本"""

import sys
import os
import unittest
import logging
from pathlib import Path

# 添加项目根目录到Python路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))


def setup_test_logging():
    """设置测试日志"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def discover_and_run_tests():
    """发现并运行所有测试"""
    # 设置测试日志
    setup_test_logging()
    
    # 获取测试目录
    test_dir = Path(__file__).parent
    
    # 发现测试
    loader = unittest.TestLoader()
    suite = loader.discover(
        start_dir=str(test_dir),
        pattern='test_*.py',
        top_level_dir=str(project_root)
    )
    
    # 运行测试
    runner = unittest.TextTestRunner(
        verbosity=2,
        stream=sys.stdout,
        buffer=True
    )
    
    print(f"\n{'='*60}")
    print("开始运行依赖注入系统测试")
    print(f"{'='*60}\n")
    
    result = runner.run(suite)
    
    print(f"\n{'='*60}")
    print("测试结果汇总:")
    print(f"运行测试数: {result.testsRun}")
    print(f"失败数: {len(result.failures)}")
    print(f"错误数: {len(result.errors)}")
    print(f"跳过数: {len(result.skipped)}")
    
    if result.failures:
        print("\n失败的测试:")
        for test, traceback in result.failures:
            newline = '\n'
            print(f"  - {test}: {traceback.split(newline)[-2]}")
    
    if result.errors:
        print("\n错误的测试:")
        for test, traceback in result.errors:
            newline = '\n'
            print(f"  - {test}: {traceback.split(newline)[-2]}")
    
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\n成功率: {success_rate:.1f}%")
    print(f"{'='*60}\n")
    
    return result.wasSuccessful()


def run_specific_test(test_name):
    """运行特定的测试"""
    setup_test_logging()
    
    # 导入测试模块
    test_module = __import__(f'tests.{test_name}', fromlist=[test_name])
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(test_module)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


def main():
    """主函数"""
    if len(sys.argv) > 1:
        # 运行特定测试
        test_name = sys.argv[1]
        success = run_specific_test(test_name)
    else:
        # 运行所有测试
        success = discover_and_run_tests()
    
    # 返回适当的退出码
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()