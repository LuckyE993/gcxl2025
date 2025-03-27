"""
主模块测试。
"""
import unittest
from src import main

class TestMain(unittest.TestCase):
    """测试主模块"""
    
    def test_main_function(self):
        """测试main函数返回值"""
        result = main.main()
        self.assertEqual(result, 0)

if __name__ == "__main__":
    unittest.main()
