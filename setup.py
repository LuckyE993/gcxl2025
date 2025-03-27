"""
设置脚本，用于安装和分发包。
"""
from setuptools import setup, find_packages

setup(
    name="gcxl2025",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "opencv-python",
        "numpy",
        "ultralytics",
    ],
    author="Qixuan Sun",
    author_email="luckyeureka52@gmail.com",
    description="GCXL2025",
    keywords="python, computer vision, deep learning, robotics",
    url="https://github.com/LuckyE993/gcxl2025",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.10",
)
