#!/bin/bash

# 检查是否以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "此脚本需要root权限运行。请使用sudo执行此脚本。"
    exit 1
fi

echo "开始安装依赖..."
echo "更新系统包索引..."
apt-get update -y

echo "安装系统级依赖..."
# 安装PortAudio开发库和其他潜在依赖
apt-get install -y \
    portaudio19-dev \
    python3-dev \
    build-essential \
    ccache

echo "安装Python包依赖..."
# 使用requirements.txt文件安装Python依赖
pip install -r requirements.txt

echo "依赖安装完成！"
