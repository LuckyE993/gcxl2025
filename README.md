# gcxl2025
🫡 使用前确认 src\config\settings.py 中config的路径是否正确
## 模型训练
### 1. 数据集准备
可以使用 src\utils\dataCollect.py 中的数据采集功能。
### 2. 模型训练
``` bash
yolo train data="D:\YOLO\ultralytics\datasets\0414-2\data.yaml" model=yolo11n.yaml epochs=1000 imgsz=160 batch=64
```
### 3. 模型转换
.pt模型转换为 .onnx 模型
``` bash
yolo export model="runs\detect\0422-160\weights\best.pt" format=onnx imgsz=160 
```
.pt模型转换为 OpenVINO 模型 int8量化必须使用data.yaml中的数据集进行训练
``` bash
yolo export model="runs\detect\0422-160\weights\best.pt" format="openvino" imgsz=160 int8=True data="D:\YOLO\ultralytics\datasets\0414-2\data.yaml"
```
### 4. 模型验证
.pt模型验证
``` bash
yolo val model=path/to/best.pt data="D:\YOLO\ultralytics\datasets\0414-2\data.yaml" imgsz=320 device=cpu
```
.onnx模型验证
``` bash
yolo val task=detect model="D:\YOLO\ultralytics\runs\detect\train\weights\best.pt" imgsz=320 data=D:\YOLO\ultralytics\datasets\0414-2\data.yaml
```

# 测试实验
- [test_track](tests/test_track.py) 完成物料抓取精度测试 (重复N次)
```bash
sudo /home/car/miniconda3/envs/gcxl2025/bin/python -m tests.test_track
```

- [test_center](tests/test_center.py) 视觉伺服对准精度测试 (目标：物料块中心)
```bash
sudo /home/car/miniconda3/envs/gcxl2025/bin/python -m tests.test_center
``` 