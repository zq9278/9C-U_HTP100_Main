# SLK-01 PyQt 调试上位机

这是给当前固件 USART2 屏幕/调试协议使用的桌面调试工具。

## 安装

```powershell
cd tools\pyqt_debug_host
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

## 运行

```powershell
python slk_debug_host.py
```

默认串口参数来自固件 `Core/Src/usart.c`：

- 波特率：115200
- 数据位：8
- 校验：None
- 停止位：1

## 已支持功能

- 串口连接、断开、自动刷新串口号。
- 工作模式控制：加热、挤压、自动模式、停止、完成信号、工厂模式。
- 目标温度、目标压力下发。
- 温度 PID 和压力/电机 PID 手动调参。
- 预设参数读写、选择、清 EEPROM、清眼部 EEPROM。
- 实时解析下位机上报数据，并显示温度曲线、压力曲线、电量、眼部检测和计时/退出事件。
- 原始收发帧日志，便于对照固件协议排查。

说明：固件目前只在对应工作状态下主动上报温度/压力，所以上位机曲线依赖设备实时上报帧。
