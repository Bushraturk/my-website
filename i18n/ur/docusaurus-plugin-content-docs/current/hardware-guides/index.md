---
title: ہارڈ ویئر سیٹ اپ گائیڈ
sidebar_position: 100
---

# ہارڈ ویئر سیٹ اپ گائیڈ

یہ گائیڈ مختلف روبوٹکس ہارڈویئر سسٹم کو سیٹ اپ کرنے کے لیے ہے جن کا استعمال کورس کے دوران کیا جائے گا۔

## NVIDIA Jetson Development Kits

### Jetson Xavier NX

1. **پیکج کھولیں**: محفوظ طریقے سے Jetson Xavier NX Devit Kit کا پیکج کھولیں
2. **کمپوننٹس چیک کریں**: 
   - Jetson Xavier NX module
   - Carrier board
   - Power adapter
   - Fan
   - Mounting accessories
3. **کیریئر بورڈ پر ماڈیول ماؤنٹ کریں**: Jetson Xavier NX module کو کیریئر بورڈ پر ماؤنٹ کریں
4. **پاور کنیکشن**: Correct DC jack میں power adapter کنیکٹ کریں
5. **Fan کنیکٹ کریں**: Fan connector کو Xavier NX module کے ساتھ کنیکٹ کریں
6. **Boot کریں**: MicroSD card اور monitor، keyboard، mouse کنیکٹ کریں اور سسٹم boot کریں

### Jetson Orin

1. **پیکج کھولیں**: Jetson Orin Devit Kit کا پیکج کھولیں
2. **کمپوننٹس چیک کریں**:
   - Jetson AGX Orin module
   - Carrier board
   - Power supplies
   - Accessories
3. **ماڈیول کو سیٹ اپ کریں**: Module کو کیریئر بورڈ پر ماؤنٹ کریں
4. **Cooling Setup**: Thermal solution کو انسٹال کریں
5. **Peripheral Connections**: Display، input devices، network کنیکٹ کریں
6. **پاور اور بوٹ**: Correct power supply استعمال کریں اور سسٹم boot کریں

## Intel RealSense ڈیپتھ کیمرے

### RealSense D435/D435i

1. **Mounting Setup**: کیمرہ کو روبوٹ یا سٹینڈ پر mount کریں
2. **USB Connection**: High-speed USB 3.0 port سے کنیکٹ کریں
3. **Driver Installation**: 
   ```bash
   sudo apt install ros-humble-realsense2-camera
   ```
4. **Calibration**: Camera parameters کی calibration کریں
5. **Testing**: 
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

### RealSense L515

1. **Mounting**: کیمرہ کو مستحکم طریقے سے mount کریں
2. **USB-C Connection**: Power اور data کے لیے USB-C cable استعمال کریں
3. **Environment Considerations**: LiDAR کیمرہ direct sunlight سے محفوظ مقام پر استعمال کریں
4. **Integration**: ROS 2 pipeline کے ساتھ کیمرہ کو ضم کریں

## RTX ورک اسٹیشن سیٹ اپ

### نصاب کی ضروریات

- **Operating System**: Ubuntu 20.04 LTS یا 22.04 LTS
- **Graphics Driver**: Latest NVIDIA driver (470.0 یا اس سے زیادہ)
- **CUDA Toolkit**: CUDA 11.x یا 12.x
- **Docker**: For containerized simulation environments
- **Isaac Sim**: NVIDIA Isaac Sim installation

### CUDA اور NVIDIA Drivers انسٹال کرنا

```bash
# NVIDIA drivers انسٹال کریں
sudo apt update
sudo apt install nvidia-driver-535 nvidia-utils-535

# CUDA toolkit انسٹال کریں
wget https://developer.download.nvidia.com/compute/cuda/12.1.0/local_installers/cuda_12.1.0_530.30.02_linux.run
sudo sh cuda_12.1.0_530.30.02_linux.run
```

### Isaac Sim انسٹال کرنا

1. NVIDIA Developer portal سے Isaac Sim download کریں
2. Omniverse launcher install کریں
3. Isaac Sim app install کریں
4. System requirements verify کریں
5. License configuration کریں

## مطابقت پذیر روبوٹکس پلیٹ فارمز

### TurtleBot3

1. **Hardware Components**:
   - Waffle یا Burger model
   - OpenCR controller
   - LDS-01 LiDAR
   - Battery pack
2. **Assembly**: Parts کو تعمیر کی ہدایات کے مطابق اکٹھا کریں
3. **OpenCR Firmware**: Latest firmware flash کریں
4. **ROS 2 Setup**: 
   ```bash
   sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations
   ```

### Universal Robots (UR5/UR10)

1. **Safety Check**: Work envelope clear کریں
2. **Network Connection**: Ethernet connection through router
3. **Polyscope Interface**: Touch panel through web interface کا استعمال کریں
4. **ROS 2 Interface**: 
   ```bash
   git clone -b humble-devel https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
   ```

## Troubleshooting چیلنجز

### Jetson Issues

- **Boot Problems**: SD card image، power supply، display connections چیک کریں
- **Performance**: Thermal management، power mode، clock speeds چیک کریں
- **Connectivity**: Network، USB، UART interfaces چیک کریں

### کیمرہ Issues

- **No Detection**: Cable، drivers، permissions چیک کریں
- **Poor Quality**: Lighting، exposure، gain settings چیک کریں
- **Sync Issues**: Time stamps، frame rates، sync modes چیک کریں

### RTX Issues

- **Driver Compatibility**: ROS، Isaac Sim، other packages کے ساتھ driver version تصدیق کریں
- **Memory Issues**: GPU memory allocation، swap file configuration چیک کریں
- **Thermal Throttling**: Cooling، fan curves، case airflow چیک کریں

## سیٹ اپ کی تصدیق

1. **سیٹ اپ چیک لسٹ**:
   - [ ] Jetson board boot کر رہا ہے
   - [ ] Camera sensors detect ہو رہے ہیں
   - [ ] Network connectivity کام کر رہی ہے
   - [ ] ROS 2 installation کام کر رہا ہے
   - [ ] Isaac packages properly installed ہیں

2. **Basic Tests**:
   - [ ] `nvidia-smi` command GPU detect کر رہا ہے
   - [ ] `ros2 run` basic nodes run کر رہا ہے
   - [ ] Camera streams show correctly
   - [ ] Basic ROS 2 topics work کر رہے ہیں

## حفاظتی احتیاطیں

- **Power Supply**: Correct voltage، amperage، connector type استعمال کریں
- **Thermal Management**: Proper cooling، ventilation، thermal monitors maintain کریں
- **Electrostatic Discharge**: Ground straps، anti-static mats، proper handling use کریں
- **Physical Safety**: Workspace clear، emergency stops، safety protocols maintain کریں