---


sidebar_position: 3
difficulty: beginner


---

# 1.3: ROS 2 تنصیب اور سیٹ اپ

## جائزہ

یہ ذیلی ماڈیول ROS 2 کی تنصیب کا عمل اور ترقی کے ماحول کو ایڈجسٹ کرنے کا طریقہ سمجھاتا ہے۔

## سیکھنے کے مقاصد

اس ذیلی ماڈیول کے اختتام تک، آپ کریں گے:
- ROS 2 تنصیب کے لیے ضروریات کو سمجھیں
- ROS 2 مختلف آپریٹنگ سسٹم پر تنصیب کریں
- ROS 2 ماحولیاتی متغیرات کو تشکیل دیں
- ابتدائی ROS 2 ایپلی کیشن کو چلائیں

## ROS 2 تنصیب کی ضروریات

### سسٹم کی ضروریات

- **آپریٹنگ سسٹم**:
  - Ubuntu 20.04 (فocal) یا 22.04 (jammy)
  - Windows 10/11 (کے ساتھ WSL2)
  - macOS 11+ (Big Sur یا اس سے اوپر)

- **ہارڈویئر**:
  - چار-core 2GHz CPU یا اس سے بہتر
  - 8GB RAM یا اس سے زیادہ
  - 5GB فری اسٹوریج سپیس

- **سافٹ ویئر**:
  - Python 3.8 یا اس سے جدید
  - pip (Python package installer)
  - Git

### ROS 2 تنصیب کے اختیارات

ROS 2 کو متعدد طریقوں سے تنصیب کیا جا سکتا ہے:

1. **APT (Ubuntu)**: سرکاری Ubuntu repositories کے ذریعے
2. **Chocolatey (Windows)**: Windows کے لیے پیکیج مینجمنٹ سسٹم
3. **Bottles (macOS)**: macOS کے لیے ROS 2 bottles
4. **Docker**: کنٹینرائزڈ ROS 2 ماحول
5. **Source**: ROS 2 کو سورس کوڈ سے کمپائل کریں

## Ubuntu پر ROS 2 تنصیب

### Debian/Ubuntu کے لیے APT ذریعہ شامل کریں

```bash
# GPG key شامل کریں
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# APT ذریعہ شامل کریں
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### ROS 2 تنصیب کریں

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### ROS 2 ماحولیاتی متغیرات تشکیل دیں

```bash
# سیشن کے لیے ماحولیاتی متغیرات شامل کریں
source /opt/ros/humble/setup.bash

# مستقل بنیادوں پر ماحولیاتی متغیرات شامل کریں
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Windows پر ROS 2 تنصیب

### WSL2 تنصیب

1. Windows فیچر: Linux Kernel اپ ڈیٹ کریں
2. WSL2 انسٹال کریں
3. Ubuntu انسٹال کریں (Microsoft Store سے)
4. WSL2 کے لیے ڈیفالٹ ورژن کے طور پر سیٹ کریں

```powershell
wsl --set-default-version 2
```

### ROS 2 داخل کریں

WSL Ubuntu میں داخل ہوں اور پہلے والے Ubuntu کے اقدامات کی پیروی کریں:

```bash
# WSL terminal میں
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

## macOS پر ROS 2 تنصیب

### Homebrew کے ذریعے

```bash
# Homebrew اگر ابھی تک انسٹال نہیں ہے
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Python3 اور pip انسٹال کریں
brew install python3

# ROS 2 bottle انسٹال کریں
brew install ros/humble/humble-desktop
source /opt/ros/humble/setup.bash
```

## Docker کا استعمال کرکے ROS 2

Docker ROS 2 کا استعمال کرنے کا ایک مؤثر طریقہ ہے:

```bash
# ROS 2 Docker image ڈاؤن لوڈ کریں
docker pull ros:humble

# ایک interactive container شروع کریں
docker run -it ros:humble

# container کے اندر
source /opt/ros/humble/setup.bash
ros2 --version
```

## تنصیب کی تصدیق

نصیب کی تصدیق کے لیے، ROS 2 کمانڈز چلائیں:

```bash
# ROS 2 ورژن چیک کریں
ros2 --version

# ROS 2 pkgs لسٹ کریں
ros2 pkg list

# rviz2 چلائیں
rviz2
```

## ROS 2 Development Environment کی ترتیب

### کمپائلر کی تنصیب

```bash
# Ubuntu
sudo apt install build-essential cmake pkg-config

# Windows (WSL)
sudo apt install build-essential cmake pkg-config

# macOS
xcode-select --install
```

### IDE کا انتخاب

- **VSCode**: ROS 2 کے لیے وسیع تر سپورٹ کے ساتھ
- **Qt Creator**: C++ ROS 2 development کے لیے
- **PyCharm**: Python ROS 2 development کے لیے

### ROS 2 workspace تشکیل

```bash
# workspace directory بنا دیں
mkdir -p ~/ros2_ws/src

# source کریں
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# colcon build
colcon build

# workspace سیشن کے لیے source کریں
source install/setup.bash
```

## خلاصہ

اس ذیلی ماڈیول نے ROS 2 کی متعدد آپریٹنگ سسٹم پر تنصیب کا عمل، ترتیب اور تصدیق کا طریقہ سمجھایا ہے۔ اب آپ ROS 2 کے ساتھ کام کرنے کے لیے تیار ہیں۔ اگلا ذیلی ماڈیول ROS 2 کی ابتدائی مشقیں پیش کرے گا۔