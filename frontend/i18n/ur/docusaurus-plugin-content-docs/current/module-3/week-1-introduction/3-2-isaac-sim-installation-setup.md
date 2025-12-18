---


sidebar_position: 2
difficulty: intermediate


---
# 3.2: آئزک ساسم اوتنصیب اوسترتب کے لِلیسس لِس ایس اِس ایس ایس آروس 2

## ج a ج

یہ یہ فصی یہ یہ یہ یہ ے ے فصی ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ j کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ ہ کے کے کے کے کے کے کے کے کے کے کے کے کے اواور تالسل ک ک ک آپ ک ک a maausol کے lli rewauss smowlain.

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- مامتل ط ط rauc (ڈ vaur ، آ bai ئی) ک a ک a amaal ک samali ہ Ou ئے آئزک saum anssaul asr یں
- ROS 2 انضمام کے saatھ آئزک saumm
- سیٹ اوشر کو مطلوبہ سیسم انحصار
- بن ی اڈاد ی ٹی sauch ک so ک v ک ِ ِ ِ ِ ِ ِ ِ ِ ِ ک ک
- ک araprd گی کی a ص laa ح ک v ssmau کے l یے saum
- مشترکہ تنب کے مسائل کو دور کرنا

## ss ی sm کی ض ض ض rorur ی at

### ہ اعاری ووور کی ض rorurautat

پہ پہ آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک ک ک ک ک ک ک ک ، ، ، ک ک ک ک ک ک ی Buna ئیں کہ/ک ک ک ک ک ک a/کی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک

** ک m ss ے ک m ata ق a ضے: **
.
- ** vram **: 8GB ک M SS ے ک m
.
- ** رام **: 16 جی بی سیسم میموری
- ** اسٹوریج **: 20 جی بی مفت جگہ
- ** ڈسپلے **: 1920x1080 ی ہائر ریزولوشن

** تحق یز ک rd ہ ta ق a ضے: **
- ** جی پی ی v **: nvidia rtx ss ی r یز
- ** vram **: 24GB+
.
- ** رام **: 32 جی بکی+
.
- **Display**: 4K یا higher کے لیے optimal experience

مست

.
- **NVIDIA GPU Driver**: 470.63.01 یا newer
.
.
.

### gpu maubahat چیک

پہ l ے jpu maUab ق t کی ta ص d یق ک r یں:
```bash
# Check اگر NVIDIA GPU ہے detected
nvidia-smi

# Check CUDA version اور compatibility
nvcc --version

# Verify compute capability (minimum 6.0)
nvidia-smi --query-gpu=name,compute_cap --format=csv
```
## تنی بِن کے ط ط r یقے

### ط r یقہ 1: ڈ waur tn ی bib (taِsso ک ک rd ہ)

ک a/کی ڈ ڈ ڈ ڈ ڈ ڈ ک ک ک ک ک a/کی زی زی زی زی زی tt tt tt er ssad ھ a sasadasadas anah snas an ظ خ خ خ خ ط ط ط پ پ پ پ پ کے کے کے کے کے کے کے کے

#####
```bash
# Update پیکیج index
sudo apt update

# Install prerequisites
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set اوپر کا/کی repository
echo \
  "deb [arch=$ signed-کے ذریعے=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update پیکیج index again
sudo apt update

# Install Docker Engine
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add آپ کا user کو کا/کی docker group
sudo usermod -aG docker $USER

# Log out اور log پیچھے میں کے لیے group changes کو take effect
```
#### nvidia ک n ٹی inr ٹ Oul کٹ anssacasal ari یں
```bash
# Add NVIDIA's GPG key اور repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Update پیکیج index
sudo apt update

# Install nvidia-container-toolkit
sudo apt install -y nvidia-container-toolkit

# Restart Docker daemon
sudo systemctl restart docker
```
#####
```bash
# Create directories کے لیے persistent data
mkdir -p ~/isaac_sim_exts
mkdir -p ~/isaac_assets

# Run آئزک سیم container
xhost +local:docker

docker run --gpus تمام -یہ --rm \
  --name isaac_sim \
  --net=host \
  -e "ACCEPT_EULA=Y" \
  -e "NVIDIA_VISIBLE_DEVICES=تمام" \
  -e "NVIDIA_DRIVER_CAPABILITIES=تمام" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/isaac_sim_exts:/home/isaac-sim/.nvidia-omniverse/extensions \
  -v ~/.nvidia-omniverse/logs:/home/isaac-sim/.nvidia-omniverse/logs \
  -v ~/isaac_assets:/home/isaac-sim/isaac_assets \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --shm-size=2g \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  nvcr.io/nvidia/isaac-sim:4.0.0
```
### ط RI یقہ 2: آ BAA ئی شہ r

کے l ِ s maum ڈ ص ص ڈ ص ص ari فی n ک o bura ہ ہ Jrsaa ئی ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض ض

#####
```bash
# Install سسٹم dependencies
sudo apt update
sudo apt install build-essential cmake git libssl-dev libffi-dev python3-dev python3-pip

# Install ROS 2 dependencies
sudo apt install python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Install Omniverse Launcher
# Download سے https://www.nvidia.com/en-ہمیں/omniverse/download/
```
#### Oawumnaiursiی Llonahr کے tssss ssیs آئزک ssaum anssacahal arsna

1.
2. GUI ہ DAA ی at کے baesd_laausonahr کے ذ ذ ذ raua ے inssaul alsr یں
3. اوسوروس لونشر لون چ -
4. سللا ائی کے l یے
5

بعد تونب ، آئزک آئزک آئزک آئزک آئزک آئزک آئزک گ گ گ گ گ گ گ گ گ گ گ گ گ
`~/.nvidia-omniverse/launcher/apps/omniverse.isaac.sim.gfx`
## ros 2 anaumamam atrt ی b

### ros 2 پ l پ پ tttttttt tt tt tt tt tt tt tt tt tt tt ت پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ

ک a/کی ros 2 br ج ہے ض ض ض ض کے کے کے ll یے آئزک saum ک o ک Aam کے SAAT ھ ROS ROS ROS ROS ROS ROS ROS ROS 2.

#### کے l یے l یے ڈvaur جون صی b

ک a/کی ش کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی ے ے کی کی کی کی کی کی کی کی ے ے ے کی کی کی کی کی کی کی کی کی کی کی ک کی کی ک ک ک ک ک ک ک ک ک ک ک
```bash
# میں کا/کی آئزک سیم container
cd /isaac-sim/python.sh -c "import omni; omni.kit.commands.execute('ExtensionManagerInstall', extension="omni.isaac.ros2_bridge.humble")"
```
#### کے l یے l یے آ baa ئی t n صی b
```bash
# Find آئزک سیم تنصیب path
ISAAC_SIM_PATH=$HOME/.nvidia-omniverse/launcher/apps/omniverse.isaac.sim.gfx

# Enable کا/کی ROS2 bridge extension
cd $ISAAC_SIM_PATH
./isaac-sim.sh --enable-extensions omni.isaac.ros2_bridge.humble
```
### ros 2 maaasol atrtaib

ماما خذ خذ خذ کے ذ ک ک swr ی s/ک صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح صحیح ق ق saum آئزک saum آئزک saum maausol:
```bash
# کے لیے ROS 2 Humble
export ISAAC_SIM_PATH=$HOME/.nvidia-omniverse/launcher/apps/omniverse.isaac.sim.gfx

# Source ROS 2
source /opt/ros/humble/ترتیب.bash

# Source آئزک سیم ماحول (کے لیے native تنصیب)
source $ISAAC_SIM_PATH/setup_conda_env.sh
```
## تال الاور کی تو

### bin ی AD ی TAAL

یک تالسل السالپٹ بنییئیں - سیسس اوسر آپ ک ک ک ک ک ک ک ک ک ک ک ک ک ک
```bash
# ~/.isaac_sim_config.sh
export ISAAC_SIM_PATH=$HOME/.nvidia-omniverse/launcher/apps/omniverse.isaac.sim.gfx

# ROS 2 ترتیب
source /opt/ros/humble/ترتیب.bash

# آئزک سیم ترتیب (کے لیے native تنصیب)
اگر [ -f "$ISAAC_SIM_PATH/setup_conda_env.sh" ]; پھر
    source $ISAAC_SIM_PATH/setup_conda_env.sh
fi

# Add آئزک سیم Python modules کو path
export PYTHONPATH="${ISAAC_SIM_PATH}/python:${PYTHONPATH}"

# NVIDIA GPU settings
export NVIDIA_VISIBLE_DEVICES=تمام
export NVIDIA_DRIVER_CAPABILITIES=تمام
```
Add کو آپ کا `.bashrc`:
```bash
echo "source ~/.isaac_sim_config.sh" >> ~/.bashrc
source ~/.bashrc
```
### twu کے ٹی s ٹ

#### ٹی S ٹ 1: بن ی AD ی آئزک saumlaun چ
```bash
# کے لیے Docker تنصیب, آئزک سیم ہے already running
# کے لیے native تنصیب:
$ISAAC_SIM_PATH/isaac-sim.sh
```
#### ٹی S ٹ 2: ROS 2 پ پ پ پ ف ف ف ف ف ف ف پ پ پ پ پ پ پ پ پ پ پ پ پ پ
Create ایک test script `test_isaac_sim_ros2.py`:
```python
#!/usr/bin/env python3

# Test script کو verify آئزک سیم ROS 2 integration
import sys
import time
import rclpy
سے geometry_msgs.msg import Twist
سے sensor_msgs.msg import LaserScan
سے std_msgs.msg import String

def test_ros2_connection():
    """Test basic ROS 2 functionality میں آئزک سیم"""
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create ایک نود
    نود = rclpy.create_node('isaac_sim_tester')
    
    # Create پبلشر کے لیے velocity commands
    cmd_vel_pub = نود.create_publisher(Twist, '/cmd_vel', 10)
    
    # Create سبسکرائیبر کے لیے laser scan
    def scan_callback(msg):
        print} points")
    
    scan_sub = نود.create_subscription(LaserScan, '/scan', scan_callback, 10)
    
    # Test publishing
    twist = Twist()
    twist.linear.x = 0.1
    twist.angular.z = 0.05
    
    # Publish ایک few messages
    کے لیے میں میں range(5):
        cmd_vel_pub.publish(twist)
        time.sleep(0.5)
    
    # Wait کے لیے messages
    rclpy.spin_once
    
    # Cleanup
    نود.destroy_node()
    rclpy.shutdown()
    print("ROS 2 test completed successfully!")

اگر __name__ == '__main__':
    test_ros2_connection()
```
#### ٹی S ٹ 3: آئزک Saum پ aa ئی tahn api
Create ایک test script `test_isaac_sim_api.py`:
```python
#!/usr/bin/env python3

# Test آئزک سیم Python API
import sys
import os

def add_isaac_sim_path():
    """Add آئزک سیم Python modules کو path"""
    # Add آئزک سیم Python path
    isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
    اگر isaac_sim_path:
        sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
        
        # Add آئزک سیم Kit path
        kit_path = os.path.join(isaac_sim_path, 'kit')
        sys.path.insert(0, kit_path)
        
        # Add آئزک سیم Apps path
        apps_path = os.path.join(isaac_sim_path, 'apps')
        sys.path.insert(0, apps_path)

# Add آئزک سیم paths پہلے importing
add_isaac_sim_path()

try:
    # Test آئزک سیم imports
    سے omni.isaac.core import World
    سے omni.isaac.core.utils.stage import add_reference_to_stage
    سے omni.isaac.core.objects import DynamicCuboid
    سے omni.isaac.core.utils.nucleus import find_nucleus_server
    سے omni.isaac.core.utils.carb import set_carb_setting
    
    
    # Test basic functionality
    
    # Create ایک world
    world = World(stage_units_in_meters=1.0)
    print("✓ World created successfully")
    
    # Add ایک object
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/Cube",
            name="test_cube",
            position=[0, 0, 1.0],
            size=0.5,
            mass=1.0
        )
    )
    
    # Reset کا/کی world
    world.reset()
    print("✓ World reset successfully")
    
    # Cleanup
    world.clear()
    
except ImportError کے طور پر e:
except Exception کے طور پر e:

# Test ROS 2 bridge imports اگر installed
try:
    import omni
    # Try کو enable کا/کی ROS2 bridge extension
    print("Testing ROS2 bridge extension...")
    # یہ کرے گا typically ہونا done through آئزک سیم UI یا extension manager
    print("✓ ROS2 bridge extension test completed")
except Exception کے طور پر e:
```
### ک a/کی ٹی s ٹ ٹی si ٹ چ sl رال ہے
```bash
# Make scripts executable
chmod +x test_isaac_sim_ros2.py
chmod +x test_isaac_sim_api.py

# Run API test
python3 test_isaac_sim_api.py

# Run ROS 2 test
python3 test_isaac_sim_ros2.py
```
## ک اراورڈ گی کی کی ص ص ص ص ص کی

### آئزک آئزک saum کی atratiagaat کے l یے زی ad ہ s ے زی ad ہ ک araird گی

i یک آئزک آئزک آئزک sisum at شکی l ف aaul bnaa ئیں ک arard گی ک s
```bash
# Create آئزک سیم config directory اگر یہ doesn't exist
mkdir -p ~/.nvidia-omniverse/config/Isaac-Sim-2023.1.1/

# Create تشکیل file
cat > ~/.nvidia-omniverse/config/Isaac-Sim-2023.1.1/config.yaml << EOF
app:
  window:
    height: 1080
    width: 1920
  template: core.template.kit
  enable_audio: false
  enable_gpu_fallback: true

physics:
  fixed_timestep: 0.008333  # 120 Hz
  solver_type: 0  # 0=PBD, 1=PGS
  solver_position_iteration_count: 4
  solver_velocity_iteration_count: 1

rendering:
  enabled: true
  render_mode: 2  # 0=off, 1=پر-demand, 2=always
  resolution:
    width: 1920
    height: 1080

nvtx:
  enabled: false

carb:
  profiling:
    enable_profiling: false

omni:
  replicator:
    isRecording: false

hydra:
  scene_query_provider: "scene_query_provider_cpu"

rtx:
  quality:
    preset: "balanced"
  denoise:
    enable: false
EOF
```
### gpu maumur ی ​​کی کی ص ص ص ص ص کی ص ص

g gpu mumumvr ی کے کے کے کے کے کے کے کے کے کے کے کے کے کے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک کے کے کے کے
```bash
# Set ماحول variables کے لیے GPU optimization
export RTX_GLOBAL_HEAP_SIZE_MB=4096
export RTX_MIN_FREE_HEAP_SIZE_MB=1024
export RTX_GLOBAL_TEXTURE_CACHE_SIZE_MB=2048
```
## اعم مسعسل ک a aiaalہ ا ہ ہ ہ ہ ہ ثر

### mssaulہ 1: gpu ک a پ at ہ چ la
```bash
# Check NVIDIA driver status
nvidia-smi

# اگر GPU ہے نہیں detected:
# 1. Reinstall NVIDIA drivers
sudo apt remove --purge nvidia-*
sudo apt autoremove
sudo apt update
sudo apt install nvidia-driver-535  # Use appropriate version

# 2. Reboot سسٹم
sudo reboot
```
### ش ش ی ی 2: آئزک ساسم السیسس ک و لونچ
```bash
# Check display settings
echo $DISPLAY

# کے لیے X11 forwarding یا Docker, ensure X11 ہے properly configured
xhost +local:docker

# Check NVIDIA container runtime
docker run --rm --gpus تمام nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi
```
### ش mauriی 3: روس 2 برج نِنن لوکن
```bash
# Check اگر extension ہے installed
# میں آئزک سیم: Window -> Extensions

# Manual extension activation
# Add یہ lines کو آپ کا آئزک سیم startup script:
# import omni
# omni.kit.app.get_app_interface().get_extension_manager().set_extension_enabled("omni.isaac.ros2_bridge.humble", True)
```
### Massaulہ 4: ک اراپرڈ گی کے Massa ئ l
```bash
# Reduce سمولیشن fidelity
# میں آئزک سیم: Window -> Compute Graph Editor -> Adjust physics اور rendering settings

# یا modify config file کے ساتھ lower settings:
# physics:
# fixed_timestep: 0.016667  # 60 Hz (lower fidelity)
```
## maa ؤ l کی thochr پٹ
Create ایک comprehensive validation script `isaac_sim_validation.py`:
```python
#!/usr/bin/env python3
"""
Comprehensive آئزک سیم اور ROS 2 validation script
"""

import os
import sys
import subprocess
import time
import rclpy
سے std_msgs.msg import String
سے geometry_msgs.msg import Twist

def check_system_requirements():
    """Check سسٹم requirements"""
    
    # Check OS
    import platform
    print} {platform.release()}")
    
    # Check Python version
    print(f"Python version: {platform.python_version()}")
    
    # Check NVIDIA GPU
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
        اگر result.returncode == 0:
            print("✓ NVIDIA GPU detected")
            print(result.stdout.split('\n')[9:12])  # Show GPU info
        else:
    except FileNotFoundError:
    
    print()

def check_isaac_sim_installation():
    """Check اگر آئزک سیم ہے properly installed"""
    print("=== آئزک سیم تنصیب Check ===")
    
    # Check اگر آئزک سیم path exists
    isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
    اگر isaac_sim_path اور os.path.exists(isaac_sim_path):
        
        # Check کے لیے essential files
        essential_files = [
            os.path.join(isaac_sim_path, 'isaac-sim.sh'),
            os.path.join(isaac_sim_path, 'python'),
            os.path.join(isaac_sim_path, 'kit')
        ]
        
        کے لیے file میں essential_files:
            اگر os.path.exists(file):
                print(f"✓ Found: {os.path.basename(file)}")
            else:
                print(f"✗ Missing: {file}")
    else:
    
    print()

def check_ros2_installation():
    """Check اگر ROS 2 ہے properly installed"""
    
    try:
        # Check اگر ROS 2 ہے sourced
        import rclpy
        print("✓ ROS 2 Python library found")
        
        # Check ROS 2 version
        result = subprocess.run(['ros2', 'cli', '--version'], capture_output=True, text=True)
        اگر result.returncode == 0:
            print(f"✓ ROS 2 CLI: {result.stdout.strip()}")
        else:
            
    except ImportError:
    
    print()

def run_validation_tests():
    """Run validation tests"""
    print("=== Running Validation Tests ===")
    
    # Test 1: آئزک سیم Python API
    try:
        # Add آئزک سیم paths
        isaac_sim_path = os.environ.get('ISAAC_SIM_PATH')
        اگر isaac_sim_path:
            sys.path.insert(0, os.path.join(isaac_sim_path, 'python'))
        
        import omni
        
        # Test basic آئزک سیم functionality
        سے omni.isaac.core import World
        
    except ImportError کے طور پر e:
    except Exception کے طور پر e:
    
    # Test 2: ROS 2 Connection
    try:
        rclpy.init()
        نود = rclpy.create_node('validation_node')
        
        # Test پبلشر creation
        pub = نود.create_publisher(String, 'validation_test', 10)
        
        # Test سبسکرائیبر creation
        sub = نود.create_subscription(String, 'validation_test', lambda msg: None, 10)
        
        نود.destroy_node()
        rclpy.shutdown()
        print("✓ ROS 2 basic functionality working")
        
    except Exception کے طور پر e:
        print(f"✗ ROS 2 connection test failed: {e}")
    
    print()

def main():
    """Main validation function"""
    print("=" * 50)
    
    check_system_requirements()
    check_isaac_sim_installation()
    check_ros2_installation()
    run_validation_tests()
    
    print("Validation complete!")

اگر __name__ == '__main__':
    main()
```
## چ لالالالسا ہے آئزک saum کے SAAT ھ ROS 2

### کے l یے l یے ڈvaur ٹیn صی b
```bash
# Start آئزک سیم container کے ساتھ ROS 2 support
xhost +local:docker

docker run --gpus تمام -یہ --rm \
  --name isaac_sim_ros2 \
  --net=host \
  -e "ACCEPT_EULA=Y" \
  -e "NVIDIA_VISIBLE_DEVICES=تمام" \
  -e "NVIDIA_DRIVER_CAPABILITIES=تمام" \
  -e "PRIVACY_CONSENT=Y" \
  -v ~/isaac_sim_exts:/home/isaac-sim/.nvidia-omniverse/extensions \
  -v ~/.nvidia-omniverse/logs:/home/isaac-sim/.nvidia-omniverse/logs \
  -v ~/isaac_assets:/home/isaac-sim/isaac_assets \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --shm-size=2g \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  nvcr.io/nvidia/isaac-sim:4.0.0
```
### کے l یے l یے le یے آ baa ئی tt n صی b

1
2.
3. ما خذ آپ ک ک ک ایک روس 2 ماوسول ماؤس ia یک irmnil
4.

## خ LAA صہ

یہ الل l ی ں

1
2
3.
4
5
6

sasasastab ی کے ssastai آئزک ssasm کے Sast ی ی ی ی ض کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے کے J J کے Laulaulaulaus ص sis sas sas گ گ کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی نیچے کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی
