---


sidebar_position: 2
difficulty: intermediate


---
# ساس 3: اوسوؤنس ڈ آئزک آئزک آئزک آئزک آئزک آئزک آئزک

## ج a ج

n nnwaua کے جامعروبوبوسس سوسمولہن ماواسال ک a ک astamal ک satau ہ ، ج ج ج ج ج ک ک ک ک ک ک ک ج ج ج ج ج ج ج ک ک ک ک ک ک ک ک ک ج ج ج ج

## ss یکھ n ے کے maua ص d

کے ذ ک ک ک ک a/کی کی خ خ خ خ atatam ک a یہ یہ ہف ہف آپ آپ ک ک ک ک ک ک ک ک ے ے ے ے گ گ گ گ گ
- ماسٹر ایڈوانسڈ آئزک سیم منظر تخلیق اوار لائٹنگ
- پ snadain ہ ط biaua ی jowrs maad ی juaut ک o ک s
- مصنوعی ڈیٹاسیٹس کے لِل کی تربیت پیدا کریں
- سووولان کے لِل ایل الک ببا- اسسال روبو ٹی saiun گ ک s s s s s

## ج دد منشر تالیق

### USD منشر تیگرب

آئزک ssaum nے ع ع ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط ط vr پ ris ک ک a araiaumriusimriی saurmiusarimi فasataul ک ک ک ک کی کی کی ک ک ک ک کی iaجaaزat ہے:
```python
# Advanced USD scene creation میں آئزک سیم
import omni
سے omni.isaac.core import World
سے omni.isaac.core.utils.stage import add_reference_to_stage
سے omni.isaac.core.utils.prims import get_prim_at_path
import carb

class AdvancedSceneSetup:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        
    def setup_complex_scene(self):
        # Add complex factory ماحول
        add_reference_to_stage(
            usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/4.1/Isaac/Environments/"
                    "hospital.usd",
            prim_path="/World/hospital"
        )
        
        # Add dynamic objects
        self.add_dynamic_objects()
        
        # Configure advanced lighting
        self.setup_advanced_lighting()
    
    def add_dynamic_objects(self):
        # Add moving conveyor belt
        سے omni.isaac.core.objects import DynamicCuboid
        
        conveyor = DynamicCuboid(
            prim_path="/World/conveyor/belt",
            name="conveyor_belt",
            position=[0, 0, 0.1],
            size=1.0,
            mass=1.0
        )
        
        # Add objects کو conveyor
        کے لیے میں میں range(5):
            obj = DynamicCuboid(
                prim_path=f"/World/conveyor/object_{میں}",
                name=f"object_{میں}",
                position=[0, میں * 0.5, 0.5],
                size=0.2,
                mass=0.1
            )
```
### vuswwroulsi ٹک llaiun گ

آئزک پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی ص پی پی پی پی پی پی پی پی پی ص پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی پی ں ں
```python
# Advanced lighting ترتیب
def setup_advanced_lighting(self):
    # Add dome light کے لیے global illumination
    سے omni.isaac.core.utils.prims import create_prim
    
    create_prim(
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        position=[0, 0, 0],
        attributes={
            "inputs:color": (0.2, 0.2, 0.2),  # ماحول color
            "inputs:intensity": 3000.0        # Intensity میں lumens
        }
    )
    
    # Add IBL (Image Based Lighting)
    سے pxr import UsdLux
    
    dome_light = get_prim_at_path("/World/DomeLight")
    UsdLux.DomeLightAPI.Apply(dome_light)
    
    # Configure IBL texture
    carb.settings.get_settings().set("/rtx/rendering/cullMode", 0)
    
    # Add directional light (sun)
    create_prim(
        prim_path="/World/Sun",
        prim_type="DistantLight",
        position=[0, 0, 50],
        orientation=[-0.707, 0, -0.707, 0],  # Pointing downward
        attributes={
            "inputs:color": (0.9, 0.9, 0.9),
            "inputs:intensity": 500.0
        }
    )
```
## طbیsasahat سامولن

### ج ج ط ط ط ط کی کی کی کی خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص خص

اٹ سنڈان ہ ط ط ط ط ہ ہ ہ ہ ہ کے پی کے پی کے کے کے کے کے کے کے کے کے کے کے کے کے کے شکی شکی شکی شکی شکی شکی شکی شکی شکی
```python
# Advanced physics تشکیل
def setup_advanced_physics(self):
    # Get physics scene
    scene = self.world.scene
    scene.enable_physics()
    
    # Configure PhysX physics properties
    scene.set_physics_dt(1.0/60.0, substeps=1)  # Physics timestep
    
    # Set gravity
    scene.enable_gravities = True
    سے omni.isaac.core.utils.physics import set_gravity
    set_gravity([0, 0, -9.81])  # Standard gravity
    
def setup_material_properties(self, prim_path):
    # Create advanced material properties
    سے omni.isaac.core.materials import PhysicsMaterial
    سے omni.kit.primitive.mesh.materials import Material
    
    # Create physics material
    physics_material = PhysicsMaterial(
        prim_path=f"{prim_path}/physics_material",
        static_friction=0.5,
        dynamic_friction=0.4,
        restitution=0.2  # Bounciness
    )
    
    # Create visual material
    visual_material = Material(
        prim_path=f"{prim_path}/visual_material",
        diffuse_color=(0.8, 0.1, 0.1),  # Red material
        metallic=0.1,
        roughness=0.2,
        clearcoat=0.0,
        clearcoat_roughness=0.0,
        opacity=1.0
    )
    
    # Apply materials کو prim
    import omni.kit.commands
    omni.kit.commands.execute(
        "BindMaterialToPrim",
        prim_path=prim_path,
        material_path=visual_material.prim_path
    )
```
### اعنیی ی ایم آر آئی ضی کے mauab ق ط baaaa at sswaumuln پر

ک saum فزک s فزک کے کے ط ط ط ط ط ط ط ط
```python
# Custom physics behavior مثال
سے omni.isaac.core import Actor
سے omni.isaac.core.utils.prims import get_prim_at_path
import numpy کے طور پر np

class CustomPhysicsBehavior:
    def __init__(self, robot_prim_path):
        self.robot_prim = get_prim_at_path(robot_prim_path)
        
    def apply_custom_force(self, force_vector, position):
        # Apply custom forces کے لیے special physics behavior
        سے omni.isaac.core.utils.physics import apply_force_at_pos
        
        apply_force_at_pos(
            prim_path=self.robot_prim.GetPrimPath().pathString,
            force=force_vector,
            position=position
        )
    
    def simulate_deformable_objects(self):
        # مثال کا simulating deformable objects using NVIDIA FleX
        سے omni.physx import get_physx_interface
        
        physx = get_physx_interface()
        # Configure FleX سمولیشن parameters
        physx.set_parameter("flex", True)
        physx.set_parameter("flex_relaxation", 0.8)
        physx.set_parameter("flex_solid_rest", 0.5)
```
## maunoa ی ڈیٹ a ج nri یش n

### ڈیٹ آسا یٹ ک lain پ aa ئپ llaiun

maunoaua ی ڈیٹ asa یٹ s Buna ئیں کے کے ِ ِ ہی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی کی
```python
# Synthetic dataset generation
سے omni.isaac.core.utils.viewports import set_camera_view
سے omni.isaac.synthetic_utils import SyntheticDataHelper
import cv2
import numpy کے طور پر np

class SyntheticDatasetGenerator:
    def __init__(self, world, output_dir):
        self.world = world
        self.output_dir = output_dir
        self.sd_helper = SyntheticDataHelper()
        
        # Configure sensors کے لیے data collection
        self.setup_sensors()
    
    def setup_sensors(self):
        # Add RGB camera
        سے omni.isaac.sensor import Camera
        
        self.camera = Camera(
            prim_path="/World/RGB_Camera",
            position=[0.5, 0.5, 1.0],
            frequency=30  # Hz
        )
        
        # Add depth camera
        self.depth_camera = Camera(
            prim_path="/World/Depth_Camera",
            position=[0.5, 0.5, 1.0],
            frequency=30
        )
        
        # Enable depth information
        self.depth_camera.add_raw_separation_result_to_frame()
    
    def capture_synthetic_data(self, num_samples=1000):
        import os
        import json
        
        # Create output directories
        rgb_dir = os.path.join(self.output_dir, "rgb")
        depth_dir = os.path.join(self.output_dir, "depth")
        seg_dir = os.path.join(self.output_dir, "segmentation")
        
        کے لیے dir_path میں [rgb_dir, depth_dir, seg_dir]:
            os.makedirs(dir_path, exist_ok=True)
        
        کے لیے میں میں range(num_samples):
            # Move objects کو نیا positions
            self.randomize_scene()
            
            # Step کا/کی physics سمولیشن
            self.world.step(render=True)
            
            # Capture data
            rgb_img = self.camera.get_rgb()
            depth_img = self.depth_camera.get_depth()
            seg_img = self.get_segmentation_data()
            
            # Save data
            cv2.imwrite)
            cv2.imwrite
            cv2.imwrite
            
            # Save metadata
            metadata = {
                "sample_id": میں,
                "timestamp": self.world.current_time_step_index,
                "camera_pose": self.get_camera_pose(),
                "object_poses": self.get_object_poses()
            }
            
            کے ساتھ کھلا کے طور پر f:
                json.dump(metadata, f)
    
    def get_segmentation_data(self):
        # Get semantic segmentation data
        return self.sd_helper.get_semantic_segmentation()
    
    def randomize_scene(self):
        # Randomize object positions, lighting, اور textures
        pass
```
### vumaun b ے atrt ی b

ڈ Omaun re ی naiumaiauchn کے Li ِ S Mausbo ط atrbaut کی atrbaut ju jna فذ inaiad:
```python
import random
import colorsys

class DomainRandomization:
    def __init__(self, world):
        self.world = world
        
    def randomize_lighting(self):
        # Randomize lighting conditions
        dome_light = get_prim_at_path("/World/DomeLight")
        
        # Randomize color temperature
        color_temp = random.uniform(2700, 6500)
        color = self.color_temperature_to_rgb(color_temp)
        
        dome_light.GetAttribute("inputs:color").Set(color)
        
        # Randomize intensity
        intensity = random.uniform(1000, 5000)
        dome_light.GetAttribute("inputs:intensity").Set(intensity)
    
    def color_temperature_to_rgb(self, color_temp):
        """
        Convert color temperature میں Kelvin کو RGB values.
        """
        temp = color_temp / 100
        اگر temp <= 66:
            red = 255
            green = temp
            green = 99.4708025861 * math.log(green) - 161.1195681661
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
            
        اگر temp >= 66:
            blue = 255
        elif temp <= 19:
            blue = 0
        else:
            blue = temp - 10
            blue = 138.5177312231 * math.log(blue) - 305.0447927307
            
        # Clamp values کو 0-255 range
        red = max(0, min(255, red))
        green = max(0, min(255, green))
        blue = max(0, min(255, blue))
        
        return (red/255, green/255, blue/255)
    
    def randomize_materials(self):
        # Randomize material properties کے لیے domain randomization
        material_prims = self.get_material_prims()
        
        کے لیے prim میں material_prims:
            # Randomize diffuse color
            hue = random.random()
            saturation = random.uniform(0.5, 1.0)
            value = random.uniform(0.5, 1.0)
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            
            prim.GetAttribute("inputs:diffuse_color").Set(rgb)
            
            # Randomize roughness
            roughness = random.uniform(0.1, 0.9)
            prim.GetAttribute("inputs:roughness").Set(roughness)
    
    def randomize_objects(self):
        # Randomize object positions, orientations, اور count
        pass
```
## ک اراورڈ گی کی کی ص ص ص ص ص کی

### ssmwl یش n a ص laa ح کی tain یک

آئزک سوم کے کے کے ِ ِ ِ ِ ِ ِ ِ کے کے کے کے کے کے کے کے کے کے کے کے کے کے SAAT کے SAAT ھ SAAT ھ BI ڑ A-ASACIL ATACRWAUN ک S
```python
def optimize_simulation_performance(self):
    # Set rendering quality settings
    carb.settings.get_settings().set("/app/profiling/captureOnExit", False)
    carb.settings.get_settings().set("/app/profiling/captureOnStart", False)
    
    # Disable unnecessary rendering features
    carb.settings.get_settings().set("/rtx/oglEmu/gammaControlEnabled", False)
    carb.settings.get_settings().set("/rtx/enableSuperSampling", False)
    carb.settings.get_settings().set("/rtx/sceneDb/enableSceneDb", False)
    
    # Physics optimization
    carb.settings.get_settings().set("/physics/ag/enable_gpu", True)
    carb.settings.get_settings().set("/physics/ag/buffer_size", 1024)
    
    # Set appropriate timestep
    self.world.set_physics_dt(1.0/30.0, substeps=1)  # Lower frequency کے لیے performance
    
def batch_simulation(self, num_scenarios=50):
    """
    Run batch simulations کے لیے testing multiple scenarios efficiently
    """
    results = []
    
    کے لیے میں میں range(num_scenarios):
        # Set اوپر scenario
        self.setup_scenario
        
        # Run سمولیشن
        scenario_result = self.run_scenario()
        results.append(scenario_result)
        
        # Reset کے لیے next scenario
        self.reset_scenario()
    
    return results
```
## عمالہ وورک اِس

ہف ہف t ہ 's vr ک sassis mausonsi ڈ آئزک si ی m maausol کی tacli خ jauli ش aml ہے:

1. OSSی پیچی پیچی پیچی ہ ہ کے کے ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ ھ کے ھ ھ ھ ھ ھ ھ ھ ھ ھ کے ھ ھ ھ ھ ھ ھ ھ ھ
2.
3. مامنو ی ڈیٹ ڈیٹ پی پی پی ک ک ک ک کے کے ذہ ذہ یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے یے
4

## خ LAA صہ

یقیnی طvr پnیnی طvr طr پnی یقیur طur پ گی گی ہے ہف آئزک ہف ہف آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک نیچے آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک آئزک پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی پیچی ھ ھ SAAT ھ SAAT ھ SAAT ھ SAAT ھ ف ف ف ٹک Jauchnajighni ڈ Jaus Snusuas ڈیٹ SASAS ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ ڈیٹ اگلا ، ہ ہ ہ M آئزک saam ia یپ l ی کیش na ز ک v stlaai ک r یں گے۔
