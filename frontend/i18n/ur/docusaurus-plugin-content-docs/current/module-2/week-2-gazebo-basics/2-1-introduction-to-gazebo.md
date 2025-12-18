---


sidebar_position: 1
difficulty: intermediate


---

# ست 2: bo بائو بنیادی باتیں

## جAج

یہ ہف ہف ہف نے کa/کی bunیadیat کa گیزabo ، کا تعارف کرایا ہے ، یک طاقتور روبوسس سمیلیٹر وِس آپ آپ ک ک ک ک کs ک ک ک ک ک کa روبوٹک سسٹمز مائی ورچوئل مائیوول کی توثیق کرتا ہے۔

## ssیکھnے کے mauaصd

کے ذ ذ کa/کی کی کی خatatam کa یہ یہ ہف ہف ہف آپ ک ک کrے گ گ گ ک
- کa/کی بنیادی تصورات کو سمجھیں
- سیٹ اوشر اوار تشکیل کریں
- اوور ہیرا پھیری سمولین ماحول بنائیں
- بنیادی روبو کے ماڈلز کو نافذ کریں

## کیa ہے گیز گیز گیز گیز

گیزبو ہے ایک 3D dynamic simulator کے ساتھ کا/کی ability کو accurately اور efficiently simulate populations کا robots میں complex indoor اور outdoor environments. یہ ک/کی ٹولز مہیا کرتا ہے اوشو ماؤل کو تخلیق ، ٹیسٹ ، اوور روبوٹک سسٹمز کو توثیق کرتے ہیں۔

### کلیدی خصوصیات

- ** طبیعیات سمولن **: حقیقت پسندانہ سمولہن - جسمانی حرکیات
۔
- ** پلگ انز **: وسیع پلگ ان سیسم کے LLیے کسٹم فعالیت
- ** ROS انضمام **: آبائی انضمام کے saatھ ros/ros 2

## گیزbw انسٹال کرنا
کے لیے detailed تنصیب instructions, refer کو کا/کی [official گیزبو تنصیب guide](http://gazebosim.org/tutorials?cat=install).
### تقاضے کے Lیے گیزBO کے saatھ ros 2

| AجزAaء | کم سے کم | تجویز کردہ |
| ----------- | --------- | ------------- |
| OS | اوبنٹو 22.04/20.04 | اوبنٹو 22.04 lts |
| GPU | اوپن جی ایل 2.1 ہم آہنگ | سرشار GPU کے saatھ cuda سپورٹ |
| رام | 8 جی بی | 16+ جی بی |
| اسٹوریج | 5 جی بی | 10+ جی بی |

## شروع ہو رہا ہے کے saatھ گیز bo

### ZBO کا آغاز کرنا
```bash
# Basic launch
گیزبو

# Launch کے ساتھ empty world
گیزبو --verbose worlds/empty.world

# Launch کے ساتھ ROS 2 integration
ros2 launch gazebo_ros گیزبو.launch.py
```
### بنیادی یبو ماماول

کb گیزbo شروع ہوتا ہے ، l 'دیکھیں گے:
- ia 3D بصری ونڈو جس میں ک/کی مصنوعی دنیا دکھائی جارہی ہے
- کنٹرول پینل کے سعتی سومولین ٹولز
- ماڈل ڈیٹا بیس کے Lیے indings indements کs کa/کی smwlin
- روشنی کے ذرائع اوار ماؤل کی ترتیبات

## گیز بیو ورلڈز اورس ماڈل

### دنیا کی تشکیل

bowbo ورلڈز SD SDF (smwlthn تفصیل کی شکل) فائلوں کا استعمال کرتے ہوئے بیان کردہ:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```
### ماڈل شامل کرنا

گیز بی او میں یک ڈیٹا بیس شامل ہے۔ آپ ک ک ک ک/کی GUI کے ذریعے پروگراموں میں ماڈل شامل کریں۔
```bash
# List available models
gz model --list
```
## ROS 2 انضمام

### گیزBW پلگ انز کے LLیے ROS 2

گیز بوب پلگ ان فراہم کرتا ہے۔

1. ** libgazebo_ros_diff_drive.so **: مختلف ڈرائیو کنٹرولر
2
3. ** libgazebo_ros_camera.so **: کیمرا سینسر انٹرفیس

### مامال انضمام
```xml
<!-- میں آپ کا روبوٹ's یوآر ڈی ایف/XACRO -->
<گیزبو>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/my_robot</namespace>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.15</wheel_diameter>
  </plugin>
</گیزبو>
```
## املی وورک اِس

یہ ہفtہ's vrک sas کs ک گa گائیڈ آپ vauchr سادہ تفریق ڈرائیو rrooboc maus گیز bo گیز bow owr ros 2 کمانڈز کے ذریعہ کنٹرولنگ یہ.

1. لانچ کریں گیزبو کے سعتی روس 2 انضمام
2. سپون اِس ڈفوریل ڈرائیو روبو ماڈل
3. ROS 2 ٹAپکS کا استعمال کرتے ہوئے کa/کی rewboc پر قابو پالیں
4. سادہ نیویگیشن ٹاسک کو نافذ کریں

## خlaaصہ

یہ ہف ہف ہف ہف ہف ہف گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز گیز Bo آپ 'سیکھا کیsے کsے کs set oauaur گیز bo گیز saatھ ros ros 2 انضمام owr کیsic کs ک ک ک ک ک کی کی کی کی کی کی کی کی کی کی remation. اگلا ، ہم اعلی درجے کی سمولہن تصورات اور انضمام کی تکنیک کو دیکھیں گے۔
