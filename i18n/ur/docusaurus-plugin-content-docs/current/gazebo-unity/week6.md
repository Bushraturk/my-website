---
title: "ہفتہ 6: یونٹی انضمام اور اعلیٰ سینسر"
sidebar_position: 7
---

# ہفتہ 6: یونٹی انضمام اور اعلیٰ سینسر

یہ آخری ہفتہ Gazebo/Unity ماڈیول کے ہے، جہاں ہم Unity کی دنیا میں درآمد کریں گے اور اعلی معیار کے سینسر ماڈلز کو سمجھیں گے۔

## سیکھنے کے اہداف

اس ہفتے کے اختتام تک، آپ درج ذیل کر سکیں گے:

- یونٹی ماحول میں روبوٹکس سیمولیشن تیار کرنا
- فوٹو ریلزم اور اعلی معیار کے ویژنل سینسرز کو نافذ کرنا
- ROS 2 کے ساتھ یونٹی کو انضمام کرنا
- Synthetic data generation for perception training
- Unity اور Gazebo کے درمیان انتخاب کے معیار کو سمجھنا

## Unity کا تعارف برائے روبوٹکس

Unity ایک 3D گیم انجن ہے جسے ایک مکمل ڈیجی ٹل ماحول تیار کرنے کے لیے استعمال کیا جا سکتا ہے جہاں روبوٹکس کی ترقی اور ٹیسٹنگ کی جا سکتی ہے۔ یہ اعلی معیار کے ویژنل اور رینڈرنگ کے فوائد فراہم کرتا ہے جو:

- **Photorealistic rendering**: اعلی معیار کے ویژل ایفیکٹس اور لائٹنگ
- **Asset store**: بڑی تعداد میں ماڈلز، ٹیکسچرز، اور اوزار
- **Scripting environment**: C# کے ذریعے کسٹم برتاؤ اور منطق لکھنا
- **XR support**: VR، AR، اور MR تجربات کے لیے تیزاب

### Unity کی کارکردگی

Unity کی فیچر لسٹ مندرجہ ذیل ہے:

- **High-definition rendering**: HDRP اور URP کے ذریعے اعلی معیار کا رینڈرنگ
- **Physics engine**: Built-in physics engine for realistic simulation
- **Visual scripting**: بیزک گرافکل انٹرفیس کے ذریعے منطق تیار کرنا
- **Large community**: وسیع ڈیویلپر کمیونٹی اور وسائل
- **Cross-platform**: Windows، Linux، Mac، اور دیگر پلیٹ فارمز کے لیے ڈیپلائیبل

## Unity میں روبوٹکس سیمولیشن

### Unity Robotics Hub

Unity Robotics Hub ایک مجموعہ ہے جو روبوٹکس ٹولز اور کمپوننٹس فراہم کرتا ہے:

- **ROS#**: C# کے ذریعے ROS 2 کنیکٹیویٹی
- **Unity Perception**: Synthetic data generation for AI training
- **ML-Agents**: Reinforcement learning framework
- **Unity Template Projects**: Robotics-specific project templates

### انسٹال کرنا

1. یونٹی ہب انسٹال کریں (2021.3 LTS یا جدید ورژن)
2. یونٹی روبوٹکس ہب ایسٹ سٹور سے انسٹال کریں
3. ROS# پیکیج کو ROS کنکشن کے لیے شامل کریں
4. Perception پیکیج synthetic data generation کے لیے شامل کریں

## Unity Perception Package

Unity Perception package synthetic data کو جنریٹ کرنے کے لیے اوزار فراہم کرتا ہے:

### Semantic Segmentation

- **Perception Camera**: کیمرہ کمپوننٹ کے ساتھ، سیمنٹک سیگمینٹیشن امیج جنریٹ کرنا
- **Annotation Manager**: کمپوننٹس کو کیسے لیبل کیا جائے اس کا انتظام

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;

public class PerceptionExample : MonoBehaviour
{
    void Start()
    {
        // Perception camera انسٹال کریں
        var perceptionCamera = GetComponent<PerceptionCamera>();
        perceptionCamera.InstancesSegmentationEnabled = true;
    }
}
```

### Bounding Box Generation

- **Bounding Box Labeler**: 2D اور 3D bounding box اینوٹیشن
- **Occlusion Handling**: اشیاء کو صحیح طریقے سے ڈرائی کرنا

### Synthetic Data Generation

Perception package کو synthetic data کے لیے استعمال کرنا:

```json
{
  "cameras": [
    {
      "name": "MainCamera",
      "captureRgb": true,
      "captureDepth": true,
      "captureSegmentation": true,
      "captureOpticalFlow": true,
      "captureBoundingBox2D": true,
      "captureBoundingBox3D": true,
      "recording": {
        "outputDir": "/path/to/output",
        "framerate": 30,
        "frameskip": 0
      }
    }
  ]
}
```

## Unity-ROS 2 Bridge

Unity-ROS 2 انضمام کے لیے، ROS# پیکیج استعمال کریں:

### Basic Connection

```csharp
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityROSConnection : MonoBehaviour
{
    ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("127.0.0.1", 10000); // ROS IP اور پورٹ
    }
    
    public void SendMessage()
    {
        var message = new StringMsg("Hello from Unity!");
        ros.Publish("unity_topic", message);
    }
    
    void OnMessageReceived(StringMsg msg)
    {
        Debug.Log("Received: " + msg.data);
    }
}
```

### Robot Control via Unity

Unity میں روبوٹ کنٹرول:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotTopic = "robot_cmd";
    
    void Start()
    {
        ros = ROSConnection.instance;
        ros.RegisterPublisher<TwistMsg>(robotTopic);
    }
    
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            var cmd = new TwistMsg();
            cmd.linear = new Vector3Msg(1, 0, 0); // آگے حرکت
            cmd.angular = new Vector3Msg(0, 0, 0.5); // گھمائیں
            ros.Publish(robotTopic, cmd);
        }
    }
}
```

## فوٹو ریلزم رینڈرنگ

### High-Quality Graphics

Unity کے فوٹو ریلزم فیچرز:

- **Ray Tracing**: Real-time ray tracing for realistic lighting
- **Global Illumination**: Light bouncing and indirect illumination
- **HDR Support**: High Dynamic Range imaging
- **PBR Materials**: Physically-based rendering materials

### Light Simulation

Unity کی لائٹنگ کو Gazebo کے فزکل سے مماثل بنانا:

- **Directional Lights**: Sun-like lighting
- **Point/Spot Lights**: Artificial lighting sources
- **Area Lights**: Soft shadows and realistic lighting
- **Reflection Probes**: Environmental reflections

## Gazebo vs Unity Selection Criteria

### Gazebo کے لیے

Gazebo مندرجہ ذیل کے لیے بہتر ہے:

- **Physics Simulation**: Highly accurate physics and collisions
- **Robotics Simulation**: Specifically designed for robotics applications
- **Sensor Simulation**: Realistic sensor models (LiDAR, cameras, IMUs)
- **ROS Integration**: Tight integration with ROS/ROS 2 ecosystems
- **Performance**: Optimized for real-time robotics simulation

### Unity کے لیے

Unity مندرجہ ذیل کے لیے بہتر ہے:

- **Visual Quality**: Photorealistic rendering and effects
- **Synthetic Data**: High-quality data for AI training
- **Perception Tasks**: Computer vision model development
- **User Experience**: VR/AR/XR applications
- **Asset Variety**: Extensive library of 3D models and environments

## Unity Perception Pipeline

Unity Perception کا استعمال synthetic vision data کو تیار کرنے کے لیے:

### Data Annotation

Unity Perception کے ساتھ automatic annotation:

- **Semantic Segmentation**: Pixel-level object classification
- **Instance Segmentation**: Individual object instance labeling
- **Depth Maps**: Metric depth data for each pixel
- **Optical Flow**: Motion vectors between frames
- **Bounding Boxes**: 2D and 3D object localization

### Training Data Generation

Pipeline کو synthetic data کے لیے ترتیب دینا:

1. **Scene Creation**: Environment and objects setup
2. **Camera Placement**: Virtual cameras positioning
3. **Parameter Randomization**: Materials, lighting, textures variation
4. **Data Capture**: RGB, depth, segmentation, etc.
5. **Annotation Generation**: Labels and ground truth creation
6. **Dataset Export**: Format conversion for ML frameworks

## کارکردگی کا موازنہ

### کمپیوٹیشنل ضروریات

| اوزار | CPU Usage | GPU Usage | Memory Usage |
|--------|-----------|------------|--------------|
| Gazebo | معمولی | معمولی-زیادہ | معمولی |
| Unity | معمولی | زیادہ | زیادہ |

### سیمولیشن ایکوائی

| پہلو | Gazebo | Unity |
|------|--------|--------|
| فزکس | Highly accurate | Good (with PhysX) |
| Rendering | Functional but basic | Highly realistic |
| Sensors | Accurate physical models | Visual quality-focused |
| Performance | Optimized for robotics | Optimized for visuals |

## مشترکہ استعمال کے معاملات

### Gazebo + Unity Hybrid

کچھ ایپلی کیشنز میں دونوں کو استعمال کرنا مفید ہے:

1. **Pre-training in Unity**: Visual perception model training in Unity
2. **Validation in Gazebo**: Physics accuracy validation
3. **Transfer to Reality**: Combined sim-to-real approaches

### Data Pipeline

```python
# Hybrid approach example
def hybrid_simulation_pipeline():
    # 1. Generate perception data in Unity
    unity_data = generate_perception_data_in_unity()
    
    # 2. Validate control in Gazebo
    gazebo_validation = validate_controls_in_gazebo(unity_data['robot_commands'])
    
    # 3. Combine results
    final_dataset = combine_datasets(unity_data, gazebo_validation)
    
    return final_dataset
```

## خلاصہ

Gazebo اور Unity دونوں ہی روبوٹکس کی ترقی کے لیے اہم اوزار ہیں، ہر ایک اپنے مقصد کے لیے موزوں ہے۔ Gazebo فزکل اور سینسر ماڈلنگ پر مرکوز ہے، جبکہ Unity ویژنل معیارات اور ویژنل ڈیٹا جنریشن پر مرکوز ہے۔

اسی طرح، اعلی معیار کے سینسرز کو سمجھنے سے آپ کو یہ معلوم ہو گا کہ کیا ہے ممکنہ استعمال کے معاملات کے لیے، آپ کو کونسا سیمولیٹر استعمال کرنا چاہئے۔

یہ ماڈیول آپ کو یہ سیکھنے کا موقع فراہم کرے گا کہ کیسے دونوں کو ایک مکمل روبوٹکس ترقی کے ماحول کے حصے کے طور پر استعمال کیا جائے، جہاں Gazebo فزکل سیمولیشن کے لیے استعمال ہو، اور Unity تاثرات کے تربیت اور ویژنل ڈیٹا جنریشن کے لیے استعمال ہو۔

[← پچھلا: ہفتہ 4-5: Gazebo کی بنیاد اور ماحول کی تیاری](./week4-5.md) | [اگلا: ماڈیول کا خاتمہ](./conclusion.md) | [MA 2 صفحہ](./intro.md)

ماڈیول کے خاتمے پر جائیں جہاں آپ [MA 2 کا خاتمہ](./conclusion.md) کریں گے اور اس کے اہم نکات کا جائزہ لیں گے۔