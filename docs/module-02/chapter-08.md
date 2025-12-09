# Chapter 8: High-Fidelity Rendering with Unity (Optional)

## Overview

In this final chapter of Module 2, you'll explore high-fidelity rendering using Unity, a powerful game engine that's increasingly used in robotics simulation. Unity provides photorealistic rendering capabilities that can generate synthetic training data for computer vision and perception systems. You'll learn how to connect Unity to ROS 2, import your robot models, and create visually stunning environments that can be used for perception training and realistic visualization.

While Gazebo is excellent for physics simulation, Unity excels at visual realism. By connecting Unity to ROS 2, you can have the best of both worlds: accurate physics simulation in Gazebo and photorealistic rendering in Unity. This approach is particularly valuable for training perception algorithms that need to operate in real-world conditions. By the end of this chapter, you'll have created a photorealistic home environment with your humanoid robot.

### Learning Objectives

By the end of this chapter, you will be able to:
- Set up Unity with the Robotics Hub package
- Connect Unity to ROS 2 using the TCP connector
- Import and configure URDF models in Unity
- Create photorealistic environments for perception training
- Generate synthetic sensor data with realistic rendering

### Prerequisites

Before starting this chapter, you should have:
- Completed all previous chapters in Module 2
- Understanding of ROS 2 communication patterns
- Basic familiarity with 3D modeling concepts
- Access to Unity Hub and Unity 2021.3 LTS or newer

## Concepts

### Unity Robotics Ecosystem

Unity provides several tools for robotics development:

- **Unity Robotics Hub**: Centralized package management for robotics tools
- **ROS-TCP-Connector**: Bridge between Unity and ROS 2 over TCP
- **Unity Perception Package**: Tools for generating synthetic training data
- **URDF Importer**: Converts URDF models to Unity format

### ROS-TCP Communication

The ROS-TCP connector enables communication between Unity and ROS 2:

- **TCP Socket**: Establishes persistent connection between Unity and ROS
- **Message Serialization**: Converts ROS messages to/from JSON format
- **Topic Mapping**: Maps Unity game objects to ROS topics
- **Service Calls**: Enables request-response communication

### Synthetic Data Generation

Unity can generate synthetic training data with:

- **Domain Randomization**: Varying visual properties for robust training
- **Sensor Simulation**: Camera, LiDAR, and other sensor models
- **Ground Truth Annotation**: Automatic labeling of objects and properties
- **Physics Simulation**: Realistic interactions between objects

### Photorealistic Rendering

Unity's rendering pipeline includes:

- **Light Transport**: Realistic light behavior and shadows
- **Material Systems**: Physically-based rendering (PBR) materials
- **Post-Processing**: Visual effects and atmospheric effects
- **Lighting Models**: Real-world lighting conditions

### Perception Training Pipeline

The complete pipeline for perception training:

- **Environment Generation**: Creating diverse scenarios
- **Sensor Simulation**: Generating realistic sensor data
- **Ground Truth Generation**: Automatic annotation of scenes
- **Data Export**: Formatting data for ML training

## Examples

### Example 1: Setting up Unity with ROS-TCP Connector

First, create a Unity project with the ROS-TCP connector:

1. Install Unity Hub and create a new 3D project
2. Open the Package Manager (Window > Package Manager)
3. Add the ROS-TCP-Connector package from the Unity Asset Store or via Git URL
4. Add the ROS-TCP-Connector to your scene

Here's a basic Unity C# script to connect to ROS:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnityRobotController : MonoBehaviour
{
    ROSConnection ros;
    string rosIPAddress = "127.0.0.1"; // Default to local host
    int rosPort = 10000; // Default port for ROS-TCP-Connector

    // Robot joint transforms
    public Transform headJoint;
    public Transform leftShoulderJoint;
    public Transform rightShoulderJoint;

    // Start is called when the game starts
    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;

        // Set the IP address and port
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to joint state messages
        ros.Subscribe<sensor_msgs.JointStateMsg>("joint_states", JointStateCallback);

        Debug.Log("Unity Robot Controller initialized");
    }

    // Callback function when joint state messages are received
    void JointStateCallback(sensor_msgs.JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = jointState.position[i];

            // Update joint transforms based on received positions
            switch (jointName)
            {
                case "neck_joint":
                    if (headJoint != null)
                        headJoint.localRotation = Quaternion.Euler(0, 0, jointPosition * Mathf.Rad2Deg);
                    break;
                case "left_shoulder_joint":
                    if (leftShoulderJoint != null)
                        leftShoulderJoint.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
                    break;
                case "right_shoulder_joint":
                    if (rightShoulderJoint != null)
                        rightShoulderJoint.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
                    break;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Publish transform data as TF messages
        PublishTransforms();
    }

    void PublishTransforms()
    {
        // Create transform message
        geometry_msgs.TransformStampedMsg tfMsg = new geometry_msgs.TransformStampedMsg();

        // Set header
        tfMsg.header.frame_id = "world";
        tfMsg.header.stamp = new builtin_interfaces.TimeMsg();
        tfMsg.child_frame_id = "unity_robot";

        // Set transform
        tfMsg.transform.translation = new geometry_msgs.Vector3Msg(
            transform.position.x,
            transform.position.y,
            transform.position.z
        );
        tfMsg.transform.rotation = new geometry_msgs.QuaternionMsg(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        );

        // Publish TF message
        ros.Publish("tf", tfMsg);
    }
}
```

### Example 2: URDF Import Setup

To import your URDF robot into Unity:

1. Install the URDF Importer package from Unity Robotics Hub
2. Create a URDF description of your robot (or use the one from previous chapters)
3. Import the URDF into Unity via Assets > Import Robot from URDF

Here's an example of preparing your URDF for Unity import:

```xml
<?xml version="1.0" ?>
<robot name="unity_humanoid">
  <!-- Materials for Unity visualization -->
  <material name="blue">
    <color rgba="0.2 0.2 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.2 0.2 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Example 3: Creating a Photorealistic Environment

Create a Unity C# script to generate a photorealistic home environment:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;

public class EnvironmentGenerator : MonoBehaviour
{
    public GameObject[] furniturePrefabs; // Tables, chairs, etc.
    public Material[] surfaceMaterials; // Different floor/counter materials
    public Light[] lightSources; // Directional and point lights

    [Header("Environment Settings")]
    public float roomWidth = 10f;
    public float roomDepth = 8f;
    public float roomHeight = 3f;

    [Header("Furniture Settings")]
    public int minFurniture = 5;
    public int maxFurniture = 10;
    public float minScale = 0.8f;
    public float maxScale = 1.2f;

    void Start()
    {
        GenerateEnvironment();
    }

    void GenerateEnvironment()
    {
        // Create room boundaries
        CreateRoomBoundaries();

        // Add random furniture
        AddRandomFurniture();

        // Configure lighting
        ConfigureLighting();

        // Apply materials
        ApplyMaterials();
    }

    void CreateRoomBoundaries()
    {
        // Create floor
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Floor";
        floor.transform.position = new Vector3(0, -roomHeight/2, 0);
        floor.transform.localScale = new Vector3(roomWidth, 0.1f, roomDepth);

        // Create walls
        CreateWall(new Vector3(0, 0, roomDepth/2), new Vector3(roomWidth, roomHeight, 0.1f)); // Back wall
        CreateWall(new Vector3(0, 0, -roomDepth/2), new Vector3(roomWidth, roomHeight, 0.1f)); // Front wall
        CreateWall(new Vector3(roomWidth/2, 0, 0), new Vector3(0.1f, roomHeight, roomDepth)); // Right wall
        CreateWall(new Vector3(-roomWidth/2, 0, 0), new Vector3(0.1f, roomHeight, roomDepth)); // Left wall
    }

    GameObject CreateWall(Vector3 position, Vector3 scale)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "Wall";
        wall.transform.position = position;
        wall.transform.localScale = scale;
        return wall;
    }

    void AddRandomFurniture()
    {
        int furnitureCount = Random.Range(minFurniture, maxFurniture + 1);

        for (int i = 0; i < furnitureCount; i++)
        {
            if (furniturePrefabs.Length > 0)
            {
                GameObject prefab = furniturePrefabs[Random.Range(0, furniturePrefabs.Length)];
                GameObject furniture = Instantiate(prefab);

                // Random position within room bounds
                float x = Random.Range(-roomWidth/2 + 1f, roomWidth/2 - 1f);
                float z = Random.Range(-roomDepth/2 + 1f, roomDepth/2 - 1f);
                furniture.transform.position = new Vector3(x, 0, z);

                // Random rotation
                furniture.transform.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);

                // Random scale
                float scale = Random.Range(minScale, maxScale);
                furniture.transform.localScale *= scale;
            }
        }
    }

    void ConfigureLighting()
    {
        // Add a main directional light (sun)
        GameObject sunLight = new GameObject("SunLight");
        sunLight.AddComponent<Light>();
        Light sun = sunLight.GetComponent<Light>();
        sun.type = LightType.Directional;
        sun.color = Color.white;
        sun.intensity = 1.0f;
        sun.transform.rotation = Quaternion.Euler(50, -30, 0);

        // Add ambient lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.2f, 0.2f, 0.4f);
        RenderSettings.ambientEquatorColor = new Color(0.2f, 0.4f, 0.2f);
        RenderSettings.ambientGroundColor = new Color(0.2f, 0.2f, 0.2f);
    }

    void ApplyMaterials()
    {
        // Apply random surface materials to floor
        GameObject floor = GameObject.Find("Floor");
        if (floor != null && surfaceMaterials.Length > 0)
        {
            Material floorMat = surfaceMaterials[Random.Range(0, surfaceMaterials.Length)];
            floor.GetComponent<Renderer>().material = floorMat;
        }
    }
}
```

### Example 4: Sensor Simulation in Unity

Create a Unity script to simulate camera sensors and publish images to ROS:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Threading.Tasks;
using System.IO;

public class UnityCameraSensor : MonoBehaviour
{
    ROSConnection ros;
    public Camera sensorCamera; // The camera to use for sensor simulation
    public string topicName = "/unity_camera/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public int publishRate = 30; // Hz

    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private byte[] imageBytes;

    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.instance;

        // Set up camera if not assigned
        if (sensorCamera == null)
            sensorCamera = GetComponent<Camera>();

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        sensorCamera.targetTexture = renderTexture;

        // Create texture for reading pixels
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Calculate publish interval
        publishInterval = 1.0f / publishRate;
        lastPublishTime = 0;
    }

    void Update()
    {
        // Publish image at specified rate
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishCameraImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishCameraImage()
    {
        // Set active render texture
        RenderTexture.active = renderTexture;

        // Read pixels from render texture
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Encode texture to PNG bytes
        imageBytes = texture2D.EncodeToPNG();

        // Create ROS image message
        sensor_msgs.ImageMsg imageMsg = new sensor_msgs.ImageMsg();
        imageMsg.header = new std_msgs.HeaderMsg();
        imageMsg.header.stamp = new builtin_interfaces.TimeMsg(0, (uint)(Time.time * 1e9));
        imageMsg.header.frame_id = "unity_camera_optical_frame";

        imageMsg.height = (uint)imageHeight;
        imageMsg.width = (uint)imageWidth;
        imageMsg.encoding = "rgb8";
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(imageWidth * 3); // 3 bytes per pixel (RGB)

        // Convert bytes to ROS byte array
        imageMsg.data = new byte[imageBytes.Length];
        for (int i = 0; i < imageBytes.Length; i++)
        {
            imageMsg.data[i] = imageBytes[i];
        }

        // Publish image message
        ros.Publish(topicName, imageMsg);
    }
}
```

## Exercise

### Required Exercise: Unity Perception Training Environment

Build a complete Unity-based perception training environment:

**Task**: Create an environment with:
1. A Unity scene with photorealistic home environment
2. Your humanoid robot imported from URDF
3. A camera sensor that publishes images to ROS
4. A system for generating diverse training scenarios

**Acceptance Criteria**:
- Unity scene renders with realistic lighting and materials
- Robot responds to ROS joint commands in Unity
- Camera sensor publishes realistic images to ROS
- Environment can be randomized for training data generation

**Implementation Steps**:
1. Set up Unity project with Robotics Hub packages
2. Import your URDF robot model into Unity
3. Create a photorealistic home environment
4. Implement camera sensor simulation
5. Add domain randomization capabilities
6. Test ROS communication between Unity and external nodes

**Extension Challenges**:
- **Beginner**: Add more furniture and objects to the environment
- **Intermediate**: Implement LiDAR sensor simulation in Unity
- **Advanced**: Create a complete synthetic dataset generation pipeline with annotations

## References

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Perception Package](https://github.com/Unity-Technologies/Unity-Perception)
- [URDF Importer for Unity](https://github.com/Unity-Technologies/URDF-Importer)
- [Synthetic Data for Computer Vision](https://blogs.unity3d.com/2021/05/10/unity-releases-ai-toolkit-for-generating-synthetic-training-data/)
- [Unity Robotics Best Practices](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/best_practices.md)
