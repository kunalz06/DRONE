Gazebo Simulation Guide: Autonomous Arena Drone (Mars Edition)

This document provides the complete setup for simulating the 16m x 16m arena mission in Gazebo Harmonic with ROS 2 Jazzy, featuring a realistic Martian terrain based on your 3D model.

1. The Arena Environment (worlds/arena.sdf)

Create a file at src/arena_drone_sim/worlds/arena.sdf. This world replaces the flat ground with a heightmap representing the rugged Martian surface.


<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="arena_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.5 0.3 1</diffuse> <!-- Martian Sun Color -->
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Martian Terrain Model -->
    <model name="mars_terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://scripts/heightmap.png</uri>
              <size>16 16 2</size> <!-- 16x16m area with 2m max elevation -->
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://scripts/mars_texture.png</diffuse>
                <normal>file://scripts/mars_texture.png</normal>
                <size>1</size>
              </texture>
              <uri>file://scripts/heightmap.png</uri>
              <size>16 16 2</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Major Boulders based on image -->
    <model name="boulder_1">
      <static>true</static>
      <pose>2 3 0.5 0.4 0.1 0.5</pose>
      <link name="link">
        <collision name="c1"><geometry><mesh><uri>[https://fuel.gazebosim.org/1.0/openrobotics/models/rock/1/meshes/rock.dae](https://fuel.gazebosim.org/1.0/openrobotics/models/rock/1/meshes/rock.dae)</uri></mesh></geometry></collision>
        <visual name="v1"><geometry><mesh><uri>[https://fuel.gazebosim.org/1.0/openrobotics/models/rock/1/meshes/rock.dae](https://fuel.gazebosim.org/1.0/openrobotics/models/rock/1/meshes/rock.dae)</uri></mesh></geometry>
        <material><ambient>0.6 0.3 0.2 1</ambient><diffuse>0.6 0.3 0.2 1</diffuse></material></visual>
      </link>
    </model>

    <model name="boulder_2">
      <static>true</static>
      <pose>-4 -2 0.3 0 0 1.2</pose>
      <link name="link">
        <visual name="v2"><geometry><box><size>1.2 0.8 0.6</size></box></geometry>
        <material><ambient>0.5 0.25 0.15 1</ambient><diffuse>0.5 0.25 0.15 1</diffuse></material></visual>
      </link>
    </model>

    <!-- 16m x 16m Yellow Boundary Markers -->
    <model name="yellow_bounds">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose> <!-- Elevated to stay visible on rugged terrain -->
      <link name="link">
        <visual name="north"><pose>8 0 0 0 0 0</pose><geometry><box><size>0.2 16 0.05</size></box></geometry><material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
        <visual name="south"><pose>-8 0 0 0 0 0</pose><geometry><box><size>0.2 16 0.05</size></box></geometry><material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
        <visual name="east"><pose>0 8 0 0 0 0</pose><geometry><box><size>16 0.2 0.05</size></box></geometry><material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
        <visual name="west"><pose>0 -8 0 0 0 0</pose><geometry><box><size>16 0.2 0.05</size></box></geometry><material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Base Station Dock (Level Platform) -->
    <model name="base_station">
      <static>true</static>
      <pose>0 0 0.1 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>1.5 1.5 0.2</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>1.5 1.5 0.2</size></box></geometry>
          <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>


2. Terrain Generation Script (scripts/generate_mars_assets.py)

Since Gazebo requires physical image files for heightmaps, run this script to generate the heightmap.png and mars_texture.png inside your package's script folder.

import numpy as np
import cv2
import os

def create_mars_assets(output_path):
    # 1. Heightmap (Greyscale 8-bit or 16-bit)
    # 129x129 is a standard size for Gazebo heightmaps (2^n + 1)
    size = 129
    heightmap = np.zeros((size, size), dtype=np.uint8)
    
    # Generate Perlin-like noise for rugged terrain
    for i in range(size):
        for j in range(size):
            # Base undulation
            val = 128 + 20 * np.sin(i/10.0) * np.cos(j/10.0)
            # Add small noise (rocks/craters)
            val += np.random.normal(0, 5)
            heightmap[i, j] = np.clip(val, 0, 255)

    # Add a specific "Hill" as seen in the top corner of the image
    cv2.circle(heightmap, (30, 30), 20, 180, -1)
    heightmap = cv2.GaussianBlur(heightmap, (15, 15), 0)

    # Save Heightmap
    cv2.imwrite(os.path.join(output_path, 'heightmap.png'), heightmap)

    # 2. Texture (Red/Orange Martian Soil)
    texture = np.zeros((512, 512, 3), dtype=np.uint8)
    texture[:, :] = [19, 53, 133] # BGR for Reddish Orange (approx #853513)
    # Add noise to texture for realism
    noise = np.random.randint(0, 30, (512, 512, 3), dtype=np.uint8)
    texture = cv2.add(texture, noise)
    
    cv2.imwrite(os.path.join(output_path, 'mars_texture.png'), texture)
    print(f"Assets generated in {output_path}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    create_mars_assets(script_dir)


3. The Drone Model (models/drone/model.sdf)

No changes required to the drone model itself, ensuring it still interfaces with ROS 2 Jazzy.

4. Execution Steps

Generate Assets:

cd ~/drone_sim_ws/src/arena_drone_sim/scripts
python3 generate_mars_assets.py


Build and Source:

cd ~/drone_sim_ws
colcon build --symlink-install
source install/setup.bash


Run Simulation:

ros2 launch arena_drone_sim sim.launch.py

