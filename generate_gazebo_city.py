import random
import math

def generate_city_sdf(filename="city.sdf", grid_size=6, spacing=15, max_height=40):
    """
    Generates a realistic Gazebo city world with:
    - Buildings on a grid
    - Helipads on rooftops (delivery destinations)
    - Roads and footpaths between building rows
    - Parked/static cars on roads
    - Pedestrians on footpaths and rooftops
    - ROS 2 state plugin for drone/truck control
    """

    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="city_world">
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Sky ambient light -->
    <light type="directional" name="ambient_sky">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.4 0.4 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.5 -0.1 -0.5</direction>
    </light>

    <!-- Dark asphalt ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>500 500</size></plane>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

    random.seed(42)  # Consistent city layout

    building_id = 0
    helipad_id = 0
    offset = (grid_size * spacing) / 2.0

    # ---------------------------------------------------------------
    # Track which grid cells have buildings for later use
    # ---------------------------------------------------------------
    has_building = {}

    for x in range(grid_size):
        for y in range(grid_size):
            if random.random() < 0.2:
                has_building[(x, y)] = False
            else:
                has_building[(x, y)] = True

    # ---------------------------------------------------------------
    # ROADS — horizontal and vertical strips between all building rows
    # ---------------------------------------------------------------
    road_width = 8.0
    road_thickness = 0.02
    road_half = road_width / 2.0
    world_extent = offset + road_half + 5

    # Vertical roads (running along X axis, separating Y columns)
    for y in range(grid_size + 1):
        ry = (y * spacing) - offset - spacing / 2.0
        road_len = world_extent * 2
        sdf_content += f"""
    <model name="road_vert_{y}">
      <pose>{0} {ry} {0.01} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{road_len} {road_width} {road_thickness}</size></box></geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    # Horizontal roads (running along Y axis, separating X rows)
    for x in range(grid_size + 1):
        rx = (x * spacing) - offset - spacing / 2.0
        road_len = world_extent * 2
        sdf_content += f"""
    <model name="road_horiz_{x}">
      <pose>{rx} {0} {0.01} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{road_width} {road_len} {road_thickness}</size></box></geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    # ---------------------------------------------------------------
    # FOOTPATHS — thin lighter strips alongside each road
    # ---------------------------------------------------------------
    footpath_width = 2.0
    footpath_thickness = 0.03
    fp_color = "0.55 0.55 0.5 1"

    for y in range(grid_size + 1):
        ry = (y * spacing) - offset - spacing / 2.0
        road_len = world_extent * 2
        for side in [-1, 1]:
            fy = ry + side * (road_width / 2.0 + footpath_width / 2.0)
            sdf_content += f"""
    <model name="footpath_vert_{y}_s{side}">
      <pose>{0} {fy} {0.015} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{road_len} {footpath_width} {footpath_thickness}</size></box></geometry>
          <material><ambient>{fp_color}</ambient><diffuse>{fp_color}</diffuse></material>
        </visual>
      </link>
    </model>"""

    for x in range(grid_size + 1):
        rx = (x * spacing) - offset - spacing / 2.0
        road_len = world_extent * 2
        for side in [-1, 1]:
            fx = rx + side * (road_width / 2.0 + footpath_width / 2.0)
            sdf_content += f"""
    <model name="footpath_horiz_{x}_s{side}">
      <pose>{fx} {0} {0.015} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{footpath_width} {road_len} {footpath_thickness}</size></box></geometry>
          <material><ambient>{fp_color}</ambient><diffuse>{fp_color}</diffuse></material>
        </visual>
      </link>
    </model>"""

    # ---------------------------------------------------------------
    # ROAD MARKINGS — white dashed centre lines
    # ---------------------------------------------------------------
    dash_len = 3.0
    dash_gap = 3.0
    dash_w = 0.3
    dash_t = 0.025

    for y in range(grid_size + 1):
        ry = (y * spacing) - offset - spacing / 2.0
        for dash_i, dx in enumerate([x * (dash_len + dash_gap) - world_extent for x in range(int(world_extent * 2 / (dash_len + dash_gap)))]):
            sdf_content += f"""
    <model name="dash_v_{y}_{dash_i}">
      <pose>{dx} {ry} {0.025} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{dash_len} {dash_w} {dash_t}</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>"""

    # ---------------------------------------------------------------
    # BUILDINGS
    # ---------------------------------------------------------------
    building_id = 0
    helipad_id = 0
    helipad_positions = []
    random.seed(42)  # reset seed for determinism

    for x in range(grid_size):
        for y in range(grid_size):
            if random.random() < 0.2:
                continue

            pos_x = (x * spacing) - offset
            pos_y = (y * spacing) - offset
            height = random.uniform(10, max_height)
            width = random.uniform(6, 12)
            depth = random.uniform(6, 12)
            z = height / 2.0

            # Window texture colour variation
            r = random.uniform(0.45, 0.7)
            g = random.uniform(0.45, 0.7)
            b = random.uniform(0.55, 0.85)

            sdf_content += f"""
    <model name="building_{building_id}">
      <pose>{pos_x} {pos_y} {z} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{width} {depth} {height}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{width} {depth} {height}</size></box></geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""

            # Helipad on ~25% of buildings
            if random.random() < 0.25 and helipad_id < 8:
                pad_z = height + 0.1
                helipad_positions.append((pos_x, pos_y, height))
                sdf_content += f"""
    <model name="helipad_{helipad_id}">
      <pose>{pos_x} {pos_y} {pad_z} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>2.5</radius><length>0.15</length></cylinder></geometry>
          <material><ambient>1.0 0.8 0.0 1</ambient><diffuse>1.0 0.8 0.0 1</diffuse></material>
        </visual>
        <!-- H marking -->
        <visual name="h_bar">
          <geometry><box><size>2.5 0.4 0.05</size></box></geometry>
          <pose>0 0 0.08 0 0 0</pose>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>"""
                helipad_id += 1

            building_id += 1

    # ---------------------------------------------------------------
    # PARKED / MOVING CARS (static models on roads)
    # ---------------------------------------------------------------
    car_colors = [
        ("0.8 0.1 0.1 1", "0.8 0.1 0.1 1"),   # red
        ("0.1 0.2 0.8 1", "0.1 0.2 0.8 1"),   # blue
        ("0.9 0.9 0.9 1", "0.9 0.9 0.9 1"),   # white
        ("0.3 0.3 0.3 1", "0.3 0.3 0.3 1"),   # dark grey
        ("1.0 0.55 0.0 1", "1.0 0.55 0.0 1"), # orange
        ("0.1 0.5 0.1 1", "0.1 0.5 0.1 1"),   # green
        ("0.6 0.6 0.7 1", "0.6 0.6 0.7 1"),   # silver
        ("0.5 0.1 0.5 1", "0.5 0.1 0.5 1"),   # purple
    ]

    car_positions = [
        (-37.5, -22.5, 0), (37.5, -22.5, 0),
        (-22.5, -37.5, 90), (22.5, 37.5, 90),
        (-7.5, -37.5, 0),  (7.5, 22.5, 0),
        (-37.5, 7.5, 90),  (22.5, -7.5, 90),
    ]

    for ci, (cx, cy, cyaw, ) in enumerate(car_positions):
        amb, diff = car_colors[ci % len(car_colors)]
        yaw_rad = math.radians(cyaw)
        sdf_content += f"""
    <model name="car_{ci}">
      <pose>{cx} {cy} 0.6 0 0 {yaw_rad:.4f}</pose>
      <static>true</static>
      <link name="link">
        <!-- Car body -->
        <visual name="body">
          <geometry><box><size>4.5 2.0 1.2</size></box></geometry>
          <material><ambient>{amb}</ambient><diffuse>{diff}</diffuse></material>
        </visual>
        <!-- Car cabin -->
        <visual name="cabin">
          <pose>-0.2 0 0.8 0 0 0</pose>
          <geometry><box><size>2.2 1.8 0.9</size></box></geometry>
          <material><ambient>0.6 0.75 0.85 0.7</ambient><diffuse>0.6 0.75 0.85 0.7</diffuse></material>
        </visual>
        <!-- Wheels -->
        <visual name="wfl">
          <pose>1.5 1.1 -0.3 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="wfr">
          <pose>1.5 -1.1 -0.3 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="wbl">
          <pose>-1.5 1.1 -0.3 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name="wbr">
          <pose>-1.5 -1.1 -0.3 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.35</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>"""

    # ---------------------------------------------------------------
    # PEDESTRIANS — simple capsule-shaped figures
    # ---------------------------------------------------------------
    pedestrian_spots = [
        (-37.5, -18.0, 0), (-22.5, -42.0, 0), (7.5, -42.0, 0),
        (-42.0, 7.5, 0), (42.0, -7.5, 0),  (-7.5, 42.0, 0),
        (22.5, 42.0, 0),  (-37.5, 37.5, 0),
    ]

    # A couple on rooftops
    if len(helipad_positions) >= 2:
        hp1 = helipad_positions[0]
        hp2 = helipad_positions[1]
        pedestrian_spots.append((hp1[0] + 1.5, hp1[1] + 1.5, hp1[2] + 0.1))
        pedestrian_spots.append((hp2[0] - 1.5, hp2[1] + 1.0, hp2[2] + 0.1))

    skin_tones = [
        "0.87 0.72 0.53 1",
        "0.6 0.4 0.2 1",
        "0.95 0.82 0.7 1",
        "0.4 0.25 0.1 1",
    ]
    shirt_colors = [
        "0.9 0.1 0.1 1", "0.1 0.1 0.9 1", "0.1 0.7 0.1 1",
        "0.9 0.6 0.1 1", "0.5 0.1 0.7 1", "0.1 0.7 0.7 1",
        "0.9 0.9 0.1 1", "0.9 0.3 0.5 1", "0.5 0.5 0.5 1",
        "0.2 0.5 0.3 1",
    ]
    trousers = "0.1 0.1 0.3 1"

    for pi, (px, py, pz) in enumerate(pedestrian_spots):
        skin = skin_tones[pi % len(skin_tones)]
        shirt = shirt_colors[pi % len(shirt_colors)]
        sdf_content += f"""
    <model name="pedestrian_{pi}">
      <pose>{px} {py} {pz} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <!-- Body/torso -->
        <visual name="torso">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><box><size>0.35 0.25 0.55</size></box></geometry>
          <material><ambient>{shirt}</ambient><diffuse>{shirt}</diffuse></material>
        </visual>
        <!-- Head -->
        <visual name="head">
          <pose>0 0 1.55 0 0 0</pose>
          <geometry><sphere><radius>0.14</radius></sphere></geometry>
          <material><ambient>{skin}</ambient><diffuse>{skin}</diffuse></material>
        </visual>
        <!-- Left leg -->
        <visual name="leg_l">
          <pose>0.09 0 0.55 0 0 0</pose>
          <geometry><box><size>0.13 0.13 0.6</size></box></geometry>
          <material><ambient>{trousers}</ambient><diffuse>{trousers}</diffuse></material>
        </visual>
        <!-- Right leg -->
        <visual name="leg_r">
          <pose>-0.09 0 0.55 0 0 0</pose>
          <geometry><box><size>0.13 0.13 0.6</size></box></geometry>
          <material><ambient>{trousers}</ambient><diffuse>{trousers}</diffuse></material>
        </visual>
        <!-- Left arm -->
        <visual name="arm_l">
          <pose>0 0.22 1.1 0 0 0</pose>
          <geometry><box><size>0.11 0.12 0.5</size></box></geometry>
          <material><ambient>{skin}</ambient><diffuse>{skin}</diffuse></material>
        </visual>
        <!-- Right arm -->
        <visual name="arm_r">
          <pose>0 -0.22 1.1 0 0 0</pose>
          <geometry><box><size>0.11 0.12 0.5</size></box></geometry>
          <material><ambient>{skin}</ambient><diffuse>{skin}</diffuse></material>
        </visual>
      </link>
    </model>"""

    sdf_content += """
  </world>
</sdf>
"""

    with open(filename, 'w') as f:
        f.write(sdf_content)

    print(f"Generated Gazebo world: {filename}")
    print(f"  Buildings: {building_id}")
    print(f"  Helipads:  {helipad_id}")
    print(f"  Cars:      {len(car_positions)}")
    print(f"  Pedestrians: {len(pedestrian_spots)}")
    print(f"  Helipad delivery positions: {helipad_positions}")
    return helipad_positions

if __name__ == "__main__":
    positions = generate_city_sdf("d:/Capstone/city.sdf")
    print("\nCopy these helipad positions into manual_control.py destinations[]:")
    for p in positions:
        print(f"  ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}),")
