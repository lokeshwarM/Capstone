import random

def generate_city_sdf(filename="city.sdf", grid_size=6, spacing=15, max_height=40):
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
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

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
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""

    building_id = 0
    helipad_id = 0

    random.seed(42) # Consistent city

    # Generate grid of buildings
    offset = (grid_size * spacing) / 2.0
    for x in range(grid_size):
        for y in range(grid_size):
            # Skip some spots for streets/parks
            if random.random() < 0.2:
                continue

            pos_x = (x * spacing) - offset
            pos_y = (y * spacing) - offset
            height = random.uniform(10, max_height)
            width = random.uniform(6, 12)
            depth = random.uniform(6, 12)
            
            z = height / 2.0

            # Building Model
            sdf_content += f"""
    <model name="building_{building_id}">
      <pose>{pos_x} {pos_y} {z} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{width} {depth} {height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{width} {depth} {height}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.7 1</ambient>
            <diffuse>0.6 0.6 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
            
            # Put a helipad on about 25% of the buildings
            if random.random() < 0.25 and helipad_id < 8:
                pad_z = height + 0.1 # slightly above roof
                sdf_content += f"""
    <model name="helipad_{helipad_id}">
      <pose>{pos_x} {pos_y} {pad_z} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>2.0</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 1.0 0.0 1</ambient>
            <diffuse>1.0 1.0 0.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
                helipad_id += 1
            
            building_id += 1

    sdf_content += """
  </world>
</sdf>
"""
    with open(filename, 'w') as f:
        f.write(sdf_content)
    print(f"Generated Gazebo world: {filename} with {building_id} buildings and {helipad_id} helipads.")

if __name__ == "__main__":
    generate_city_sdf("d:/Capstone/city.sdf")
