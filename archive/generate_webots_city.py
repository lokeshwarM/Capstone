import random

def generate_city_wbt(filename="worlds/city.wbt", grid_size=4, block_size=30, road_width=8, max_height=30):
    content = """#VRML_SIM R2023b utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/dji/mavic/protos/Mavic2Pro.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/road/protos/Road.proto"

WorldInfo {
  info [
    "Autonomous Drone City Delivery Simulation"
  ]
  title "City Delivery"
  basicTimeStep 8
  defaultDamping Damping {
    linear 0.5
    angular 0.5
  }
}
Viewpoint {
  orientation -0.27 -0.15 0.95 1.5
  position 0 -100 80
}
TexturedBackground {
}
TexturedBackgroundLight {
}
RectangleArena {
  floorSize 400 400
  floorAppearance PBRAppearance {
    baseColor 0.2 0.2 0.2
    roughness 0.8
    metalness 0
  }
}
"""

    building_id = 0
    helipad_id = 0
    random.seed(42)

    total_size = grid_size * block_size
    offset = total_size / 2.0

    # Draw Roads
    # Horizontal roads
    for y in range(grid_size + 1):
        pos_y = (y * block_size) - offset
        content += f"""
Road {{
  translation 0 0 0.01
  width {road_width}
  wayPoint [
    {-offset - road_width} {pos_y} 0
    {offset + road_width} {pos_y} 0
  ]
}}
"""
    # Vertical roads
    for x in range(grid_size + 1):
        pos_x = (x * block_size) - offset
        content += f"""
Road {{
  translation 0 0 0.02
  width {road_width}
  wayPoint [
    {pos_x} {-offset - road_width} 0
    {pos_x} {offset + road_width} 0
  ]
}}
"""

    # Generate blocks of buildings
    for x in range(grid_size):
        for y in range(grid_size):
            # Center of the block
            block_center_x = (x * block_size) - offset + (block_size / 2)
            block_center_y = (y * block_size) - offset + (block_size / 2)
            
            # Place 1 to 4 buildings per block
            num_buildings = random.randint(1, 4)
            for b in range(num_buildings):
                pos_x = block_center_x + random.uniform(-block_size/4, block_size/4)
                pos_y = block_center_y + random.uniform(-block_size/4, block_size/4)
                
                height = random.uniform(10, max_height)
                width = random.uniform(6, 12)
                depth = random.uniform(6, 12)
                z = height / 2.0
                
                color = f"{random.uniform(0.3, 0.8)} {random.uniform(0.3, 0.8)} {random.uniform(0.3, 0.8)}"

                content += f"""
Solid {{
  translation {pos_x} {pos_y} {z}
  name "building_{building_id}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {color}
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {width} {depth} {height}
      }}
    }}
  ]
  boundingObject Box {{
    size {width} {depth} {height}
  }}
}}
"""
                # Add helipad and drone to a few buildings
                if random.random() < 0.3 and helipad_id < 4:
                    pad_z = height + 0.05
                    content += f"""
Solid {{
  translation {pos_x} {pos_y} {pad_z}
  name "helipad_{helipad_id}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 1 0
        roughness 1
        metalness 0
      }}
      geometry Cylinder {{
        radius 2.0
        height 0.1
      }}
    }}
  ]
}}
Mavic2Pro {{
  translation {pos_x} {pos_y} {pad_z + 0.2}
  name "drone_{helipad_id}"
  controller "drone_controller"
  supervisor TRUE
}}
"""
                    helipad_id += 1
                
                building_id += 1

    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"Generated Webots world: {filename} with {building_id} buildings and {helipad_id} drones.")

if __name__ == "__main__":
    generate_city_wbt("d:/Capstone/worlds/city.wbt")
