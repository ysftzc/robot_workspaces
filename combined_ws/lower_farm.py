import xml.etree.ElementTree as ET
import os

world_file = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds/tomato_farm_px4_complete.sdf'

tree = ET.parse(world_file)
root = tree.getroot()
changed = False

for include in root.findall('.//include'):
    name = include.find('name')
    if name is not None and name.text:
        if 'soilbed' in name.text or 'tomato' in name.text or 'flowerpot' in name.text or 'metal' in name.text:
            pose = include.find('pose')
            if pose is not None and pose.text:
                parts = pose.text.strip().split()
                if len(parts) == 6:
                    x, y, z, r, p, yw = parts
                    new_z = float(z) - 0.40
                    pose.text = f"{x} {y} {new_z:.2f} {r} {p} {yw}"
                    changed = True

if changed:
    tree.write(world_file, encoding='utf-8', xml_declaration=True)
    print("Successfully lowered all farm objects to the ground!")
else:
    print("No objects were changed.")
