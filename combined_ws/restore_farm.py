import xml.etree.ElementTree as ET
import os

world_file = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds/tomato_farm_px4_complete.sdf'

tree = ET.parse(world_file)
root = tree.getroot()
changed = False

for include in root.findall('.//include'):
    name = include.find('name')
    if name is not None and name.text:
        pose = include.find('pose')
        if pose is not None and pose.text:
            parts = pose.text.strip().split()
            if len(parts) == 6:
                x, y, z, r, p, yw = parts
                if 'soilbed' in name.text:
                    z = '0.40'
                elif 'flowerpot' in name.text:
                    z = '0.50'
                elif 'tomato' in name.text:
                    z = '0.55'
                elif 'metal' in name.text:
                    z = '-0.34'
                else:
                    continue
                pose.text = f"{x} {y} {z} {r} {p} {yw}"
                changed = True

if changed:
    tree.write(world_file, encoding='utf-8', xml_declaration=True)
    print("Successfully RESTORED all farm objects to Fonyuy45 EXACT defaults!")
else:
    print("No objects were changed.")
