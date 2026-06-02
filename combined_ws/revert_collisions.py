import os
import glob
import xml.etree.ElementTree as ET

models_dir = os.path.expanduser('~/.gz/models')
sdf_files = glob.glob(os.path.join(models_dir, '*', 'model.sdf'))

fixed_count = 0
for sdf_file in sdf_files:
    dir_name = os.path.basename(os.path.dirname(sdf_file))
    
    # We only want to revert lamps and tomatoes that might crash dartsim
    if not (dir_name.startswith('lamp') or dir_name.startswith('tomato')):
        continue
        
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()
        changed = False
        
        for model in root.findall('model'):
            for link in model.findall('link'):
                collision = link.find('collision')
                if collision is not None:
                    col_geom = collision.find('geometry')
                    if col_geom is not None:
                        mesh = col_geom.find('mesh')
                        if mesh is not None:
                            # Found a mesh collision that we previously added!
                            # Revert it to the fake 1x1x1 box to prevent crashes
                            col_geom.remove(mesh)
                            box = ET.SubElement(col_geom, 'box')
                            size = ET.SubElement(box, 'size')
                            size.text = '1 1 1'
                            # Or even better, make the box very small so it doesn't clip!
                            if dir_name.startswith('tomato'):
                                size.text = '0.1 0.1 1.0'
                            elif dir_name.startswith('lamp'):
                                size.text = '0.1 0.1 0.1'
                            changed = True
        if changed:
            tree.write(sdf_file, encoding='utf-8', xml_declaration=True)
            fixed_count += 1
    except Exception as e:
        print(f"Error processing {sdf_file}: {e}")

print(f"Reverted {fixed_count} crashing models back to box collisions!")
