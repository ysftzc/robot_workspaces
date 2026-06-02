import os
import glob
import xml.etree.ElementTree as ET

models_dir = os.path.expanduser('~/.gz/models')
soilbed_sdfs = glob.glob(os.path.join(models_dir, 'soilbed_*', 'model.sdf'))

fixed_count = 0
for sdf_file in soilbed_sdfs:
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()
        changed = False
        
        for model in root.findall('model'):
            for link in model.findall('link'):
                # Check if we already added invisible legs
                has_legs = False
                for col in link.findall('collision'):
                    if col.get('name') == 'invisible_legs':
                        has_legs = True
                        break
                
                if not has_legs:
                    # Add a new collision block for the invisible base
                    # The soilbed visual is roughly 6m long (X), 0.5m wide (Y), and sits at Z=0.4
                    # We want a box from Z=0 to Z=0.4. So size Z=0.4, centered at Z=-0.2 relative to link.
                    legs_col = ET.SubElement(link, 'collision', name='invisible_legs')
                    pose = ET.SubElement(legs_col, 'pose')
                    pose.text = '0 0 -0.2 0 0 0'
                    
                    geom = ET.SubElement(legs_col, 'geometry')
                    box = ET.SubElement(geom, 'box')
                    size = ET.SubElement(box, 'size')
                    # Width ~6m, Depth ~0.8m, Height 0.4m
                    size.text = '5.8 0.8 0.4'
                    
                    changed = True
        
        if changed:
            tree.write(sdf_file, encoding='utf-8', xml_declaration=True)
            fixed_count += 1
    except Exception as e:
        print(f"Error processing {sdf_file}: {e}")

print(f"Added invisible legs to {fixed_count} soilbeds!")
