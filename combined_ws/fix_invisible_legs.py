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
                for col in link.findall('collision'):
                    if col.get('name') == 'invisible_legs':
                        # Found the invisible legs we added!
                        geom = col.find('geometry')
                        if geom is not None:
                            box = geom.find('box')
                            if box is not None:
                                size = box.find('size')
                                if size is not None:
                                    # The soilbeds are individual 1x1 units, not one giant 6m row!
                                    # We set them to 5.8 before, which filled the whole greenhouse.
                                    # Change width/depth to 0.8m x 0.8m, height 0.4m
                                    size.text = '0.8 0.8 0.4'
                                    changed = True
        
        if changed:
            tree.write(sdf_file, encoding='utf-8', xml_declaration=True)
            fixed_count += 1
    except Exception as e:
        print(f"Error processing {sdf_file}: {e}")

print(f"Shrunk invisible legs to proper size for {fixed_count} soilbeds!")
