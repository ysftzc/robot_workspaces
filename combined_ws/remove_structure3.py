import os
import xml.etree.ElementTree as ET

model_path = os.path.expanduser('~/.gz/models/structure_0/model.sdf')

if os.path.exists(model_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    changed = False
    
    for link in root.findall('.//link'):
        # Remove collision3
        for col in link.findall('collision'):
            if col.get('name') == 'collision3':
                link.remove(col)
                changed = True
                print("Removed collision3")
        
        # Remove visual3
        for vis in link.findall('visual'):
            if vis.get('name') == 'visual3':
                link.remove(vis)
                changed = True
                print("Removed visual3")
                
    if changed:
        tree.write(model_path, encoding='utf-8', xml_declaration=True)
        print("Successfully removed the artificial floor (Structure3)!")
    else:
        print("Structure3 already removed or not found.")
else:
    print(f"{model_path} not found!")
