import os
import glob
import xml.etree.ElementTree as ET

models_dir = os.path.expanduser('~/.gz/models')

# Find all model.sdf files
sdf_files = glob.glob(os.path.join(models_dir, '*', 'model.sdf'))

fixed_count = 0
for sdf_file in sdf_files:
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()
        
        changed = False
        for model in root.findall('model'):
            for link in model.findall('link'):
                collision = link.find('collision')
                visual = link.find('visual')
                
                if collision is not None and visual is not None:
                    col_geom = collision.find('geometry')
                    vis_geom = visual.find('geometry')
                    
                    # If collision is a 1x1x1 box and visual is a mesh
                    if col_geom is not None and vis_geom is not None:
                        box = col_geom.find('box')
                        if box is not None:
                            size = box.find('size')
                            if size is not None and size.text.strip() == '1 1 1':
                                # Copy visual geometry to collision geometry
                                col_geom.clear()
                                mesh = vis_geom.find('mesh')
                                if mesh is not None:
                                    # Create new mesh element
                                    new_mesh = ET.SubElement(col_geom, 'mesh')
                                    uri = mesh.find('uri')
                                    scale = mesh.find('scale')
                                    if uri is not None:
                                        ET.SubElement(new_mesh, 'uri').text = uri.text
                                    if scale is not None:
                                        ET.SubElement(new_mesh, 'scale').text = scale.text
                                    changed = True
        if changed:
            tree.write(sdf_file, encoding='utf-8', xml_declaration=True)
            fixed_count += 1
    except Exception as e:
        print(f"Error processing {sdf_file}: {e}")

print(f"Fixed {fixed_count} SDF files with correct mesh collisions!")
