#!/usr/bin/env python3
"""Batch-optimize tomato GLB assets for Gazebo.

Run with Blender:
  blender -b --python src/combined_robot/scripts/optimize_tomato_glbs.py

The script keeps the tomato geometry and base-color texture, but removes
metallic/roughness texture links that trigger Assimp failures in Gazebo.
Original copied GLBs are preserved as *.source.glb. Source symlink targets are
left untouched; the symlink is replaced by a local optimized GLB.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import bpy


ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = ROOT / "src" / "combined_robot" / "models"
MAX_TEXTURE_SIZE = 512

TOMATO_MESHES = (
    ("tomato_ripe", "tomatoripe.glb"),
    ("tomato_rotten", "tomatorotten.glb"),
    ("tomato_diseased", "tomatodiseased.glb"),
    ("tomato_unripe", "tomatounripe.glb"),
    ("tomato_ripe2", "tomatoripe2.glb"),
    ("tomato_rotten2", "tomatorotten2.glb"),
    ("tomato_rotten3", "tomatorotten3.glb"),
    ("tomato_rotten4", "tomatorotten4.glb"),
    ("tomato_rotten5", "tomatorotten5.glb"),
)


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def linked_image_from_socket(socket):
    if not socket or not socket.is_linked:
        return None
    seen = set()
    stack = [link.from_node for link in socket.links]
    while stack:
        node = stack.pop()
        if node in seen:
            continue
        seen.add(node)
        if node.bl_idname == "ShaderNodeTexImage" and node.image:
            return node.image
        for input_socket in getattr(node, "inputs", []):
            if input_socket.is_linked:
                stack.extend(link.from_node for link in input_socket.links)
    return None


def material_base(material):
    color = tuple(material.diffuse_color) if material else (0.8, 0.1, 0.05, 1.0)
    image = None
    if material and material.use_nodes:
        for node in material.node_tree.nodes:
            if node.bl_idname != "ShaderNodeBsdfPrincipled":
                continue
            base_socket = node.inputs.get("Base Color")
            if base_socket:
                color = tuple(base_socket.default_value)
                image = linked_image_from_socket(base_socket)
            break
    if material and material.use_nodes and image is None:
        for node in material.node_tree.nodes:
            if node.bl_idname == "ShaderNodeTexImage" and node.image:
                image = node.image
                break
    return color, image


def scale_image(image) -> None:
    if not image or image.size[0] <= 0 or image.size[1] <= 0:
        return
    width, height = int(image.size[0]), int(image.size[1])
    longest = max(width, height)
    if longest <= MAX_TEXTURE_SIZE:
        return
    scale = MAX_TEXTURE_SIZE / float(longest)
    new_width = max(1, int(width * scale))
    new_height = max(1, int(height * scale))
    try:
        image.scale(new_width, new_height)
    except RuntimeError as exc:
        print(f"WARN image scale skipped for {image.name}: {exc}")


def simplified_material(source):
    color, image = material_base(source)
    mat = bpy.data.materials.new(f"{source.name if source else 'tomato'}_gazebo")
    mat.diffuse_color = color
    mat.use_nodes = True

    nodes = mat.node_tree.nodes
    bsdf = next((n for n in nodes if n.bl_idname == "ShaderNodeBsdfPrincipled"), None)
    if bsdf:
        if bsdf.inputs.get("Base Color"):
            bsdf.inputs["Base Color"].default_value = color
        if bsdf.inputs.get("Metallic"):
            bsdf.inputs["Metallic"].default_value = 0.0
        if bsdf.inputs.get("Roughness"):
            bsdf.inputs["Roughness"].default_value = 0.65
        if image:
            scale_image(image)
            tex = nodes.new(type="ShaderNodeTexImage")
            tex.image = image
            mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def prepare_source(mesh_path: Path) -> Path:
    if mesh_path.is_symlink():
        source = mesh_path.resolve()
        link_note = mesh_path.with_suffix(mesh_path.suffix + ".source_link.txt")
        if not link_note.exists():
            link_note.write_text(str(source) + "\n", encoding="utf-8")
        return source

    backup = mesh_path.with_suffix(mesh_path.suffix + ".source.glb")
    if not backup.exists():
        shutil.copy2(mesh_path, backup)
    return backup


def optimize(mesh_path: Path) -> tuple[int, int]:
    source = prepare_source(mesh_path)
    tmp = mesh_path.with_suffix(mesh_path.suffix + ".optimized_tmp.glb")
    if tmp.exists():
        tmp.unlink()

    reset_scene()
    before = source.stat().st_size
    bpy.ops.import_scene.gltf(filepath=str(source))

    material_map = {}
    for material in list(bpy.data.materials):
        material_map[material] = simplified_material(material)

    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue
        for slot in obj.material_slots:
            if slot.material in material_map:
                slot.material = material_map[slot.material]

    bpy.ops.outliner.orphans_purge(do_recursive=True)
    bpy.ops.export_scene.gltf(
        filepath=str(tmp),
        export_format="GLB",
        export_materials="EXPORT",
        export_texcoords=True,
        export_normals=True,
    )
    tmp.replace(mesh_path)
    after = mesh_path.stat().st_size
    return before, after


def main() -> None:
    for model_name, mesh_name in TOMATO_MESHES:
        mesh_path = MODELS_DIR / model_name / "meshes" / mesh_name
        if not mesh_path.exists():
            print(f"SKIP missing {mesh_path}")
            continue
        before, after = optimize(mesh_path)
        print(f"{model_name}: {before / 1048576:.1f} MB -> {after / 1048576:.1f} MB")


if __name__ == "__main__":
    main()
