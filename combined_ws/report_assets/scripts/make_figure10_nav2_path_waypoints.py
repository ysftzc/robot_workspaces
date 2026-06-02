#!/usr/bin/env python3
"""Create Figure 10 by drawing the Nav2 route and mission waypoints on the map."""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = ROOT / "src" / "combined_robot" / "maps"
CONFIG_DIR = ROOT / "src" / "combined_robot" / "config"
FIG_DIR = ROOT / "report_assets" / "figures"
DOWNLOADS_DIR = Path.home() / "Downloads"

MAP_YAML = MAP_DIR / "my_map.yaml"
MAP_PGM = MAP_DIR / "my_map.pgm"
MASK_PGM = MAP_DIR / "keepout_mask.pgm"
WAYPOINTS_YAML = CONFIG_DIR / "sera_waypoints.yaml"
INITIAL_POSE_YAML = CONFIG_DIR / "sera_initial_pose.yaml"

ROUTE_NAME = "center_corridor_patrol"
OUT_PNG = FIG_DIR / "figure10_nav2_global_path_waypoints.png"
OUT_PDF = FIG_DIR / "figure10_nav2_global_path_waypoints.pdf"


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def colorize_occupancy_grid(gray: Image.Image) -> Image.Image:
    gray = gray.convert("L")
    out = Image.new("RGB", gray.size, "white")
    px_in = gray.load()
    px_out = out.load()
    for y in range(gray.height):
        for x in range(gray.width):
            v = px_in[x, y]
            if v < 80:
                px_out[x, y] = (25, 25, 25)
            elif v > 230:
                px_out[x, y] = (255, 255, 255)
            else:
                px_out[x, y] = (205, 205, 205)
    return out


def mask_alpha(mask: Image.Image) -> Image.Image:
    mask = mask.convert("L")
    alpha = Image.new("L", mask.size, 0)
    src = mask.load()
    dst = alpha.load()
    for y in range(mask.height):
        for x in range(mask.width):
            if src[x, y] < 128:
                dst[x, y] = 75
    return alpha


def add_keepout_overlay(map_img: Image.Image, mask_img: Image.Image, scale: int) -> Image.Image:
    overlay = Image.new("RGBA", map_img.size, (215, 48, 39, 0))
    overlay.putalpha(mask_alpha(mask_img))
    composed = Image.alpha_composite(map_img.convert("RGBA"), overlay)
    composed = composed.resize((composed.width * scale, composed.height * scale), Image.Resampling.NEAREST)
    mask_scaled = mask_img.resize((mask_img.width * scale, mask_img.height * scale), Image.Resampling.NEAREST)

    hatch = Image.new("RGBA", composed.size, (0, 0, 0, 0))
    hatch_draw = ImageDraw.Draw(hatch)
    for x in range(-composed.height, composed.width, 30):
        hatch_draw.line((x, composed.height, x + composed.height, 0), fill=(150, 18, 25, 70), width=2)

    hatch_alpha = Image.new("L", composed.size, 0)
    hp = hatch_alpha.load()
    mp = mask_scaled.load()
    for y in range(mask_scaled.height):
        for x in range(mask_scaled.width):
            if mp[x, y] < 128:
                hp[x, y] = 70
    hatch.putalpha(hatch_alpha)
    return Image.alpha_composite(composed, hatch)


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def coalesce_waypoints(waypoints: list[dict], tolerance: float = 0.05) -> list[dict]:
    markers: list[dict] = []
    for waypoint in waypoints:
        if markers:
            last = markers[-1]
            if math.hypot(waypoint["x"] - last["x"], waypoint["y"] - last["y"]) <= tolerance:
                last["names"].append(waypoint["name"])
                last["yaw"] = waypoint["yaw"]
                continue
        markers.append(
            {
                "x": waypoint["x"],
                "y": waypoint["y"],
                "yaw": waypoint["yaw"],
                "names": [waypoint["name"]],
            }
        )
    return markers


def yaw_arrow_points(cx: int, cy: int, yaw: float, length: int = 42) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
    # ROS map yaw uses +x forward and +y left; image y is inverted.
    ex = cx + int(math.cos(yaw) * length)
    ey = cy - int(math.sin(yaw) * length)
    left = yaw + math.radians(150)
    right = yaw - math.radians(150)
    lh = (ex + int(math.cos(left) * 13), ey - int(math.sin(left) * 13))
    rh = (ex + int(math.cos(right) * 13), ey - int(math.sin(right) * 13))
    return (cx, cy), (ex, ey), lh, rh


def draw_arrow(draw: ImageDraw.ImageDraw, cx: int, cy: int, yaw: float, color: tuple[int, int, int], width: int = 4) -> None:
    start, end, lh, rh = yaw_arrow_points(cx, cy, yaw)
    draw.line((*start, *end), fill=color, width=width)
    draw.polygon((end, lh, rh), fill=color)


def draw_marker(
    draw: ImageDraw.ImageDraw,
    x: int,
    y: int,
    label: str,
    fill: tuple[int, int, int],
    font: ImageFont.ImageFont,
    radius: int = 17,
) -> None:
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=fill, outline=(255, 255, 255), width=4)
    bbox = draw.textbbox((0, 0), label, font=font)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    draw.text((x - tw / 2, y - th / 2 - 1), label, fill=(255, 255, 255), font=font)


def draw_label_box(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str, font: ImageFont.ImageFont) -> None:
    x, y = xy
    pad_x = 9
    pad_y = 5
    bbox = draw.textbbox((x, y), text, font=font)
    rect = (bbox[0] - pad_x, bbox[1] - pad_y, bbox[2] + pad_x, bbox[3] + pad_y)
    draw.rounded_rectangle(rect, radius=6, fill=(255, 255, 255), outline=(170, 170, 170), width=1)
    draw.text((x, y), text, fill=(35, 35, 35), font=font)


def draw_legend(draw: ImageDraw.ImageDraw, x: int, y: int, font: ImageFont.ImageFont, bold_font: ImageFont.ImageFont) -> None:
    draw.line((x, y + 15, x + 52, y + 15), fill=(25, 96, 220), width=7)
    draw.text((x + 68, y), "Nav2 global path", fill=(35, 35, 35), font=font)

    sx = x + 330
    draw.ellipse((sx, y, sx + 30, y + 30), fill=(32, 139, 58), outline=(255, 255, 255), width=3)
    draw.text((sx + 45, y), "Start pose", fill=(35, 35, 35), font=font)

    wx = x + 575
    draw.ellipse((wx, y, wx + 30, y + 30), fill=(235, 128, 25), outline=(255, 255, 255), width=3)
    draw.text((wx + 45, y), "Waypoint", fill=(35, 35, 35), font=font)

    gx = x + 790
    draw.ellipse((gx, y, gx + 30, y + 30), fill=(127, 63, 152), outline=(255, 255, 255), width=3)
    draw.text((gx + 45, y), "Goal", fill=(35, 35, 35), font=font)

    kx = x + 970
    draw.rectangle((kx, y + 2, kx + 30, y + 30), fill=(215, 48, 39), outline=(130, 30, 35), width=2)
    draw.text((kx + 45, y), "Keepout", fill=(35, 35, 35), font=font)

    route_label = "Route source:"
    route_y = y + 54
    draw.text((x, route_y), route_label, fill=(70, 70, 70), font=font)
    label_box = draw.textbbox((x, route_y), route_label, font=font)
    draw.text((label_box[2] + 18, route_y), ROUTE_NAME, fill=(35, 35, 35), font=bold_font)


def draw_scale_bar(draw: ImageDraw.ImageDraw, x: int, y: int, scale: int, resolution: float, font: ImageFont.ImageFont) -> None:
    bar_m = 2.0
    bar_px = int((bar_m / resolution) * scale)
    draw.line((x, y, x + bar_px, y), fill=(20, 20, 20), width=5)
    draw.line((x, y - 10, x, y + 10), fill=(20, 20, 20), width=4)
    draw.line((x + bar_px, y - 10, x + bar_px, y + 10), fill=(20, 20, 20), width=4)
    draw.text((x + bar_px + 16, y - 17), "2 m", fill=(35, 35, 35), font=font)


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    DOWNLOADS_DIR.mkdir(parents=True, exist_ok=True)

    map_meta = load_yaml(MAP_YAML)
    resolution = float(map_meta["resolution"])
    origin_x, origin_y = map(float, map_meta["origin"][:2])

    initial_pose = load_yaml(INITIAL_POSE_YAML)["initial_pose"]
    route = load_yaml(WAYPOINTS_YAML)["routes"][ROUTE_NAME]
    route_waypoints = route["waypoints"]

    raw_map = Image.open(MAP_PGM).convert("L")
    colored = colorize_occupancy_grid(raw_map)
    mask = Image.open(MASK_PGM).convert("L")

    scale = 5
    map_layer = add_keepout_overlay(colored, mask, scale)

    margin_x = 90
    margin_top = 80
    margin_bottom = 205
    canvas_w = map_layer.width + 2 * margin_x
    canvas_h = map_layer.height + margin_top + margin_bottom
    canvas = Image.new("RGB", (canvas_w, canvas_h), (248, 249, 250))

    map_x = margin_x
    map_y = margin_top
    shadow = Image.new("RGBA", (map_layer.width + 16, map_layer.height + 16), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    sd.rectangle((8, 8, map_layer.width + 8, map_layer.height + 8), fill=(0, 0, 0, 24))
    canvas.paste(shadow.convert("RGB"), (map_x - 8, map_y - 8))
    canvas.paste(map_layer.convert("RGB"), (map_x, map_y))

    def world_to_canvas(x: float, y: float) -> tuple[int, int]:
        px = (x - origin_x) / resolution
        py = raw_map.height - 1 - ((y - origin_y) / resolution)
        return int(round(map_x + px * scale)), int(round(map_y + py * scale))

    draw = ImageDraw.Draw(canvas)
    draw.rectangle((map_x, map_y, map_x + map_layer.width - 1, map_y + map_layer.height - 1), outline=(80, 80, 80), width=2)

    path_points_world = [
        {"x": initial_pose["x"], "y": initial_pose["y"], "yaw": initial_pose["yaw"], "name": "start"},
        *route_waypoints,
    ]
    path_points = [world_to_canvas(p["x"], p["y"]) for p in path_points_world]

    for width, color in ((14, (255, 255, 255)), (8, (25, 96, 220))):
        if len(path_points) > 1:
            draw.line(path_points, fill=color, width=width, joint="curve")

    # Direction arrows on longer route segments.
    for a, b in zip(path_points[:-1], path_points[1:]):
        ax, ay = a
        bx, by = b
        dist = math.hypot(bx - ax, by - ay)
        if dist < 90:
            continue
        mx = int(ax + (bx - ax) * 0.63)
        my = int(ay + (by - ay) * 0.63)
        yaw_img = math.atan2(by - ay, bx - ax)
        arrow_len = 34
        ex = int(mx + math.cos(yaw_img) * arrow_len)
        ey = int(my + math.sin(yaw_img) * arrow_len)
        left = yaw_img + math.radians(150)
        right = yaw_img - math.radians(150)
        draw.line((mx, my, ex, ey), fill=(20, 77, 176), width=6)
        draw.polygon(
            (
                (ex, ey),
                (int(ex + math.cos(left) * 13), int(ey + math.sin(left) * 13)),
                (int(ex + math.cos(right) * 13), int(ey + math.sin(right) * 13)),
            ),
            fill=(20, 77, 176),
        )

    font = load_font(26)
    small_font = load_font(23)
    marker_font = load_font(23, bold=True)
    bold_font = load_font(26, bold=True)

    sx, sy = world_to_canvas(initial_pose["x"], initial_pose["y"])
    draw_marker(draw, sx, sy, "S", (32, 139, 58), marker_font)
    draw_arrow(draw, sx, sy, initial_pose["yaw"], (32, 139, 58), width=4)
    draw_label_box(draw, (sx - 42, sy - 60), "Initial pose", small_font)

    markers = coalesce_waypoints(route_waypoints)
    label_offsets = [
        (22, -42),
        (-58, -42),
        (-62, 28),
        (24, 30),
        (-62, -42),
        (-42, 30),
        (-72, -42),
        (24, -42),
    ]
    for index, waypoint in enumerate(markers, start=1):
        wx, wy = world_to_canvas(waypoint["x"], waypoint["y"])
        is_goal = index == len(markers)
        color = (127, 63, 152) if is_goal else (235, 128, 25)
        label = "G" if is_goal else str(index)
        draw_marker(draw, wx, wy, label, color, marker_font)
        draw_arrow(draw, wx, wy, waypoint["yaw"], color, width=3)
        if index in (1, 2, 3, 4, 5, len(markers)):
            ox, oy = label_offsets[(index - 1) % len(label_offsets)]
            text = "Goal" if is_goal else f"W{index}"
            draw_label_box(draw, (wx + ox, wy + oy), text, small_font)

    legend_y = map_y + map_layer.height + 46
    draw_legend(draw, margin_x, legend_y, font, bold_font)
    draw_scale_bar(draw, canvas_w - margin_x - 300, map_y + map_layer.height + 94, scale, resolution, font)

    canvas.save(OUT_PNG, quality=95)
    canvas.save(OUT_PDF, resolution=300.0)

    for path in (OUT_PNG, OUT_PDF):
        shutil.copy2(path, DOWNLOADS_DIR / path.name)
        print(path)
        print(DOWNLOADS_DIR / path.name)


if __name__ == "__main__":
    main()
