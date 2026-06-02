#!/usr/bin/env python3
"""Create Figure 8 from the SLAM Toolbox occupancy grid map."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = ROOT / "src" / "combined_robot" / "maps"
FIG_DIR = ROOT / "report_assets" / "figures"

MAP_PGM = MAP_DIR / "my_map.pgm"
OUT_PNG = FIG_DIR / "figure08_slam_toolbox_greenhouse_map.png"
OUT_PDF = FIG_DIR / "figure08_slam_toolbox_greenhouse_map.pdf"

RESOLUTION_M_PER_PIXEL = 0.05


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
    """Map ROS occupancy grid colors to a clean report palette."""
    gray = gray.convert("L")
    out = Image.new("RGB", gray.size, "white")
    px_in = gray.load()
    px_out = out.load()
    for y in range(gray.height):
        for x in range(gray.width):
            v = px_in[x, y]
            if v < 80:
                px_out[x, y] = (20, 20, 20)       # occupied
            elif v > 230:
                px_out[x, y] = (255, 255, 255)    # free
            else:
                px_out[x, y] = (205, 205, 205)    # unknown
    return out


def draw_legend(draw: ImageDraw.ImageDraw, x: int, y: int, font: ImageFont.ImageFont) -> None:
    items = [
        ((20, 20, 20), "Occupied"),
        ((255, 255, 255), "Free"),
        ((205, 205, 205), "Unknown"),
    ]
    sw = 28
    gap = 150
    for i, (color, label) in enumerate(items):
        ix = x + i * gap
        draw.rectangle((ix, y, ix + sw, y + sw), fill=color, outline=(60, 60, 60), width=2)
        draw.text((ix + sw + 10, y - 1), label, fill=(35, 35, 35), font=font)


def draw_scale_bar(draw: ImageDraw.ImageDraw, x: int, y: int, scale: int, font: ImageFont.ImageFont) -> None:
    bar_m = 2.0
    bar_px = int((bar_m / RESOLUTION_M_PER_PIXEL) * scale)
    draw.line((x, y, x + bar_px, y), fill=(20, 20, 20), width=5)
    draw.line((x, y - 10, x, y + 10), fill=(20, 20, 20), width=4)
    draw.line((x + bar_px, y - 10, x + bar_px, y + 10), fill=(20, 20, 20), width=4)
    draw.text((x + bar_px + 16, y - 17), "2 m", fill=(35, 35, 35), font=font)


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    raw = Image.open(MAP_PGM).convert("L")
    colored = colorize_occupancy_grid(raw)

    scale = 5
    map_img = colored.resize((colored.width * scale, colored.height * scale), Image.Resampling.NEAREST)

    margin_x = 90
    margin_top = 80
    margin_bottom = 165
    canvas_w = map_img.width + 2 * margin_x
    canvas_h = map_img.height + margin_top + margin_bottom

    canvas = Image.new("RGB", (canvas_w, canvas_h), (248, 249, 250))
    map_x = margin_x
    map_y = margin_top

    shadow = Image.new("RGBA", (map_img.width + 16, map_img.height + 16), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    sd.rectangle((8, 8, map_img.width + 8, map_img.height + 8), fill=(0, 0, 0, 24))
    canvas.paste(shadow.convert("RGB"), (map_x - 8, map_y - 8))

    canvas.paste(map_img, (map_x, map_y))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((map_x, map_y, map_x + map_img.width - 1, map_y + map_img.height - 1), outline=(80, 80, 80), width=2)

    font = load_font(26)
    draw_legend(draw, margin_x, map_y + map_img.height + 52, font)
    draw_scale_bar(draw, canvas_w - margin_x - 300, map_y + map_img.height + 70, scale, font)

    canvas.save(OUT_PNG, quality=95)
    canvas.save(OUT_PDF, resolution=300.0)

    print(OUT_PNG)
    print(OUT_PDF)


if __name__ == "__main__":
    main()
