#!/usr/bin/env python3
"""Create Figure 9 by overlaying the keepout mask on the SLAM map."""

from collections import deque
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
MAP_DIR = ROOT / "src" / "combined_robot" / "maps"
FIG_DIR = ROOT / "report_assets" / "figures"

MAP_PGM = MAP_DIR / "my_map.pgm"
MASK_PGM = MAP_DIR / "keepout_mask.pgm"
OUT_PNG = FIG_DIR / "figure09_keepout_mask_overlay.png"
OUT_PDF = FIG_DIR / "figure09_keepout_mask_overlay.pdf"

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
    """Black mask cells are forbidden keepout areas."""
    mask = mask.convert("L")
    alpha = Image.new("L", mask.size, 0)
    src = mask.load()
    dst = alpha.load()
    for y in range(mask.height):
        for x in range(mask.width):
            if src[x, y] < 128:
                dst[x, y] = 150
    return alpha


def connected_boxes(mask: Image.Image) -> list[tuple[int, int, int, int]]:
    """Return bounding boxes for contiguous black keepout regions."""
    mask = mask.convert("L")
    w, h = mask.size
    px = mask.load()
    seen = set()
    boxes = []
    for y in range(h):
        for x in range(w):
            if (x, y) in seen or px[x, y] >= 128:
                continue
            q = deque([(x, y)])
            seen.add((x, y))
            min_x = max_x = x
            min_y = max_y = y
            while q:
                cx, cy = q.popleft()
                min_x, max_x = min(min_x, cx), max(max_x, cx)
                min_y, max_y = min(min_y, cy), max(max_y, cy)
                for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
                    if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in seen and px[nx, ny] < 128:
                        seen.add((nx, ny))
                        q.append((nx, ny))
            boxes.append((min_x, min_y, max_x, max_y))
    return boxes


def draw_legend(draw: ImageDraw.ImageDraw, x: int, y: int, font: ImageFont.ImageFont) -> None:
    items = [
        ((215, 48, 39), "Keepout zone"),
        ((25, 25, 25), "Occupied"),
        ((255, 255, 255), "Free"),
        ((205, 205, 205), "Unknown"),
    ]
    sw = 28
    gap = 250
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

    map_img = colorize_occupancy_grid(Image.open(MAP_PGM))
    mask_img = Image.open(MASK_PGM).convert("L")

    overlay = Image.new("RGBA", map_img.size, (215, 48, 39, 0))
    overlay.putalpha(mask_alpha(mask_img))
    composed = Image.alpha_composite(map_img.convert("RGBA"), overlay)

    scale = 5
    composed = composed.resize((composed.width * scale, composed.height * scale), Image.Resampling.NEAREST)
    mask_scaled = mask_img.resize((mask_img.width * scale, mask_img.height * scale), Image.Resampling.NEAREST)

    draw_overlay = ImageDraw.Draw(composed)
    for box in connected_boxes(mask_img):
        x0, y0, x1, y1 = [v * scale for v in box]
        draw_overlay.rectangle((x0, y0, x1 + scale - 1, y1 + scale - 1), outline=(150, 18, 25, 255), width=3)

    # Add subtle diagonal hatch on keepout cells so the mask remains visible in print.
    hatch = Image.new("RGBA", composed.size, (0, 0, 0, 0))
    hatch_draw = ImageDraw.Draw(hatch)
    for x in range(-composed.height, composed.width, 28):
        hatch_draw.line((x, composed.height, x + composed.height, 0), fill=(150, 18, 25, 95), width=2)
    hatch_alpha = Image.new("L", composed.size, 0)
    hp = hatch_alpha.load()
    mp = mask_scaled.load()
    for y in range(mask_scaled.height):
        for x in range(mask_scaled.width):
            if mp[x, y] < 128:
                hp[x, y] = 95
    hatch.putalpha(hatch_alpha)
    composed = Image.alpha_composite(composed, hatch)

    margin_x = 90
    margin_top = 80
    margin_bottom = 165
    canvas_w = composed.width + 2 * margin_x
    canvas_h = composed.height + margin_top + margin_bottom
    canvas = Image.new("RGB", (canvas_w, canvas_h), (248, 249, 250))

    map_x = margin_x
    map_y = margin_top
    shadow = Image.new("RGBA", (composed.width + 16, composed.height + 16), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    sd.rectangle((8, 8, composed.width + 8, composed.height + 8), fill=(0, 0, 0, 24))
    canvas.paste(shadow.convert("RGB"), (map_x - 8, map_y - 8))
    canvas.paste(composed.convert("RGB"), (map_x, map_y))

    draw = ImageDraw.Draw(canvas)
    draw.rectangle((map_x, map_y, map_x + composed.width - 1, map_y + composed.height - 1), outline=(80, 80, 80), width=2)

    font = load_font(26)
    draw_legend(draw, margin_x, map_y + composed.height + 52, font)
    draw_scale_bar(draw, canvas_w - margin_x - 300, map_y + composed.height + 70, scale, font)

    canvas.save(OUT_PNG, quality=95)
    canvas.save(OUT_PDF, resolution=300.0)
    print(OUT_PNG)
    print(OUT_PDF)


if __name__ == "__main__":
    main()
