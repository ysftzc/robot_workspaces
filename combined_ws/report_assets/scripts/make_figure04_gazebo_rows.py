#!/usr/bin/env python3
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


ROOT = Path("/home/yusuf/robot_workspaces/combined_ws")
SRC = Path("/home/yusuf/Pictures/Screenshots/Screenshot from 2026-05-13 20-48-17.png")
OUT_DIR = ROOT / "report_assets" / "figures"
DL_DIR = Path("/home/yusuf/Downloads")

PNG_OUT = OUT_DIR / "figure04_gazebo_greenhouse_b_c_rows.png"
PDF_OUT = OUT_DIR / "figure04_gazebo_greenhouse_b_c_rows.pdf"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def rounded_label(draw, xy, text, fill, outline, text_fill="white", pad=(18, 10), size=44):
    fnt = font(size, bold=True)
    x, y = xy
    bbox = draw.textbbox((0, 0), text, font=fnt)
    w = bbox[2] - bbox[0] + pad[0] * 2
    h = bbox[3] - bbox[1] + pad[1] * 2
    draw.rounded_rectangle((x, y, x + w, y + h), radius=18, fill=fill, outline=outline, width=4)
    draw.text((x + pad[0], y + pad[1] - 2), text, font=fnt, fill=text_fill)
    return (x, y, x + w, y + h)


def arrow(draw, start, end, color, width=10):
    draw.line([start, end], fill=color, width=width)
    ex, ey = end
    sx, sy = start
    dx, dy = ex - sx, ey - sy
    length = (dx * dx + dy * dy) ** 0.5 or 1
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    head = 34
    wing = 18
    points = [
        (ex, ey),
        (ex - ux * head + px * wing, ey - uy * head + py * wing),
        (ex - ux * head - px * wing, ey - uy * head - py * wing),
    ]
    draw.polygon(points, fill=color)


def main():
    src = Image.open(SRC).convert("RGB")

    # Crop the Gazebo 3D viewport from the selected screenshot, excluding entity tree and terminals.
    crop = src.crop((70, 58, 854, 1008))

    canvas_w, canvas_h = 1800, 1200
    canvas = Image.new("RGB", (canvas_w, canvas_h), "white")

    # Fill the report canvas while preserving the Gazebo perspective.
    scale = max(canvas_w / crop.width, canvas_h / crop.height)
    resized = crop.resize((int(crop.width * scale), int(crop.height * scale)), Image.Resampling.LANCZOS)
    left = (canvas_w - resized.width) // 2
    top = (canvas_h - resized.height) // 2
    canvas.paste(resized, (left, top))

    # Gentle dark overlay improves label contrast without hiding the simulation scene.
    overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    od.rectangle((0, 0, canvas_w, canvas_h), fill=(0, 0, 0, 34))
    canvas = Image.alpha_composite(canvas.convert("RGBA"), overlay)
    draw = ImageDraw.Draw(canvas)

    blue = "#2563EB"
    violet = "#7C3AED"
    slate = "#0F172A"

    # Row annotations. The crop shows the B row on the left side of the aisle and C row on the right.
    rounded_label(draw, (92, 128), "B Row", blue, "#DBEAFE", size=48)
    arrow(draw, (245, 183), (535, 360), blue, width=12)
    draw.text((92, 205), "plants B0 - B10", font=font(26, bold=True), fill="white")

    rounded_label(draw, (1210, 120), "C Row", violet, "#EDE9FE", size=48)
    arrow(draw, (1340, 176), (1145, 422), violet, width=12)
    draw.text((1210, 198), "plants C0 - C10", font=font(26, bold=True), fill="white")

    # Central aisle label.
    draw.rounded_rectangle((650, 968, 1150, 1042), radius=20, fill=(255, 255, 255, 230), outline="#CBD5E1", width=3)
    draw.text((690, 988), "central navigation aisle", font=font(34, bold=True), fill=slate)

    # Top-down inset created from world coordinates.
    inset = (60, 820, 540, 1135)
    draw.rounded_rectangle(inset, radius=24, fill=(255, 255, 255, 236), outline="#CBD5E1", width=3)
    draw.text((92, 850), "World layout inset", font=font(30, bold=True), fill=slate)

    ix0, iy0, ix1, iy1 = inset
    plot = (118, 905, 496, 1090)
    draw.rounded_rectangle(plot, radius=16, fill="#F8FAFC", outline="#E2E8F0", width=2)

    # Coordinate mapping: x is horizontal, y is row length. B row x=36.75, C row x=38.25.
    b_x = 220
    c_x = 390
    y_top = 940
    y_bottom = 1060
    draw.line((b_x, y_top, b_x, y_bottom), fill=blue, width=12)
    draw.line((c_x, y_top, c_x, y_bottom), fill=violet, width=12)
    draw.line((b_x + 48, y_top, b_x + 48, y_bottom), fill="#A8A29E", width=26)
    draw.text((b_x - 56, y_top - 35), "B row", font=font(24, bold=True), fill=blue)
    draw.text((c_x - 56, y_top - 35), "C row", font=font(24, bold=True), fill=violet)
    draw.text((b_x + 18, y_bottom + 15), "x=36.75", font=font(19), fill="#475569")
    draw.text((c_x - 15, y_bottom + 15), "x=38.25", font=font(19), fill="#475569")
    draw.text((128, 1097), "B/C plants are indexed along y: 0, 2, 4, 6, 8, 10", font=font(19), fill="#475569")

    for y, lab in [(y_top, "0"), (y_bottom, "10")]:
        draw.ellipse((b_x - 13, y - 13, b_x + 13, y + 13), fill="white", outline=blue, width=4)
        draw.ellipse((c_x - 13, y - 13, c_x + 13, y + 13), fill="white", outline=violet, width=4)
        draw.text((145, y - 12), lab, font=font(20, bold=True), fill="#475569")

    # Export high-quality PNG and PDF.
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    canvas_rgb = canvas.convert("RGB")
    canvas_rgb.save(PNG_OUT, quality=95)
    canvas_rgb.save(PDF_OUT, resolution=300.0)

    # Convenience copies for direct insertion.
    canvas_rgb.save(DL_DIR / "figure04_gazebo_greenhouse_b_c_rows.png", quality=95)
    canvas_rgb.save(DL_DIR / "figure04_gazebo_greenhouse_b_c_rows.pdf", resolution=300.0)


if __name__ == "__main__":
    main()
