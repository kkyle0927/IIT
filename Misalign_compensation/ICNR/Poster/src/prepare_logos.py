#!/usr/bin/env python3
"""Prepare poster-resolution logo PNGs into ICNR/Poster/assets/.

KAIST : official wordmark SVG (robotics.kaist.ac.kr /images/common/logo-uni-w.svg),
        recoloured from white to KAIST blue and rasterised at poster resolution.
EXO LAB: reconstructed from the only on-disk copy (131x69 px). The mark and the
        "EXO LAB" wordmark are upscaled and re-quantised to the two brand colours;
        the unreadable "EXOSKELETON LABORATORY" subline is cropped off and is
        redrawn as native PowerPoint text by build_pptx.py.
        If assets/logo_exolab_hires.png exists it is used verbatim instead.
"""
import shutil
from pathlib import Path

import cairosvg
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
EXOLAB_SRC = Path("/home/chanyoungko/IIT/Data_Acq/MD/MD GUI/Images/MD_GUI_FIGURE_EXOLAB_LOGO.png")

KAIST_BLUE = "#1F4899"
EXO_DARK = np.array([61, 63, 54])
EXO_GREEN = np.array([159, 190, 25])


def kaist_wordmark():
    svg = (ASSETS / "logo_kaist_wordmark_src.svg").read_text()
    blue = svg.replace('fill="white"', f'fill="{KAIST_BLUE}"')
    out = ASSETS / "logo_kaist.png"
    cairosvg.svg2png(bytestring=blue.encode(), write_to=str(out), output_width=2400)
    print(f"{out.name}: {Image.open(out).size}")


def exolab():
    override = ASSETS / "logo_exolab_hires.png"
    out = ASSETS / "logo_exolab.png"
    if override.exists():
        shutil.copy(override, out)
        print(f"{out.name}: {Image.open(out).size} (from user-supplied hi-res)")
        return

    src = Image.open(EXOLAB_SRC).convert("RGBA")
    # drop the subline band (source rows 65-68): 4 px tall, unrecoverable when upscaled
    src = src.crop((0, 0, src.width, 56))

    scale = 16
    big = np.array(src.resize((src.width * scale, src.height * scale), Image.LANCZOS)).astype(float)
    alpha = np.clip((big[..., 3] - 128) * 20 + 128, 0, 255)
    rgb = big[..., :3]
    is_green = np.linalg.norm(rgb - EXO_GREEN, axis=-1) < np.linalg.norm(rgb - EXO_DARK, axis=-1)
    flat = np.where(is_green[..., None], EXO_GREEN, EXO_DARK).astype(float)
    res = np.dstack([flat, alpha]).astype(np.uint8)

    im = Image.fromarray(res)
    im = im.crop(im.getchannel("A").point(lambda v: 255 if v > 8 else 0).getbbox())
    pad = Image.new("RGBA", (im.width + 2 * scale, im.height + 2 * scale), (0, 0, 0, 0))
    pad.paste(im, (scale, scale))
    pad.save(out)
    im = pad
    print(f"{out.name}: {im.size} (reconstructed from {EXOLAB_SRC.name})")


if __name__ == "__main__":
    kaist_wordmark()
    exolab()
