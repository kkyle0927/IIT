#!/usr/bin/env python3
"""Render ICNR_Poster.pptx to a PNG through headless LibreOffice.

This is the visual check loop: build_pptx.py -> render_preview.py -> look at it.
LibreOffice was extracted (no root) to ~/opt/libreoffice by hand; SOFFICE points
at that binary. Output goes to the scratch dir, not into the repo.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import fitz

SOFFICE = Path("/home/chanyoungko/opt/libreoffice/opt/libreoffice26.2/program/soffice")
PPTX = Path(__file__).resolve().parents[1] / "ICNR_Poster.pptx"


def render(out_png: Path, dpi: int = 72):
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [str(SOFFICE), "--headless", "--norestore",
             f"-env:UserInstallation=file://{tmp}/profile",
             "--convert-to", "pdf", "--outdir", tmp, str(PPTX)],
            check=True, capture_output=True, timeout=300)
        pdf = Path(tmp) / (PPTX.stem + ".pdf")
        page = fitz.open(pdf)[0]
        page.get_pixmap(dpi=dpi).save(out_png)
    print(f"{out_png}  ({out_png.stat().st_size // 1024} kB)")


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/poster_preview.png")
    render(out, dpi=int(sys.argv[2]) if len(sys.argv) > 2 else 72)
