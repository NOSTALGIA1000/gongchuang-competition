"""Usage: python predict.py sign-crop.jpg [another-sign.png ...]"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageOps
import torch


def main():
    parser = argparse.ArgumentParser(description="Classify one printed sign per input image.")
    parser.add_argument("images", type=Path, nargs="+")
    args = parser.parse_args()
    weights = Path(__file__).with_name("YOLO5-Z1.torchscript.pt")
    with weights.open("rb") as handle:
        model = torch.jit.load(handle, map_location="cpu").eval()
    names = ["hazardous", "person", "interference"]
    with torch.inference_mode():
        for path in args.images:
            with Image.open(path) as image:
                tile = ImageOps.contain(image.convert("RGB"), (115, 115), Image.Resampling.LANCZOS)
            canvas = Image.new("RGB", (128, 128), (240, 240, 240))
            canvas.paste(tile, ((128 - tile.width)//2, (128 - tile.height)//2))
            tensor = torch.from_numpy(np.array(canvas)).permute(2, 0, 1).float().div_(255)
            probs = model(tensor.unsqueeze(0)).softmax(1)[0]
            idx = int(probs.argmax())
            print(f"{path.name}: {names[idx]} {float(probs[idx]):.4f}")


if __name__ == "__main__":
    main()
