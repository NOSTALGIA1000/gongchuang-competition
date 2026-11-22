"""Review real candidate sign crops, then score them with the frozen classifier."""
import sys
from collections import Counter
from pathlib import Path

from train import OUT, ROOT, NAMES, Image, ImageOps, ImageDraw, batch, csv, read, rgb, sample, torch

ds = ROOT / "real_capture/datasets/targets_2class_20261116_v4"
val = ROOT / "real_capture/training/targets_v4_person_0p5m_yolov5s/val.txt"
stems = {Path(line).stem for line in val.read_text(encoding="utf-8-sig").splitlines()}
crops = []
for row in read(ds / "manifest.json"):
    if row["stem"] in stems and not row["objects"]:
        for cand in row["review_candidates"]:
            image = rgb(ds / "images" / (row["stem"] + ".jpg"))
            crop = image.crop(tuple(cand["box_xyxy"]))
            crops.append(sample(crop, 2, row["capture_group"], row["stem"]))

if "--score" in sys.argv:
    with (OUT / "YOLO5-Z1.torchscript.pt").open("rb") as handle:
        model = torch.jit.load(handle).eval().cuda()
    results = []
    with torch.inference_mode():
        for start in range(0, len(crops), 32):
            rows = crops[start:start+32]
            scores = model(batch(rows)).softmax(1)
            for row, vec in zip(rows, scores.tolist()):
                idx = max(range(3), key=vec.__getitem__)
                results.append([row[3],row[2],NAMES[idx],round(vec[idx],5)])
    with (OUT/"real_interference.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id","capture_group","prediction","score"])
        writer.writerows(results)
    print(Counter(r[2] for r in results))
else:
    sheet = Image.new("RGB", (800, ((len(crops)+9)//10)*78), "white")
    draw = ImageDraw.Draw(sheet)
    for i,row in enumerate(crops):
        x,y = i%10*80,i//10*78
        sheet.paste(ImageOps.contain(row[0],(78,61)),(x,y))
        draw.text((x,y+61), row[3][-4:], fill="black")
    sheet.save(OUT/"real_interference_review.jpg")
