"""Train and evaluate a three-way classifier on single printed-sign crops."""
import csv
import hashlib
import json
import math
import os
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter, ImageOps
import torch
from torch import nn

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
REPO = ROOT / "third_party/yolov5"
sys.path.insert(0, str(REPO))
os.environ["YOLOv5_AUTOINSTALL"] = "false"
from models.yolo import ClassificationModel, DetectionModel

NAMES = ["hazardous", "person", "interference"]
SIZE = 128
CODES = {f"1-{n:02d}" for n in [2, 3, 4, 5, 6, 24, 25, 26, 27]}
SIGNS = ROOT / "国标标识打印包"
RNG = random.Random(2101)


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rgb(path):
    with Image.open(path) as image:
        return image.convert("RGB")


def sample(image, label, group, ident):
    return (image, label, group, ident)


def load_data():
    manifest = read(SIGNS / "图样清单.json")
    pdf = next((SIGNS / "来源").glob("GB2894*.pdf"))
    assert digest(pdf) == manifest["source_sha256"]
    negative, positive, real, test = [], [], [], []
    extra = read(next(SIGNS.glob("05*/扩展图样清单.json")))
    negative_codes = sorted({r["code"] for r in manifest["items"] + extra} - CODES)
    heldout = set(random.Random(45).sample(negative_codes, 18))
    for r in manifest["items"] + extra:
        path = SIGNS / r["file"]
        assert digest(path) == r["sha256"], path
        code = r["code"]
        label = 0 if code in CODES else 2
        row = sample(rgb(path), label, code, path.name)
        (positive if label == 0 else test if code in heldout else negative).append(row)
    assert {r[2] for r in positive} == CODES and len(positive) == 9
    crosswalk = next(SIGNS.glob("08*/01_注意行人_高清国标.png"))
    negative.append(sample(rgb(crosswalk), 2, "crosswalk", crosswalk.name))
    avatar = next(SIGNS.glob("07*/person_position_1_native.png"))
    person_pdf = sample(rgb(avatar), 1, "person_pdf", avatar.name)
    ds = ROOT / "real_capture/datasets/targets_2class_20261116_v4"
    split = ROOT / "real_capture/training/targets_v4_person_0p5m_yolov5s"
    train_ids, test_ids = [
        {Path(p).stem for p in (split / f"{s}.txt").read_text(encoding="utf-8-sig").splitlines()}
        for s in ("train", "val")
    ]
    assert not train_ids.intersection(test_ids)
    for row in read(ds / "manifest.json"):
        stem = row["stem"]
        for obj in row["objects"]:
            person = obj["glyph"] == "person"
            if stem not in test_ids and not (person and stem in train_ids):
                continue
            path = ds / "images" / f"{stem}.jpg"
            img = rgb(path).crop(tuple(obj["box_xyxy"]))
            record = sample(img, int(person), obj["glyph"], stem)
            (test if stem in test_ids else real).append(record)
    assert len(real) == 107 and sum(r[2] == "person" for r in test) == 26
    return positive, negative, person_pdf, real, test


def render(row, augment=False):
    image = row[0]
    scale = RNG.uniform(0.72, 0.99) if augment else 0.90
    thumb = ImageOps.contain(image, (round(SIZE*scale), round(SIZE*scale)), Image.Resampling.LANCZOS)
    dx, dy = SIZE-thumb.width, SIZE-thumb.height
    shade = RNG.randint(148, 255) if augment else 240
    canvas = Image.new("RGB", (SIZE, SIZE), (shade,)*3)
    canvas.paste(thumb, (RNG.randint(0, dx), RNG.randint(0, dy)) if augment else (dx//2, dy//2))
    if augment:
        canvas = ImageEnhance.Brightness(canvas).enhance(RNG.uniform(0.50, 1.15))
        canvas = ImageEnhance.Color(canvas).enhance(RNG.uniform(0.42, 1.22))
        canvas = ImageEnhance.Contrast(canvas).enhance(RNG.uniform(0.64, 1.20))
        if RNG.random() < 0.45:
            canvas = canvas.filter(ImageFilter.GaussianBlur(RNG.uniform(0.3, 1.5)))
    return np.asarray(canvas)


def batch(rows, augment=False):
    arr = np.stack([render(row, augment) for row in rows])
    return torch.from_numpy(arr).to("cuda").permute(0, 3, 1, 2).float().div_(255)


def repeated(pool, count):
    return [pool[i % len(pool)] for i in range(count)]


def main():
    torch.manual_seed(2101)
    torch.set_num_threads(3)
    positive, negative, pdf, real, test = load_data()
    torch.cuda.manual_seed_all(2101)
    backbone = DetectionModel(str(REPO / "models/yolov5n.yaml"))
    model = ClassificationModel(model=backbone, nc=3, cutoff=10).cuda()
    opt = torch.optim.AdamW(model.parameters(), lr=0.0015, weight_decay=0.001)
    epochs = 42
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=0.00005)
    for epoch in range(epochs):
        model.train()
        RNG.shuffle(positive)
        RNG.shuffle(negative)
        rows = repeated(positive, 214) + repeated(negative, 214) + [pdf]*107 + real
        RNG.shuffle(rows)
        hits = 0
        for i in range(0, len(rows), 48):
            chunk = rows[i:i+48]
            targets = torch.tensor([r[1] for r in chunk], device="cuda")
            opt.zero_grad(set_to_none=True)
            scores = model(batch(chunk, True))
            loss = nn.functional.cross_entropy(scores, targets, label_smoothing=0.02)
            loss.backward()
            opt.step()
            hits += int((scores.argmax(1) == targets).sum())
        schedule.step()
        print(f"epoch {epoch+1}/{epochs}, train {hits}/{len(rows)}", flush=True)
    model = model.eval().cpu()
    example = torch.zeros(1,3,SIZE,SIZE)
    with torch.inference_mode():
        artifact = torch.jit.trace(model, example)
        with (OUT / "YOLO5-Z1.torchscript.pt").open("wb") as handle:
            torch.jit.save(artifact, handle)
    with (OUT / "YOLO5-Z1.torchscript.pt").open("rb") as handle:
        predictor = torch.jit.load(handle).eval().cuda()
    attachments = ["4f46944c-266e-4c08-a8d7-ac6126fca650", "f4a25770-c0ad-4ae0-9049-7ffcbaab3f95", "64d43025-bc27-4b69-ba41-1eea5ecdefd5", "1a117c0e-35fc-4ea4-8ba2-d7cd5b79006a", "13b09733-7aad-45bc-aca5-5ffd9bbbd274", "0a3459db-3a86-4274-9d15-0073eefe70eb"]
    for uid in attachments:
        path = Path(os.environ["TEMP"]) / f"codex-clipboard-{uid}.png"
        if path.exists():
            test.append(sample(rgb(path), 0, "attachment", uid))
    stats = defaultdict(Counter)
    records, examples = [], defaultdict(list)
    with torch.inference_mode():
        for i in range(0, len(test), 32):
            chunk = test[i:i+32]
            for row, prob in zip(chunk, predictor(batch(chunk)).softmax(1).tolist()):
                _, label, group, ident = row
                pred = int(np.argmax(prob))
                stats[group]["correct"] += int(pred == label)
                stats[group]["total"] += 1
                records.append([ident,group,NAMES[label],NAMES[pred],round(prob[pred],5)])
                examples[group].append((row, pred, prob[pred]))
    with (OUT/"predictions.csv").open("w",encoding="utf-8-sig",newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id","group","actual","predicted","score"])
        writer.writerows(records)
    chosen = []
    for key in ["attachment","fire","explosion","toxic","person"]:
        vals = sorted(examples[key],key=lambda v:(v[1] == v[0][1],v[2]))
        chosen.extend(vals[:6 if key == "attachment" else 3])
    negs = [v for group in examples.values() for v in group if v[0][1] == 2]
    chosen.extend(sorted(negs,key=lambda v:(v[1] == 2,v[2]))[:6])
    sheet = Image.new("RGB",(4*185,math.ceil(len(chosen)/4)*174),"white")
    draw = ImageDraw.Draw(sheet)
    for k,(row,pred,score) in enumerate(chosen):
        x,y=k%4*185,k//4*174
        sheet.paste(Image.fromarray(render(row)),(x+24,y))
        draw.text((x+4,y+128),f"True: {row[2]}",fill="black")
        draw.text((x+4,y+144),f"{NAMES[pred]}  {score:.2f}",fill="green" if pred == row[1] else "red")
    sheet.save(OUT/"preview.jpg",quality=92)
    summary = dict(architecture="YOLOv5n-cls",initialization="random",seed=2101,epochs=epochs,
        classes=NAMES,input="RGB, letterbox to 90% of 128x128 on RGB(240,240,240), NCHW float32 /255",
        output="three logits in classes order",rotation=0,train_samples_per_epoch=[214,214,214],
        unique_hazard_pdf=len(positive), unique_interference_train=len(negative),person_per_epoch={"pdf":107,"real":107},
        evaluation=stats, artifact_sha256=digest(OUT/"YOLO5-Z1.torchscript.pt"))
    (OUT/"result.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    print("Evaluation complete", flush=True)


if __name__ == "__main__":
    main()
