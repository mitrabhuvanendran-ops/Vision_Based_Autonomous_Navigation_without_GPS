#!/usr/bin/env python3
"""Stage 4.2 - Fine-tune BiSeNetV2 from the pretrained 4-class RUGD weights
down to 2 classes (traversable / non-traversable) on the sim dataset
produced by bisenet_dataset_collector.py.

Usage:
    python3 train_bisenet.py \
        --dataset ~/ugv_ws/bisenet_dataset \
        --pretrained ~/OFFSEG-main/Models/BiSeNet_RUGD/model_final.pth \
        --out ~/ugv_ws/models/bisenet_sim_2class.pth \
        --freeze-epochs 5 --total-epochs 40

The original 4-class weights are never overwritten - the fine-tuned model
is always saved to a separate --out path.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

sys.path.insert(0, os.path.expanduser('~/OFFSEG-main/Segmentation/BiSeNet'))
from lib.models.bisenetv2 import BiSeNetV2  # noqa: E402

N_CLASSES = 2
VOID_LABEL = 255
IGNORE_INDEX = 255


class SimSegDataset(Dataset):
    def __init__(self, root, size=(512, 512)):
        self.img_paths = sorted(glob.glob(os.path.join(root, 'images', '*.png')))
        self.mask_dir = os.path.join(root, 'masks')
        self.size = size
        if not self.img_paths:
            raise RuntimeError(f"No frames found under {root}/images - run "
                                "bisenet_dataset_collector.py first.")

    def __len__(self):
        return len(self.img_paths)

    def __getitem__(self, idx):
        img_path = self.img_paths[idx]
        mask_path = os.path.join(self.mask_dir, os.path.basename(img_path))

        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

        img = cv2.resize(img, self.size, interpolation=cv2.INTER_LINEAR)
        mask = cv2.resize(mask, self.size, interpolation=cv2.INTER_NEAREST)

        img = img.astype(np.float32) / 255.0
        img = (img - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        img = torch.from_numpy(img.transpose(2, 0, 1)).float()

        mask = mask.astype(np.int64)
        mask[mask == VOID_LABEL] = IGNORE_INDEX
        mask = torch.from_numpy(mask)
        return img, mask


def load_pretrained_backbone(model, pretrained_path):
    """Load the 4-class RUGD checkpoint, keeping only layers whose shapes
    still match (i.e. the shared backbone) and skipping the old 4-class
    head/aux layers, which get freshly-initialized 2-class weights instead."""
    ckpt = torch.load(pretrained_path, map_location='cpu')
    state_dict = ckpt.get('state_dict', ckpt) if isinstance(ckpt, dict) else ckpt

    model_dict = model.state_dict()
    matched, skipped = {}, []
    for k, v in state_dict.items():
        key = k.replace('module.', '')
        if key in model_dict and model_dict[key].shape == v.shape:
            matched[key] = v
        else:
            skipped.append(k)

    model_dict.update(matched)
    model.load_state_dict(model_dict)
    print(f"Loaded {len(matched)} backbone tensors from {pretrained_path}; "
          f"skipped {len(skipped)} (old 4-class head/aux, reinitialized for 2 classes)")


def set_backbone_trainable(model, trainable):
    for module in (model.detail, model.segment, model.bga):
        for p in module.parameters():
            p.requires_grad = trainable


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', default=os.path.expanduser('~/ugv_ws/bisenet_dataset'))
    ap.add_argument('--pretrained', default=os.path.expanduser(
        '~/OFFSEG-main/Models/BiSeNet_RUGD/model_final.pth'))
    ap.add_argument('--out', default=os.path.expanduser(
        '~/ugv_ws/models/bisenet_sim_2class.pth'))
    ap.add_argument('--freeze-epochs', type=int, default=5,
                     help='epochs with the backbone frozen, head-only training')
    ap.add_argument('--total-epochs', type=int, default=40)
    ap.add_argument('--batch-size', type=int, default=8)
    ap.add_argument('--lr-head', type=float, default=1e-3)
    ap.add_argument('--lr-finetune', type=float, default=1e-4)
    ap.add_argument('--val-split', type=float, default=0.1)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    full_ds = SimSegDataset(args.dataset)
    n_val = max(1, int(len(full_ds) * args.val_split))
    n_train = len(full_ds) - n_val
    train_ds, val_ds = torch.utils.data.random_split(full_ds, [n_train, n_val])
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=2)

    model = BiSeNetV2(n_classes=N_CLASSES, output_aux=True).to(device)
    load_pretrained_backbone(model, args.pretrained)

    criterion = nn.CrossEntropyLoss(ignore_index=IGNORE_INDEX)

    set_backbone_trainable(model, False)
    optimizer = torch.optim.Adam(
        [p for p in model.parameters() if p.requires_grad], lr=args.lr_head)

    best_val_loss = float('inf')
    for epoch in range(args.total_epochs):
        if epoch == args.freeze_epochs:
            print(f"Epoch {epoch}: unfreezing backbone, switching to lr={args.lr_finetune}")
            set_backbone_trainable(model, True)
            optimizer = torch.optim.Adam(model.parameters(), lr=args.lr_finetune)

        model.train()
        train_loss = 0.0
        for imgs, masks in train_loader:
            imgs, masks = imgs.to(device), masks.to(device)
            logits, aux2, aux3, aux4, aux5_4 = model(imgs)
            loss = (criterion(logits, masks) + 0.4 * (
                criterion(aux2, masks) + criterion(aux3, masks) +
                criterion(aux4, masks) + criterion(aux5_4, masks)))
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item()
        train_loss /= max(1, len(train_loader))

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for imgs, masks in val_loader:
                imgs, masks = imgs.to(device), masks.to(device)
                logits, *_ = model(imgs)
                val_loss += criterion(logits, masks).item()
        val_loss /= max(1, len(val_loader))

        print(f"epoch {epoch:03d} train_loss={train_loss:.4f} val_loss={val_loss:.4f} "
              f"backbone_frozen={epoch < args.freeze_epochs}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({'state_dict': model.state_dict(), 'n_classes': N_CLASSES},
                       args.out)
            print(f"  -> saved new best checkpoint to {args.out}")

    print(f"Done. Best val_loss={best_val_loss:.4f}. Fine-tuned weights at {args.out} "
          f"(original {args.pretrained} untouched).")


if __name__ == '__main__':
    main()
