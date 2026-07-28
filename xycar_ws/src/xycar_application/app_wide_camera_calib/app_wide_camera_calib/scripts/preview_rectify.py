#!/usr/bin/env python3

import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def load_camera_yaml(path):
    with open(path, 'r') as f:
        y = yaml.safe_load(f)

    K = np.array(y['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(y['distortion_coefficients']['data'], dtype=np.float64).reshape(4, 1)
    w = int(y['image_width'])
    h = int(y['image_height'])
    return K, D, (w, h)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', required=True)
    parser.add_argument('--image_dir', required=True)
    parser.add_argument('--output_dir', required=True)
    parser.add_argument('--balance', type=float, default=0.3)
    args = parser.parse_args()

    K, D, image_size = load_camera_yaml(args.yaml)
    w, h = image_size

    newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K,
        D,
        image_size,
        np.eye(3),
        balance=args.balance,
        new_size=image_size,
        fov_scale=1.0
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K,
        D,
        np.eye(3),
        newK,
        image_size,
        cv2.CV_16SC2
    )

    os.makedirs(args.output_dir, exist_ok=True)

    image_paths = sorted(glob.glob(os.path.join(args.image_dir, '*.png')))[:10]

    if not image_paths:
        raise RuntimeError(f'No PNG images found in {args.image_dir}')

    for i, path in enumerate(image_paths):
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            continue

        rect = cv2.remap(
            img,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        pair = np.hstack([
            cv2.resize(img, (w // 2, h // 2)),
            cv2.resize(rect, (w // 2, h // 2))
        ])

        out = os.path.join(args.output_dir, f'rectify_preview_{i:02d}.png')
        cv2.imwrite(out, pair)
        print(f'Saved: {out}')

    print('\n왼쪽: 원본 fisheye, 오른쪽: undistorted/rectified')
    print(f'balance={args.balance}')
    print('balance를 키우면 화각은 넓어지고, 검은 영역/왜곡 잔류가 커질 수 있음.')
    print('balance를 줄이면 crop은 커지지만 직선성이 좋아짐.')


if __name__ == '__main__':
    main()
