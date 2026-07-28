#!/usr/bin/env python3

import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def detect_corners(gray, pattern_size):
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH |
        cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)

    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        1e-3
    )

    corners = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=criteria
    )

    return True, corners


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_dir', required=True)
    parser.add_argument('--output_yaml', required=True)
    parser.add_argument('--board_cols', type=int, default=8)
    parser.add_argument('--board_rows', type=int, default=9)
    parser.add_argument('--square_size', type=float, default=0.07)
    parser.add_argument('--camera_name', default='wide_camera')
    args = parser.parse_args()

    pattern_size = (args.board_cols, args.board_rows)

    image_paths = sorted(glob.glob(os.path.join(args.image_dir, '*.png')))
    if not image_paths:
        raise RuntimeError(f'No PNG images found in {args.image_dir}')

    objp = np.zeros((1, args.board_cols * args.board_rows, 3), np.float64)
    grid = np.mgrid[0:args.board_cols, 0:args.board_rows].T.reshape(-1, 2)
    objp[0, :, :2] = grid * args.square_size

    objpoints = []
    imgpoints = []

    image_size = None
    used = 0
    failed = 0

    for path in image_paths:
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            failed += 1
            print(f'[FAIL] read: {path}')
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
        elif image_size != gray.shape[::-1]:
            raise RuntimeError(f'Image size mismatch: {path}, {gray.shape[::-1]} != {image_size}')

        found, corners = detect_corners(gray, pattern_size)

        if found:
            objpoints.append(objp.copy())
            imgpoints.append(corners.astype(np.float64))
            used += 1
            print(f'[OK] {path}')
        else:
            failed += 1
            print(f'[FAIL] corners: {path}')

    if used < 15:
        raise RuntimeError(f'Not enough valid images: {used}. Capture at least 30 valid images.')

    width, height = image_size

    K = np.zeros((3, 3), dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)

    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(used)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(used)]

    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
        cv2.fisheye.CALIB_FIX_SKEW
    )

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6
    )

    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        image_size,
        K,
        D,
        rvecs,
        tvecs,
        flags,
        criteria
    )

    R = np.eye(3, dtype=np.float64)
    P = np.zeros((3, 4), dtype=np.float64)
    P[:3, :3] = K

    data = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': args.camera_name,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': K.reshape(-1).tolist()
        },
        'distortion_model': 'equidistant',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 4,
            'data': D.reshape(-1).tolist()
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': R.reshape(-1).tolist()
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': P.reshape(-1).tolist()
        },
        'calibration_info': {
            'model': 'opencv_fisheye',
            'board_inner_corners': [args.board_cols, args.board_rows],
            'square_size_m': args.square_size,
            'used_images': used,
            'failed_images': failed,
            'rms_reprojection_error_px': float(rms)
        }
    }

    os.makedirs(os.path.dirname(args.output_yaml), exist_ok=True)

    with open(args.output_yaml, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print('\n========== FISHEYE CALIBRATION RESULT ==========')
    print(f'image_size: {width} x {height}')
    print(f'used images: {used}')
    print(f'failed images: {failed}')
    print(f'RMS reprojection error: {rms:.4f} px')
    print('K:')
    print(K)
    print('D:')
    print(D.reshape(-1))
    print(f'\nSaved: {args.output_yaml}')

    if rms < 1.0:
        print('판정: 좋음')
    elif rms < 2.0:
        print('판정: 사용 가능. 단, extrinsic 전에 overlay 검증 필요')
    else:
        print('판정: 재촬영 권장. 가장자리/기울어진 보드 이미지 추가 필요')


if __name__ == '__main__':
    main()
