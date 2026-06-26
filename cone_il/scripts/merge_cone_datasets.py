#!/usr/bin/env python3
import argparse
import csv
import os
import shutil
from pathlib import Path


FIELDNAMES = [
    'index', 'stamp', 'image_path', 'scan_path',
    'angle', 'speed', 'roi_top_ratio', 'resize_width', 'resize_height',
]


# 설명: 여러 학습 데이터셋의 이미지, 스캔, 라벨 CSV를 하나의 데이터셋으로 합친다.
def merge_datasets(sources, output_dir: Path):
    if output_dir.exists():
        raise RuntimeError(f'output directory already exists: {output_dir}')

    image_dir = output_dir / 'images'
    scan_dir = output_dir / 'scans'
    image_dir.mkdir(parents=True)
    scan_dir.mkdir(parents=True)

    out_rows = []
    skipped = []
    out_idx = 0

    for source in sources:
        labels_path = source / 'labels.csv'
        if not labels_path.exists():
            skipped.append((str(source), 'missing labels.csv'))
            continue

        with open(labels_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                source_idx = row.get('index', '?')
                image_rel = row.get('image_path', '')
                if not image_rel:
                    skipped.append((str(source), f'row {source_idx}: missing image_path'))
                    continue

                image_src = source / image_rel
                if not image_src.exists():
                    skipped.append((str(source), f'row {source_idx}: missing image {image_rel}'))
                    continue

                image_name = f'{out_idx:06d}{image_src.suffix or ".jpg"}'
                image_dst = image_dir / image_name
                shutil.copy2(image_src, image_dst)

                scan_rel_out = ''
                scan_rel = row.get('scan_path', '')
                if scan_rel:
                    scan_src = source / scan_rel
                    if scan_src.exists():
                        scan_name = f'{out_idx:06d}{scan_src.suffix or ".npy"}'
                        scan_dst = scan_dir / scan_name
                        shutil.copy2(scan_src, scan_dst)
                        scan_rel_out = os.path.join('scans', scan_name)
                    else:
                        skipped.append((str(source), f'row {source_idx}: missing scan {scan_rel}'))

                out_rows.append({
                    'index': out_idx,
                    'stamp': row.get('stamp', ''),
                    'image_path': os.path.join('images', image_name),
                    'scan_path': scan_rel_out,
                    'angle': row.get('angle', '0'),
                    'speed': row.get('speed', '0'),
                    'roi_top_ratio': row.get('roi_top_ratio', ''),
                    'resize_width': row.get('resize_width', ''),
                    'resize_height': row.get('resize_height', ''),
                })
                out_idx += 1

    with open(output_dir / 'labels.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(out_rows)

    return len(out_rows), skipped


# 설명: ROS 노드나 스크립트 실행을 시작하는 진입점이다.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('sources', nargs='+')
    args = parser.parse_args()

    sources = [Path(p).expanduser() for p in args.sources]
    output_dir = Path(args.output_dir).expanduser()
    sample_count, skipped = merge_datasets(sources, output_dir)

    print(f'merged target={output_dir}')
    print(f'samples={sample_count}')
    print(f'warnings={len(skipped)}')
    for source, reason in skipped[:20]:
        print(f'skipped: {source} | {reason}')


if __name__ == '__main__':
    main()
