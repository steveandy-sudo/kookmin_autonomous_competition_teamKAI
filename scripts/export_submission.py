#!/usr/bin/env python3
"""제출 전 필수 파일 존재 여부를 확인하는 스크립트."""

from __future__ import annotations

from pathlib import Path


REQUIRED_FILES = [
    'track_drive/track_drive.py',
    'dev_plan.pdf',
    'result.mp4',
]


# 설명: ROS 노드나 스크립트 실행을 시작하는 진입점이다.
def main() -> int:
    """필수 파일 체크리스트를 출력한다."""
    root = Path(__file__).resolve().parent.parent

    print('[Submission Checklist]')
    missing = []
    for rel_path in REQUIRED_FILES:
        target = root / rel_path
        exists = target.exists()
        mark = 'OK' if exists else 'MISSING'
        print(f'- [{mark}] {rel_path}')
        if not exists:
            missing.append(rel_path)

    if missing:
        print('\n누락 파일이 있습니다. 최종 패키징 전에 반드시 채워주세요.')
        return 1

    print('\n필수 파일 확인 완료. (zip 생성 기능은 추후 추가 예정)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
