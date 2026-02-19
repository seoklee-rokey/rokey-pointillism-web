from __future__ import annotations

from pathlib import Path
import random
from PIL import Image, ImageOps, ImageDraw


def generate_stipple(
    input_path: str | Path,
    output_path: str | Path,
    density: int,
    *,
    max_size: int = 1000,
    seed: int | None = None,
    color: bool = True,          # ✅ 추가: True면 컬러 점, False면 검정 점
    vary_radius: bool = True,    # ✅ 추가: 밝기에 따라 점 크기 변화
) -> None:
    """
    실험용 점묘화 생성기.
    - input_path: 원본 이미지 경로
    - output_path: 결과 이미지 경로 (보통 processed.png)
    - density: 0~100 (높을수록 점이 많아짐)
    - max_size: 긴 변 리사이즈(처리 속도용)
    - seed: 동일 결과 재현용(선택)
    - color: True면 원본 픽셀 색으로 점을 찍음(컬러 점묘), False면 검정 점(기존)
    - vary_radius: True면 밝기(어두움) 기반으로 점 크기를 조금 바꿈
    """

    input_path = Path(input_path)
    output_path = Path(output_path)

    if seed is not None:
        random.seed(seed)

    if not input_path.exists():
        raise FileNotFoundError(f"Input not found: {input_path}")

    img = Image.open(input_path).convert("RGB")

    # ✅ 너무 큰 이미지면 처리 속도 때문에 리사이즈(긴 변 max_size)
    w, h = img.size
    long_side = max(w, h)
    if long_side > max_size:
        scale = max_size / long_side
        img = img.resize((int(w * scale), int(h * scale)))
        w, h = img.size

    # ✅ 그레이스케일(밝기 기반으로 점 찍기)
    gray = ImageOps.grayscale(img)  # 0(검)~255(흰)

    # density 보정
    density = int(density)
    density = max(0, min(100, density))

    # ✅ 점 개수(기존 로직 유지): density가 커질수록 많이 찍음
    n_points = int((w * h) * (density / 1200.0))
    n_points = max(0, min(n_points, w * h))

    # ✅ 흰 바탕 캔버스
    out = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(out)

    # ✅ 기본 점 크기(기존 로직 유지)
    base_r = 1 if density < 40 else 2 if density < 80 else 3

    for _ in range(n_points):
        x = random.randrange(w)
        y = random.randrange(h)

        v = gray.getpixel((x, y))      # 0~255
        p = 1.0 - (v / 255.0)          # 어두울수록 1에 가까움

        # 확률적으로 점 찍기(어두운 영역에 더 많이)
        if random.random() < p:
            # ✅ 점 색: 컬러면 원본 픽셀 색, 아니면 검정(기존)
            if color:
                fill = img.getpixel((x, y))  # (R,G,B)
            else:
                fill = (0, 0, 0)

            # ✅ 점 크기: 밝기 기반으로 약간 가변(어두울수록 좀 더 크게)
            if vary_radius:
                # v=0(어두움) -> t=1, v=255(밝음) -> t=0
                t = 1.0 - (v / 255.0)
                # base_r 기준으로 0.7배 ~ 1.6배 정도
                r = max(1, int(round(base_r * (0.7 + 0.9 * t))))
            else:
                r = base_r

            draw.ellipse((x - r, y - r, x + r, y + r), fill=fill)

    # ✅ 저장 (부모 폴더 없으면 생성)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    out.save(output_path)
