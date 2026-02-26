# preview_utils.py
from pathlib import Path
import cv2
import numpy as np

# draw_stipple 팔레트는 RGB 튜플(r,g,b) 기준
# + 로봇 작업영역 상수도 draw_stipple에 정의되어 있음
from draw_stipple import palette, X_LEFT, X_RIGHT, Y_TOP, Y_BOTTOM


def save_preview_from_points(points_list, img_w, img_h, out_path, radius=1):
    """
    (기존 방식) 이미지 픽셀 좌표(canvas: img_h x img_w)에 점을 찍어서 저장.
    points_list: [(x, y, color_index), ...]  # color_index는 1~N
    img_w, img_h: 점이 생성된 이미지 크기
    """
    out_path = str(out_path)
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)

    canvas = np.ones((img_h, img_w, 3), dtype=np.uint8) * 255

    for x, y, v in points_list:
        r, g, b = palette[v - 1]  # palette는 RGB
        cv2.circle(
            canvas,
            (int(x), int(y)),
            int(radius),
            (int(b), int(g), int(r)),  # OpenCV는 BGR
            -1,
            lineType=cv2.LINE_AA,
        )

    cv2.imwrite(out_path, canvas)
    return out_path


def _pad_to_aspect(canvas_bgr: np.ndarray, target_aspect: float) -> np.ndarray:
    """
    canvas_bgr를 내용 손상 없이(늘리지 않고) 흰 여백으로 padding해서 target_aspect(w/h)를 맞춘다.
    """
    h, w = canvas_bgr.shape[:2]
    cur_aspect = w / h

    if target_aspect <= 0:
        return canvas_bgr

    # 이미 같으면 그대로
    if abs(cur_aspect - target_aspect) < 1e-6:
        return canvas_bgr

    if cur_aspect > target_aspect:
        # 현재가 더 가로로 넓음 -> 높이를 늘려야 함
        new_w = w
        new_h = int(round(w / target_aspect))
    else:
        # 현재가 더 세로로 김 -> 너비를 늘려야 함
        new_h = h
        new_w = int(round(h * target_aspect))

    padded = np.ones((new_h, new_w, 3), dtype=np.uint8) * 255
    off_x = (new_w - w) // 2
    off_y = (new_h - h) // 2
    padded[off_y:off_y + h, off_x:off_x + w] = canvas_bgr
    return padded


def save_preview_robot_style(points_list, img_w, img_h, out_path, pad_to_original_aspect: bool = True):
    """
    ✅ 변경된 draw_stipple.show_stipple() 스타일과 동일하게 미리보기 저장 (draw_stipple 수정 X)

    - 로봇 작업영역(mm) 기준 캔버스 생성
    - 스케일/오프셋/좌우반전 적용 (convert_to_robot_coords와 같은 식)
    - mm_to_px=4, real_radius_mm=1.5
    - GaussianBlur(7,7)
    - (옵션) 최종 이미지를 원본(img_w:img_h) 비율로 padding해서 웹에서 원본/처리결과 비율이 같게 보이도록 함

    points_list: [(x, y, color_index), ...]  # x,y는 이미지 픽셀 좌표
    img_w,img_h: 점이 생성된 이미지 크기
    """
    out_path = str(out_path)
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)

    # draw_stipple.show_stipple 내부 상수와 동일하게 맞춤
    mm_to_px = 4
    real_radius_mm = 1.5
    radius_px = int(real_radius_mm * mm_to_px)

    work_w = X_RIGHT - X_LEFT
    work_h = Y_BOTTOM - Y_TOP

    # 로봇 작업영역(mm)을 px 캔버스로 생성
    canvas_w = int(work_w * mm_to_px)
    canvas_h = int(work_h * mm_to_px)
    canvas = np.ones((canvas_h, canvas_w, 3), dtype=np.uint8) * 255

    # 이미지와 작업영역 비율 비교해 scale 결정 (convert_to_robot_coords와 동일)
    img_ratio = img_w / img_h
    work_ratio = work_w / work_h

    if img_ratio > work_ratio:
        scale = work_w / img_w
    else:
        scale = work_h / img_h

    draw_w = img_w * scale
    draw_h = img_h * scale

    # show_stipple 쪽은 작업영역 캔버스를 0,0 기준으로 쓰므로 X_LEFT/Y_TOP은 제외하고 센터링만
    offset_x = (work_w - draw_w) / 2
    offset_y = (work_h - draw_h) / 2

    for x, y, color_index in points_list:
        # 좌우 반전 포함
        mm_x = offset_x + x * scale #(img_w - x) * scale
        mm_y = offset_y + y * scale

        px = int(mm_x * mm_to_px)
        py = int(mm_y * mm_to_px)

        # 범위 밖 방어
        if px < 0 or px >= canvas_w or py < 0 or py >= canvas_h:
            continue

        r, g, b = palette[color_index - 1]  # RGB
        color_bgr = (int(b), int(g), int(r))
        cv2.circle(canvas, (px, py), radius_px, color_bgr, -1)

    # draw_stipple.show_stipple과 동일한 블러
    canvas = cv2.GaussianBlur(canvas, (7, 7), 0)

    # ✅ 원본과 동일한 가로세로 비율로 보이게 padding (기본 True)
    if pad_to_original_aspect and img_h != 0:
        target_aspect = img_w / img_h
        canvas = _pad_to_aspect(canvas, target_aspect)

    cv2.imwrite(out_path, canvas)
    return out_path