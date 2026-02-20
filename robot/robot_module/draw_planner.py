# stipple_robot_planner.py

# ===== 로봇 작업 영역 =====
X_LEFT = 320
Y_TOP = 0
X_RIGHT = 500
Y_BOTTOM = 120


# --------------------------------------------------
# stroke 기반 로봇 좌표 변환
def convert_strokes_to_robot_coords(strokes, img_w, img_h):
    """
    strokes: [[(x, y, v), ...], ...]  # stroke 단위 리스트
    img_w, img_h: 이미지 크기
    return: [[(rx, ry, v), ...], ...]  # stroke 단위 유지
    """

    if not strokes:
        return []

    work_w = X_RIGHT - X_LEFT
    work_h = Y_BOTTOM - Y_TOP

    img_ratio = img_w / img_h
    work_ratio = work_w / work_h

    # 비율 유지 스케일
    if img_ratio > work_ratio:
        scale = work_w / img_w
    else:
        scale = work_h / img_h

    draw_w = img_w * scale
    draw_h = img_h * scale

    offset_x = X_LEFT + (work_w - draw_w) / 2
    offset_y = Y_TOP  + (work_h - draw_h) / 2

    robot_strokes = []

    for stroke in strokes:
        robot_stroke = []
        for x, y, v in stroke:
            rx = offset_x + (img_w - x) * scale
            ry = offset_y + y * scale
            robot_stroke.append((rx, ry, v))
        robot_strokes.append(robot_stroke)

    return robot_strokes


# --------------------------------------------------
# stroke 내부 순서 NN 정렬 (선택적)
def order_stroke_points_nn(stroke, close_loop=True):
    """
    stroke: [(x, y, v), ...]
    close_loop: True면 시작점으로 되돌아오는 폐곡선 생성
    """

    if not stroke:
        return []

    # 좌표만 분리
    remaining = [(p[0], p[1]) for p in stroke]

    # 1️⃣ 아무 점이나 시작점 (첫 점 사용)
    current = remaining.pop(0)
    ordered = [current]

    # 2️⃣ 가장 가까운 점 계속 선택
    while remaining:
        next_pt = min(
            remaining,
            key=lambda p: (p[0] - current[0])**2 +
                          (p[1] - current[1])**2
        )
        ordered.append(next_pt)
        remaining.remove(next_pt)
        current = next_pt

    # 3️⃣ 폐곡선이면 시작점으로 복귀
    if close_loop:
        ordered.append(ordered[0])

    # 4️⃣ 색상값 복원
    v_val = stroke[0][2]
    ordered_with_v = [(x, y, v_val) for x, y in ordered]

    return ordered_with_v