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
            rx = offset_x + x * scale
            ry = offset_y + y * scale
            robot_stroke.append((rx, ry, v))
        robot_strokes.append(robot_stroke)

    return robot_strokes


# --------------------------------------------------
# stroke 내부 순서 NN 정렬 (선택적)
def order_stroke_points_nn(stroke, cell_size=10):
    """
    stroke: [(x, y, v), ...]
    return: [(x, y, v), ...]  # NN 순서
    """
    if not stroke:
        return []

    pts = [(p[0], p[1]) for p in stroke]
    grid = {}

    def cell_coord(p):
        return (int(p[0]) // cell_size, int(p[1]) // cell_size)

    for p in pts:
        c = cell_coord(p)
        grid.setdefault(c, []).append(p)

    def remove_point(p):
        c = cell_coord(p)
        grid[c].remove(p)
        if not grid[c]:
            del grid[c]

    # 시작점은 가장 왼쪽 점
    current = min(pts, key=lambda p: p[0])  
    ordered = [current]
    remove_point(current)
    pts.remove(current)

    def find_nearest(cur):
        cx, cy = cell_coord(cur)
        radius = 0
        while True:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cell = (cx + dx, cy + dy)
                    if cell in grid and grid[cell]:
                        return min(
                            grid[cell],
                            key=lambda p: (p[0]-cur[0])**2 + (p[1]-cur[1])**2
                        )
            radius += 1

    while pts:
        next_pt = find_nearest(current)
        ordered.append(next_pt)
        remove_point(next_pt)
        pts.remove(next_pt)
        current = next_pt

    # 색상 값 복원
    ordered_with_v = [(x, y, stroke[0][2]) for x, y in ordered]
    return ordered_with_v
