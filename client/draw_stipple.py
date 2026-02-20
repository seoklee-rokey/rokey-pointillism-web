import cv2
import numpy as np
import random
from itertools import groupby

# ===== 고정 파라미터 =====
CANNY_LOW = 80
CANNY_HIGH = 150

# ===== 로봇 작업 영역 =====
X_LEFT = 320
Y_TOP = 0
X_RIGHT = 500
Y_BOTTOM = 120

# ===== 팔레트 (1~18번 색상 인덱스 기준) =====
palette = [

    # 🔴 빨강 계열
    (159, 52, 58),     # 딥 레드
    (179, 78, 96),     # 로즈 레드

    # 🟠 주황 계열
    (187, 99, 61),     # 브릭 오렌지

    # 🟡 노랑 계열
    (202, 181, 53),    # 머스터드 옐로우

    # 🟢 초록 계열
    (104, 150, 75),    # 올리브 그린
    (71, 125, 99),     # 포레스트 그린

    # 🟢 청록 계열
    (79, 118, 135),    # 틸 블루
    (58, 103, 168),    # 블루 그린

    # 🔵 파랑 계열
    (53, 46, 139),     # 딥 블루
    (68, 72, 97),      # 슬레이트 블루
    (56, 62, 78),      # 다크 블루 그레이

    # 🟣 보라 계열
    (111, 70, 140),    # 퍼플
    (130, 76, 113),    # 모브 퍼플

    # 🌸 핑크 계열
    (209, 145, 171),   # 소프트 핑크

    # 🟤 갈색 계열
    (103, 78, 71),     # 브라운
    (194, 166, 129),   # 베이지 브라운

    # 🟢 카키/올리브 계열
    (144, 128, 69),    # 카키

    # ⚪ 무채색
    (34, 33, 41),      # 차콜 블랙
    (0, 0, 0),         # 블랙
    (255, 255, 255),   # 화이트
]


# --------------------------------------------------
# 1️⃣ 이미지 리사이즈
def resize_keep_ratio(image, max_size):
    h, w = image.shape[:2]
    if max(h, w) <= max_size:
        return image
    scale = max_size / max(h, w)
    return cv2.resize(image, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)

# --------------------------------------------------
# 2️⃣ 팔레트 최근접 색상
def nearest_color(r, g, b):
    min_dist = float('inf')
    nearest = palette[0]
    for pr, pg, pb in palette:
        dist = (int(r)-pr)**2 + (int(g)-pg)**2 + (int(b)-pb)**2
        if dist < min_dist:
            min_dist = dist
            nearest = (pr, pg, pb)
    return nearest

# --------------------------------------------------
# 3️⃣ Grid 기반 NN 정렬
def order_points_nn(points, cell_size=10):
    if len(points) == 0:
        return []

    pts = points.copy()
    grid = {}
    def cell_coord(p):
        return (int(p[0])//cell_size, int(p[1])//cell_size)
    for p in pts:
        grid.setdefault(cell_coord(p), []).append(p)
    def remove_point(p):
        c = cell_coord(p)
        grid[c].remove(p)
        if not grid[c]:
            del grid[c]
    current = pts[0]
    ordered = [current]
    remove_point(current)
    pts.remove(current)
    def find_nearest(cur):
        cx, cy = cell_coord(cur)
        radius = 0
        while True:
            for dx in range(-radius, radius+1):
                for dy in range(-radius, radius+1):
                    if abs(dx)!=radius and abs(dy)!=radius:
                        continue
                    cell = (cx+dx, cy+dy)
                    if cell in grid and grid[cell]:
                        return min(grid[cell], key=lambda p: (p[0]-cur[0])**2 + (p[1]-cur[1])**2)
            radius += 1
    while pts:
        next_pt = find_nearest(current)
        ordered.append(next_pt)
        remove_point(next_pt)
        pts.remove(next_pt)
        current = next_pt
    return ordered

# --------------------------------------------------
# 4️⃣ 점 생성
def generate_stipple_points(path, edge_prob, inner_density, color_mode, max_size, canny_low=80, canny_high=150):
    """
    점 생성 → [(x, y, color_index), ...] 단일 리스트 반환
    """
    image = cv2.imread(path)
    if image is None:
        raise ValueError("이미지 로드 실패")
    
    # 이미지 리사이즈
    image = resize_keep_ratio(image, max_size)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, canny_low, canny_high)
    h, w = gray.shape
    
    points_list = []
    
    for y in range(h):
        for x in range(w):
            draw_flag = False
            if edges[y, x] > 0:
                if random.random() < edge_prob:
                    draw_flag = True
            else:
                brightness = gray[y, x]/255.0
                if random.random() < (1-brightness)*inner_density:
                    draw_flag = True
            
            if draw_flag:
                if color_mode == "bw":
                    rgb_color = (0, 0, 0)
                else:
                    b, g, r = image[y, x]
                    rgb_color = nearest_color(r, g, b)
                
                # 색상 → 인덱스 변환 후 바로 리스트에 추가
                color_index = color_to_index(rgb_color)
                points_list.append((x, y, color_index))
    
    # 색상 기준 정렬 후 NN 정렬
    from itertools import groupby
    points_list_sorted = sorted(points_list, key=lambda p: p[2])
    
    final_list = []
    for color_index, group in groupby(points_list_sorted, key=lambda p: p[2]):
        group_pts = [(p[0], p[1]) for p in group]
        ordered_xy = order_points_nn(group_pts)
        for x, y in ordered_xy:
            final_list.append((x, y, color_index))
    
    return final_list, w, h
# --------------------------------------------------
# 5️⃣ imshow 확인
def show_stipple(points_list, img_w, img_h):
    """
    리스트 [(x, y, color_index), ...]를 화면에 표시
    """
    canvas = np.ones((img_h, img_w, 3), dtype=np.uint8) * 255
    for x, y, color_index in points_list:
        # color_index → RGB
        color = palette[color_index-1]  # 인덱스가 1~n이므로 -1
        cv2.circle(canvas, (x, y), 0, color, -1)
    cv2.imshow("Stipple Preview", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# --------------------------------------------------
# 6️⃣ 색상 → 번호
def color_to_index(color):
    if color in palette:
        return palette.index(color)+1
    return 1

# --------------------------------------------------
# 7️⃣ 로봇 좌표 변환 (비율 유지)
def convert_to_robot_coords(points_list, img_w, img_h):
    """
    리스트 [(x,y,color_index), ...] → 로봇 좌표 리스트 [(rx, ry, color_index), ...]
    """
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
    offset_x = X_LEFT + (work_w - draw_w)/2
    offset_y = Y_TOP  + (work_h - draw_h)/2

    robot_list = []
    for x, y, color_index in points_list:
        rx = offset_x + (img_w - x)*scale  # 좌우 반전
        ry = offset_y + y*scale
        robot_list.append([rx, ry, color_index])

    return robot_list

# --------------------------------------------------
# 8️⃣ MAIN
def stipple(edge_prob=0.6, inner_density=0.01, color_mode="color",
         max_size=400, img_path="img.jpeg", canny_low=80, canny_high=150):
    """
    Stipple 생성 후 로봇 좌표 변환 및 리스트 반환

    Parameters
    ----------
    edge_prob : float
        엣지 점 생성 확률
    inner_density : float
        내부 점 생성 밀도
    color_mode : str
        "color" 또는 "bw"
    max_size : int
        이미지 최대 리사이즈
    img_path : str
        입력 이미지 경로

    Returns
    -------
    robot_points_list : list of [x, y, color_index]
    """

    points_list, img_w, img_h = generate_stipple_points(
        img_path, edge_prob, inner_density, color_mode, max_size, canny_low, canny_high
    )

    # 🔹 imshow 확인
    #show_stipple(points_list, img_w, img_h)

    # 🔹 로봇 좌표 변환
    robot_points_list = convert_to_robot_coords(points_list, img_w, img_h)

    return robot_points_list

# --------------------------------------------------
# 9️⃣ 실행
if __name__ == "__main__":
    robot_points_list = stipple(
        edge_prob=0.6,
        inner_density=0.01,
        color_mode="color",
        max_size=400,
        img_path="img.jpeg",
        canny_low=50,
        canny_high=120
    )

    print("총 점 개수:", len(robot_points_list))
    print(robot_points_list[:10])
    if robot_points_list:
        xs = [p[0] for p in robot_points_list]
        ys = [p[1] for p in robot_points_list]

        print(f"X min: {min(xs):.2f}, X max: {max(xs):.2f}")
        print(f"Y min: {min(ys):.2f}, Y max: {max(ys):.2f}")