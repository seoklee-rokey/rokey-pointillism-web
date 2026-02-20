import cv2

import random

import numpy as np



# ===== 고정 파라미터 =====

CANNY_LOW = 80

CANNY_HIGH = 150



# ===== 팔레트 (RGB 순서) =====

palette = [

    (159, 52, 58), (179, 78, 96),

    (187, 99, 61),

    (202, 181, 53),

    (104, 150, 75), (71, 125, 99),

    (79, 118, 135), (58, 103, 168),

    (53, 46, 139), (68, 72, 97), (56, 62, 78),

    (111, 70, 140), (130, 76, 113),

    (209, 145, 171),

    (103, 78, 71), (194, 166, 129),

    (144, 128, 69),

    (34, 33, 41), (0, 0, 0), (255, 255, 255),

]



# --------------------------------------------------

# 이미지 리사이즈

def resize_keep_ratio(image, max_size):

    h, w = image.shape[:2]

    if max(h, w) <= max_size:

        return image

    scale = max_size / max(h, w)

    new_w = max(1, int(w * scale))

    new_h = max(1, int(h * scale))

    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)





# --------------------------------------------------

# 팔레트 최근접 색상

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

# 색상 → 인덱스 (1부터 시작)

def color_to_index(color):

    if color in palette:

        return palette.index(color) + 1

    return 1





# --------------------------------------------------

# 점묘화 생성

def generate_stipple(

        img_path,

        edge_prob=0.5,

        inner_density=0.01,

        color_mode="bw",

        max_size=200,

        show_preview=True

):

    image = cv2.imread(img_path)

    if image is None:

        raise ValueError("이미지 로드 실패")



    image = resize_keep_ratio(image, max_size)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)



    h, w = gray.shape

    result_points = []

    preview = 255 * np.ones((h, w, 3), dtype=np.uint8)



    for y in range(h):

        for x in range(w):

            draw_flag = False



            if edges[y, x] > 0:

                if random.random() < edge_prob:

                    draw_flag = True

            else:

                brightness = gray[y, x] / 255.0

                prob = (1 - brightness) * inner_density

                if random.random() < prob:

                    draw_flag = True



            if not draw_flag:

                continue



            brightness = gray[y, x] / 255.0

            if brightness > 0.95:

                continue



            if color_mode == "bw":

                rgb_color = (0, 0, 0)

            else:

                b, g, r = image[y, x]

                rgb_color = nearest_color(r, g, b)

                if rgb_color == (255, 255, 255):

                    continue



            v = color_to_index(rgb_color)

            result_points.append((x, y, v))



            cv2.circle(

                preview,

                (x, y),

                0,

                (rgb_color[2], rgb_color[1], rgb_color[0]),

                -1

            )



    if show_preview:

        cv2.imshow("Stipple Preview", preview)

        cv2.waitKey(0)

        cv2.destroyAllWindows()



    return result_points, w, h





# --------------------------------------------------

# Zhang-Suen Thinning (skeletonization) - 입력은 0/1이어야 함

def zhang_suen_thinning(binary_01):

    img = binary_01.copy().astype(np.uint8)

    changing = True

    h, w = img.shape



    while changing:

        changing = False

        to_remove = []



        # Step 1

        for y in range(1, h - 1):

            for x in range(1, w - 1):

                if img[y, x] != 1:

                    continue



                neighbors = [

                    img[y-1, x], img[y-1, x+1], img[y, x+1], img[y+1, x+1],

                    img[y+1, x], img[y+1, x-1], img[y, x-1], img[y-1, x-1]

                ]



                B = sum(neighbors)

                A = sum((neighbors[i] == 0 and neighbors[(i + 1) % 8] == 1) for i in range(8))



                if (2 <= B <= 6 and

                    A == 1 and

                    neighbors[0] * neighbors[2] * neighbors[4] == 0 and

                    neighbors[2] * neighbors[4] * neighbors[6] == 0):

                    to_remove.append((y, x))



        if to_remove:

            changing = True

            for y, x in to_remove:

                img[y, x] = 0



        to_remove = []



        # Step 2

        for y in range(1, h - 1):

            for x in range(1, w - 1):

                if img[y, x] != 1:

                    continue



                neighbors = [

                    img[y-1, x], img[y-1, x+1], img[y, x+1], img[y+1, x+1],

                    img[y+1, x], img[y+1, x-1], img[y, x-1], img[y-1, x-1]

                ]



                B = sum(neighbors)

                A = sum((neighbors[i] == 0 and neighbors[(i + 1) % 8] == 1) for i in range(8))



                if (2 <= B <= 6 and

                    A == 1 and

                    neighbors[0] * neighbors[2] * neighbors[6] == 0 and

                    neighbors[0] * neighbors[4] * neighbors[6] == 0):

                    to_remove.append((y, x))



        if to_remove:

            changing = True

            for y, x in to_remove:

                img[y, x] = 0



    return img  # 0/1





# --------------------------------------------------

# 중심선(한 줄) 추출: contrib 있으면 cv2.ximgproc.thinning 우선

def thin_to_single_line(binary_0_255):

    # binary_0_255: 0 또는 255

    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):

        return cv2.ximgproc.thinning(binary_0_255)  # 0/255 반환



    # contrib 없으면 Zhang-Suen으로 처리 (0/1 -> 0/255)

    binary_01 = (binary_0_255 > 0).astype(np.uint8)

    skel_01 = zhang_suen_thinning(binary_01)

    return (skel_01 * 255).astype(np.uint8)





# --------------------------------------------------

def generate_sketch(

    image_path,

    color_mode="bw",

    max_size=300,

    min_stroke_length=0,

    show_preview=False,

    # ✅ 한줄 중심선 안정화 옵션

    connect_kernel=3,

    connect_iter=1,

    use_close=True

):

    img = cv2.imread(image_path)

    if img is None:

        raise ValueError("이미지를 불러올 수 없습니다.")



    img = resize_keep_ratio(img, max_size)

    h, w = img.shape[:2]



    if color_mode not in ["bw", "color"]:

        raise ValueError("color_mode는 'bw' 또는 'color'만 지원")



    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)



    # ----------------------------

    # 이진화(선 영역 만들기)

    # ----------------------------

    if color_mode == "bw":

        binary = cv2.adaptiveThreshold(

            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,

            cv2.THRESH_BINARY_INV, 15, 5

        )

    else:

        _, binary = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY_INV)



    # ----------------------------

    # ✅ 연결 보강 (두껍게 만드는 dilate 대신 close 권장)

    # - 한 줄 중심선을 원하면 "최소한만" 연결

    # ----------------------------

    if use_close and connect_kernel > 1 and connect_iter > 0:

        k = np.ones((connect_kernel, connect_kernel), np.uint8)

        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k, iterations=connect_iter)



    # ----------------------------

    # ✅ 중심선(1픽셀) 추출

    # ----------------------------

    skel = thin_to_single_line(binary)  # 0/255



    # ----------------------------

    # Stroke 추출(연결요소 단위)

    # ----------------------------

    visited = np.zeros((h, w), dtype=bool)

    strokes = []



    auto_min_len = max(3, int(max(h, w) * 0.01))

    if min_stroke_length > 0:

        auto_min_len = min_stroke_length



    def dfs(sx, sy, out_stroke):

        stack = [(sx, sy)]

        visited[sy, sx] = True



        while stack:

            cx, cy = stack.pop()



            if color_mode == "bw":

                v = 1

            else:

                b, g, r = img[cy, cx]

                rgb = nearest_color(r, g, b)

                if rgb == (255, 255, 255):

                    continue

                v = color_to_index(rgb)



            out_stroke.append((cx, cy, v))



            # 8-이웃

            for ny in range(cy - 1, cy + 2):

                for nx in range(cx - 1, cx + 2):

                    if nx == cx and ny == cy:

                        continue

                    if 0 <= nx < w and 0 <= ny < h:

                        if (not visited[ny, nx]) and (skel[ny, nx] > 0):

                            visited[ny, nx] = True

                            stack.append((nx, ny))



    for y in range(h):

        for x in range(w):

            if skel[y, x] > 0 and not visited[y, x]:

                stroke_pixels = []

                dfs(x, y, stroke_pixels)

                if len(stroke_pixels) >= auto_min_len:

                    strokes.append(stroke_pixels)



    # ----------------------------

    # Preview

    # ----------------------------

    if show_preview:

        preview = 255 * np.ones((h, w, 3), dtype=np.uint8)

        for stroke in strokes:

            for x, y, v in stroke:

                rgb = palette[v - 1] if color_mode == "color" else (0, 0, 0)

                cv2.circle(preview, (x, y), 0, (rgb[2], rgb[1], rgb[0]), -1)

        cv2.imshow("Sketch Preview", preview)

        cv2.waitKey(0)

        cv2.destroyAllWindows()



    return strokes, w, h