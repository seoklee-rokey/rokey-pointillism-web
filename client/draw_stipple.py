import cv2

import numpy as np

import random

from rembg import remove

from PIL import Image



# ===== 로봇 작업 영역 =====

X_LEFT = 275.75

Y_TOP = -39.2

X_RIGHT = 488.31

Y_BOTTOM = 125.84



# ===== 팔레트 (1~18번 색상 인덱스 기준) =====



palette = [



    (159, 52, 58), (179, 78, 96), (187, 99, 61), (202, 181, 53),



    (104, 150, 75), (71, 125, 99), (79, 118, 135), (58, 103, 168),



    (53, 46, 139), (68, 72, 97), (56, 62, 78), (111, 70, 140),



    (130, 76, 113), (209, 145, 171), (103, 78, 71), (194, 166, 129),



    (144, 128, 69), (0, 0, 0), (255, 255, 255)



]







# --------------------------



# 1️⃣ 이미지 리사이즈



def resize_keep_ratio(image, max_size):



    h, w = image.shape[:2]



    if max(h, w) <= max_size:



        return image



    scale = max_size / max(h, w)



    return cv2.resize(image, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)







# --------------------------



# 2️⃣ 팔레트 최근접 색상



def nearest_color(r, g, b):



    target = np.uint8([[[b, g, r]]])



    target_lab = cv2.cvtColor(target, cv2.COLOR_BGR2LAB).astype(np.float32)[0][0]



    L, A, B = target_lab



    min_dist = float('inf')



    nearest = palette[0]



    for pr, pg, pb in palette:



        pal = np.uint8([[[pb, pg, pr]]])



        pal_lab = cv2.cvtColor(pal, cv2.COLOR_BGR2LAB).astype(np.float32)[0][0]



        pL, pA, pB = pal_lab



        # 🔥 흰색 특별 처리



        if (pr, pg, pb) == (255, 255, 255):



            if abs(A - 128) > 6 or abs(B - 128) > 6:



                continue



        # 🔥 색상 중심 거리 (L 거의 무시)



        dist = 0.05*((L-pL)**2) + ((A-pA)**2) + ((B-pB)**2)



        if dist < min_dist:



            min_dist = dist



            nearest = (pr, pg, pb)



    return nearest







# --------------------------



# 3️⃣ Grid 기반 NN 정렬



def order_points_nn(points, cell_size=10):



    if len(points) == 0: return []



    pts = points.copy()



    grid = {}



    def cell_coord(p): return (int(p[0])//cell_size, int(p[1])//cell_size)



    for p in pts: grid.setdefault(cell_coord(p), []).append(p)



    def remove_point(p):



        c = cell_coord(p)



        grid[c].remove(p)



        if not grid[c]: del grid[c]



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



                    if abs(dx)!=radius and abs(dy)!=radius: continue



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





# --------------------------



# 4️⃣ 점 생성 (자동 회전 적용)



def generate_stipple_points(path, edge_prob, inner_density, color_mode,



                            max_size, canny_low=80, canny_high=150,



                            remove_background=False, min_dist=4):



    # 이미지 로드



    image = cv2.imread(path)



    if image is None: raise ValueError("이미지 로드 실패")







    # 배경 제거



    if remove_background:



        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)



        pil_img = Image.fromarray(image_rgb)



        output = remove(pil_img)



        output_np = np.array(output)



        if output_np.shape[2] == 4:



            alpha = output_np[:, :, 3]



            mask = alpha > 10



            image_rgb[~mask] = [255, 255, 255]



        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)







    # 이미지 리사이즈 먼저



    image = resize_keep_ratio(image, max_size)







    # 자동 회전 적용: 로봇 작업 영역의 긴 쪽에 맞춤



    img_h, img_w = image.shape[:2]



    work_w, work_h = X_RIGHT - X_LEFT, Y_BOTTOM - Y_TOP







    img_ratio = img_w / img_h



    work_ratio = work_w / work_h



    # 회전 적용 여부 판단

    rotated = False



    if (img_ratio > 1 and work_ratio < 1) or (img_ratio < 1 and work_ratio > 1):



        # 이미지와 작업 영역이 세로/가로 방향이 반대면 90도 회전



        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

        rotated = True

        img_h, img_w = image.shape[:2]







    # 회전 후 grayscale, edges, h, w 계산



    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)



    edges = cv2.Canny(gray, canny_low, canny_high)



    h, w = gray.shape







    # 후보점 생성



    edge_candidates, inner_candidates = [], []



    for y in range(h):



        for x in range(w):



            if gray[y, x] > 250: continue



            if edges[y, x] > 0:



                if random.random() < edge_prob: edge_candidates.append((x, y))



            else:



                brightness = gray[y, x]/255.0



                if random.random() < (1-brightness)*inner_density:



                    inner_candidates.append((x, y))







    candidates = [(x,y,'edge') for x,y in edge_candidates] + [(x,y,'inner') for x,y in inner_candidates]



    random.shuffle(candidates)







    # 2D mask



    occupied = np.zeros((h,w), dtype=bool)



    final_points = []



    for x, y, _ in candidates:



        if occupied[y,x]: continue



        if color_mode=="bw":



            rgb_color = (0,0,0)



        else:



            b,g,r = image[y,x]



            rgb_color = nearest_color(r,g,b)



            if rgb_color==(255,255,255): continue



        color_index = color_to_index(rgb_color)



        final_points.append((x,y,color_index))



        y0 = max(0, y-min_dist); y1 = min(h, y+min_dist+1)



        x0 = max(0, x-min_dist); x1 = min(w, x+min_dist+1)



        occupied[y0:y1, x0:x1] = True







    # 색상 기준 정렬 후 NN 정렬



    color_groups = {}



    for x,y,color_index in final_points:



        color_groups.setdefault(color_index, []).append((x,y))



    def color_brightness(idx): r,g,b = palette[idx-1]; return 0.299*r+0.587*g+0.114*b



    sorted_colors = sorted(color_groups.keys(), key=color_brightness, reverse=True)



    ordered_list = []



    for color_index in sorted_colors:



        group_pts = color_groups[color_index]



        ordered_xy = order_points_nn(group_pts)



        for x,y in ordered_xy: ordered_list.append((x,y,color_index))







    return ordered_list, w, h, rotated







# --------------------------



# 5️⃣ 색상 → 번호



def color_to_index(color):



    if color in palette: return palette.index(color)+1



    return 1







# --------------------------



# 6️⃣ imshow 확인



def show_stipple(points_list, img_w, img_h):



    mm_to_px = 4



    work_w, work_h = X_RIGHT-X_LEFT, Y_BOTTOM-Y_TOP



    canvas_w, canvas_h = int(work_w*mm_to_px), int(work_h*mm_to_px)



    canvas = np.ones((canvas_h, canvas_w, 3), dtype=np.uint8)*255



    img_ratio, work_ratio = img_w/img_h, work_w/work_h



    scale = work_w/img_w if img_ratio>work_ratio else work_h/img_h



    draw_w, draw_h = img_w*scale, img_h*scale



    offset_x, offset_y = (work_w-draw_w)/2, (work_h-draw_h)/2



    real_radius_mm = 1.5; radius_px=int(real_radius_mm*mm_to_px)



    for x,y,color_index in points_list:



        mm_x, mm_y = offset_x + x*scale, offset_y + y*scale



        px, py = int(mm_x*mm_to_px), int(mm_y*mm_to_px)



        r,g,b = palette[color_index-1]



        cv2.circle(canvas,(px,py),radius_px,(int(b),int(g),int(r)),-1)



    canvas = cv2.GaussianBlur(canvas,(7,7),0)



    cv2.imshow("Stipple Preview",canvas)



    cv2.waitKey(0)



    cv2.destroyAllWindows()







# --------------------------



# 7️⃣ 로봇 좌표 변환



def convert_to_robot_coords(points_list,img_w,img_h):



    work_w, work_h = X_RIGHT-X_LEFT, Y_BOTTOM-Y_TOP



    img_ratio, work_ratio = img_w/img_h, work_w/work_h



    scale = work_w/img_w if img_ratio>work_ratio else work_h/img_h



    draw_w, draw_h = img_w*scale, img_h*scale



    offset_x, offset_y = X_LEFT + (work_w-draw_w)/2, Y_TOP + (work_h-draw_h)/2



    robot_list=[]



    for x,y,color_index in points_list:



        rx = offset_x + (img_w-x)*scale



        ry = offset_y + y*scale



        robot_list.append([rx, ry, color_index])



    return robot_list







# --------------------------



# 8️⃣ Stipple 최종 실행



def stipple(edge_prob=0.6, inner_density=0.01, color_mode="color",



            max_size=400, img_path="img.jpeg",



            canny_low=80, canny_high=150,



            remove_background=False):



    # 회전 여부 적용

    points_list, img_w, img_h, rotated = generate_stipple_points(



        img_path, edge_prob, inner_density, color_mode,



        max_size, canny_low, canny_high,



        remove_background



    )



    show_stipple(points_list, img_w, img_h)



    robot_points_list = convert_to_robot_coords(points_list, img_w, img_h)



    return robot_points_list, rotated





# --------------------------



# 9️⃣ 실행 예시



if __name__ == "__main__":



    robot_points_list = stipple(



        edge_prob=0.5,



        inner_density=0.04,



        color_mode="color",



        max_size=400,



        img_path="turtle.png",



        canny_low=70,



        canny_high=90,



        remove_background=False



    )



    print("총 점 개수:", len(robot_points_list))



    print(robot_points_list[:10])