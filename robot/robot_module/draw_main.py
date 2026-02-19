import sys
import os

# draw_main.py 기준 상위 폴더 ../web_module 경로 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../web_module')))

from draw_module import generate_sketch
from draw_planner import convert_strokes_to_robot_coords, order_stroke_points_nn


# 1️⃣ 스케치 생성
strokes, w, h = generate_sketch("/home/leeseungmin/Desktop/Doosan/rokey_ws/cooperation1/robot/sign.png", color_mode="bw", max_size=300)

# 2️⃣ 필요하면 stroke 내부 NN 정렬
strokes_ordered = [order_stroke_points_nn(s) for s in strokes]

# 3️⃣ 로봇 좌표 변환
robot_strokes = convert_strokes_to_robot_coords(strokes_ordered, w, h)

# 이제 robot_strokes를 ROS2 action으로 바로 사용 가능
