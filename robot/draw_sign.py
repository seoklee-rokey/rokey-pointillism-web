import sys
import os
import time
import rclpy
import DR_init

# -------------------------------------------------
# 🔹 경로 설정 (robot 폴더를 import 루트로 추가)
# -------------------------------------------------
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(BASE_DIR)

from web_module.draw_module import generate_sketch
from robot_module.draw_planner import (
    convert_strokes_to_robot_coords,
    order_stroke_points_nn
)

# -------------------------------------------------
# 🔹 로봇 설정
# -------------------------------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 25
ACC = 50

DRAW_Z = 68     # 종이에 닿는 높이 (실측 후 조정!)
LIFT_Z = 136     # 선 이동 시 높이

RX = 180
RY = 0
RZ = 180

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# -------------------------------------------------
# 🔹 로봇 초기화
# -------------------------------------------------
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    from DSR_ROBOT2 import ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    from DSR_ROBOT2 import set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    time.sleep(2)
    print("✅ Robot Initialized")


# -------------------------------------------------
# 🔹 실제 드로잉 수행
# -------------------------------------------------
def perform_drawing(robot_strokes):
    from DSR_ROBOT2 import posx, movej, movel

    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)

    for stroke_idx, stroke in enumerate(robot_strokes):

        if len(stroke) < 2:
            continue

        print(f"✏️ Drawing stroke {stroke_idx}")

        # 1️⃣ 시작점 위로 이동 (펜 들고 이동)
        sx, sy, v = stroke[0]
        movel(posx([sx, sy, LIFT_Z, RX, RY, RZ]),
              vel=VELOCITY, acc=ACC)

        # 2️⃣ 펜 내리기
        movel(posx([sx, sy, DRAW_Z, RX, RY, RZ]),
              vel=VELOCITY, acc=ACC)

        # 3️⃣ 선 그리기
        for x, y, v in stroke[1:]:
            movel(posx([x, y, DRAW_Z, RX, RY, RZ]),
                  vel=VELOCITY, acc=ACC)

        # 4️⃣ 펜 올리기
        ex, ey, v = stroke[-1]
        movel(posx([ex, ey, LIFT_Z, RX, RY, RZ]),
              vel=VELOCITY, acc=ACC)

    print("🎉 Drawing Finished")


# -------------------------------------------------
# 🔹 메인
# -------------------------------------------------
def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node("draw_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        # -----------------------------------------
        # 1️⃣ 이미지 → stroke 생성
        # -----------------------------------------
        strokes, img_w, img_h = generate_sketch(
            "img.jpeg",
            color_mode="bw",
            max_size=600,
            min_stroke_length=15,
            show_preview=False
        )

        # -----------------------------------------
        # 2️⃣ stroke 내부 NN 정렬
        # -----------------------------------------
        strokes_ordered = [
            order_stroke_points_nn(s) for s in strokes
        ]

        # -----------------------------------------
        # 3️⃣ 로봇 좌표 변환
        # -----------------------------------------
        robot_strokes = convert_strokes_to_robot_coords(
            strokes_ordered,
            img_w,
            img_h
        )

        # -----------------------------------------
        # 4️⃣ 실제 드로잉
        # -----------------------------------------
        perform_drawing(robot_strokes)

    except KeyboardInterrupt:
        print("⛔ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

