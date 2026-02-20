import sys
import os
import time
import rclpy
import DR_init

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(BASE_DIR)

from web_module.draw_module import generate_sketch
from robot_module.draw_planner import convert_strokes_to_robot_coords

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 25
ACC = 50

DRAW_Z = 83.347      # 종이에 닿는 높이
LIFT_Z = 100         # 이동 높이

RX = 0
RY = 180
RZ = 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


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


def perform_drawing(robot_strokes):
    from DSR_ROBOT2 import (
        posx, movej, movel, movesx,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        release_force,
        release_compliance_ctrl,
        wait,
        DR_FC_MOD_REL,
        DR_MVS_VEL_NONE
    )

    DRAW_VEL = 25
    DRAW_ACC = 40
    Z_FORCE = 3            # ✅ 2N이 약하면 3~5로 올려(종이에 파고들면 다시 내림)
    MAX_SEG = 80

    # ✅ 접근 높이(힘제어 켜기 직전)
    APPROACH_Z = DRAW_Z + 2.0  # 0.2 말고 2mm 위에서 접근 후, DRAW_Z로 '찍고' force ON

    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)

    for stroke_idx, stroke in enumerate(robot_strokes):
        if len(stroke) < 2:
            continue

        print(f"✏️ Drawing stroke {stroke_idx}  (points={len(stroke)})")

        sx, sy, _ = stroke[0]

        # 1) 시작점 위로 이동
        movel(posx([sx, sy, LIFT_Z, RX, RY, RZ]), vel=VELOCITY, acc=ACC)

        # 2) 접근 높이로 내려오기
        movel(posx([sx, sy, APPROACH_Z, RX, RY, RZ]), vel=DRAW_VEL, acc=DRAW_ACC)

        # ✅ 3) Force ON 전에 DRAW_Z로 "무조건 접촉" 만들기 (핵심)
        movel(posx([sx, sy, DRAW_Z, RX, RY, RZ]), vel=10, acc=20)
        wait(0.1)

        # 4) Force ON
        set_ref_coord(0)  # base 기준 (너 설정 유지)
        task_compliance_ctrl(stx=[3000, 3000, 80, 300, 300, 300])
        wait(0.2)

        set_desired_force(
            fd=[0, 0, -Z_FORCE, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )
        wait(0.2)

        # ✅ 5) 한 획은 무조건 이어그리기: stroke 내부에서 펜업 로직 제거
        xlist = []
        for (x, y, _) in stroke:
            # ✅ 그리는 점들은 DRAW_Z에 두지 말고, 힘제어가 알아서 눌러주게 "근처 높이"로 유지
            #    다만 너무 높으면 안 닿으니 DRAW_Z 근처로 유지
            px = posx([x, y, DRAW_Z + 0.2, RX, RY, RZ])
            xlist.append(px)

            if len(xlist) >= MAX_SEG:
                movesx(
                    xlist,
                    vel=[DRAW_VEL + 10, DRAW_VEL],
                    acc=[DRAW_ACC + 20, DRAW_ACC],
                    vel_opt=DR_MVS_VEL_NONE
                )
                xlist = []

        if len(xlist) >= 2:
            movesx(
                xlist,
                vel=[DRAW_VEL + 10, DRAW_VEL],
                acc=[DRAW_ACC + 20, DRAW_ACC],
                vel_opt=DR_MVS_VEL_NONE
            )

        # 6) Force OFF + 들어올리기
        release_force()
        release_compliance_ctrl()
        wait(0.2)

        ex, ey, _ = stroke[-1]
        movel(posx([ex, ey, LIFT_Z, RX, RY, RZ]), vel=VELOCITY, acc=ACC)

    print("🎉 Drawing Finished")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        # 1) 이미지 -> strokes
        strokes, img_w, img_h = generate_sketch(
            "/home/daehyuk/Downloads/rokey-pointillism-web-feature-sketch_one_line/robot/sign.png",
            color_mode="bw",
            max_size=300,
            min_stroke_length=15,
            show_preview=False
        )

        # ✅ NN 정렬 제거: 1픽셀 선은 순서가 생명이라 최단거리 재정렬하면 점프가 생김
        # strokes_ordered = [order_stroke_points_nn(s) for s in strokes]
        strokes_ordered = strokes

        # 2) 로봇 좌표 변환
        robot_strokes = convert_strokes_to_robot_coords(strokes_ordered, img_w, img_h)

        # (디버그) 각 stroke 길이 출력
        lens = sorted([len(s) for s in robot_strokes], reverse=True)[:10]
        print(f"strokes={len(robot_strokes)}, top lens={lens}")

        # 3) 드로잉
        perform_drawing(robot_strokes)

    except KeyboardInterrupt:
        print("⛔ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
