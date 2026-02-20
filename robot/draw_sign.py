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

DRAW_Z = 83.347     # 종이에 닿는 높이 (실측 후 조정!)
LIFT_Z = 150     # 선 이동 시 높이

RX = 0
RY = 180
RZ = 0

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

    # -----------------------------
    # 안정 세팅값
    # -----------------------------
    DRAW_VEL = 25
    DRAW_ACC = 40
    Z_FORCE = 2
    MAX_SEG = 80

    PEN_LIFT_THRESHOLD = 3.0   # 거리 기준 (mm) -> 실제 로봇 동작 보고 미세 조정 필요

    APPROACH_Z = DRAW_Z + 0.2

    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)

    for stroke_idx, stroke in enumerate(robot_strokes):

        if len(stroke) < 2:
            continue

        print(f"✏️ Drawing stroke {stroke_idx}")

        sx, sy, _ = stroke[0]

        # -----------------------------
        # 시작점 위로 이동
        # -----------------------------
        movel(posx([sx, sy, LIFT_Z, RX, RY, RZ]),
              vel=VELOCITY, acc=ACC)

        movel(posx([sx, sy, APPROACH_Z, RX, RY, RZ]),
              vel=DRAW_VEL, acc=DRAW_ACC)

        # -----------------------------
        # Force ON
        # -----------------------------
        set_ref_coord(0)

        task_compliance_ctrl(
            stx=[3000, 3000, 80, 300, 300, 300]
        )
        wait(0.2)

        set_desired_force(
            fd=[0, 0, -Z_FORCE, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            mod=DR_FC_MOD_REL
        )
        wait(0.4)

        prev_x, prev_y = sx, sy
        xlist = []

        # -----------------------------
        # Stroke 따라가기
        # -----------------------------
        for i, (x, y, _) in enumerate(stroke):

            # 거리 계산
            dist = ((x - prev_x)**2 + (y - prev_y)**2) ** 0.5

            # 🔴 멀리 떨어지면 펜 들기
            if dist > PEN_LIFT_THRESHOLD:

                # Force OFF
                release_force()
                release_compliance_ctrl()
                wait(0.1)

                # Lift
                movel(posx([prev_x, prev_y, LIFT_Z, RX, RY, RZ]),
                      vel=VELOCITY, acc=ACC)

                # 이동
                movel(posx([x, y, LIFT_Z, RX, RY, RZ]),
                      vel=VELOCITY, acc=ACC)

                movel(posx([x, y, APPROACH_Z, RX, RY, RZ]),
                      vel=DRAW_VEL, acc=DRAW_ACC)

                # Force 다시 ON
                task_compliance_ctrl(
                    stx=[3000, 3000, 80, 300, 300, 300]
                )
                wait(0.2)

                set_desired_force(
                    fd=[0, 0, -Z_FORCE, 0, 0, 0],
                    dir=[0, 0, 1, 0, 0, 0],
                    mod=DR_FC_MOD_REL
                )
                wait(0.3)

                xlist = []

            px = posx([x, y, APPROACH_Z, RX, RY, RZ])
            xlist.append(px)

            # segment 단위 실행
            if len(xlist) >= MAX_SEG:
                movesx(
                    xlist,
                    vel=[DRAW_VEL + 10, DRAW_VEL],
                    acc=[DRAW_ACC + 20, DRAW_ACC],
                    vel_opt=DR_MVS_VEL_NONE
                )
                xlist = []

            prev_x, prev_y = x, y

        # 남은 segment 처리
        if len(xlist) >= 2:
            movesx(
                xlist,
                vel=[DRAW_VEL + 10, DRAW_VEL],
                acc=[DRAW_ACC + 20, DRAW_ACC],
                vel_opt=DR_MVS_VEL_NONE
            )

        # -----------------------------
        # Stroke 종료 → Force OFF
        # -----------------------------
        release_force()
        release_compliance_ctrl()
        wait(0.2)

        ex, ey, _ = stroke[-1]

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
            "/home/daehyuk/Downloads/rokey-pointillism-web-feature-sketch/robot/sign.png",
            color_mode="bw",
            max_size=300,
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