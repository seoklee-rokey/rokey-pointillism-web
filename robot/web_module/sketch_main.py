from draw_module import generate_sketch


def main():

    image_path = "/home/leeseungmin/Desktop/Doosan/rokey_ws/cooperation1/robot/sign.png"  # 테스트 이미지 경로

    strokes, w, h = generate_sketch(
        image_path,
        color_mode="bw",
        max_size=300,
        min_stroke_length=40,
        show_preview=True
    )

    print("====== Sketch Result ======")
    print("Image Size:", w, "x", h)
    print("Number of strokes:", len(strokes))

    total_points = 0
    for stroke in strokes:
        total_points += len(stroke)

    print("Total points:", total_points)

    # 첫 5개 stroke 확인
    for i in range(min(5, len(strokes))):
        print(f"Stroke {i+1} length:", len(strokes[i]))

    print("===========================")


if __name__ == "__main__":
    main()
