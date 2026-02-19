from stipple_module import generate_stipple

def main():

    points, img_w, img_h = generate_stipple(
        img_path="img.jpeg"
    )

    print("이미지 크기:", img_w, img_h)
    print("점 개수:", len(points))

    # 출력 형식 확인
    print(points[:10])  # 앞 10개만 출력


if __name__ == "__main__":
    main()

