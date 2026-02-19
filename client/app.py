import os
import secrets
import shutil
from pathlib import Path

import time


from flask import Flask, render_template, request, redirect, url_for, send_from_directory, jsonify, abort
import qrcode
from PIL import Image, ImageOps

# app.py 상단 어딘가에 추가(점묘화용 추가)
#from stipple_processor import generate_stipple
from stipple_module import generate_stipple
#test 주석

app = Flask(__name__)

BASE_DIR = Path(__file__).resolve().parent
UPLOAD_DIR = BASE_DIR / "uploads"
QR_DIR = BASE_DIR / "static" / "qr"
GALLERY_DIR = BASE_DIR / "static" / "gallery"

UPLOAD_DIR.mkdir(exist_ok=True)
QR_DIR.mkdir(parents=True, exist_ok=True)
GALLERY_DIR.mkdir(parents=True, exist_ok=True)

ALLOWED_EXT = {"png", "jpg", "jpeg", "webp"}

def new_token() -> str:
    return secrets.token_urlsafe(12)

def token_dir(token: str) -> Path:
    d = UPLOAD_DIR / token
    d.mkdir(parents=True, exist_ok=True)
    return d

def allowed(filename: str) -> bool:
    if "." not in filename:
        return False
    ext = filename.rsplit(".", 1)[1].lower()
    return ext in ALLOWED_EXT

def save_grayscale(src_path: Path, dst_path: Path):
    img = Image.open(src_path).convert("RGB")
    gray = ImageOps.grayscale(img).convert("RGB")
    gray.save(dst_path)

@app.route("/")
def index():
    return render_template("index.html")

# -------------------------
# 1) 사진 업로드하기 (QR)
# -------------------------
@app.route("/qr")
def qr_page():
    token = new_token()
    # QR이 찍히면 휴대폰에서 열릴 업로드 URL
    upload_url = url_for("phone_upload_page", token=token, _external=True)

    # QR 이미지 생성
    qr_img_path = QR_DIR / f"{token}.png"
    img = qrcode.make(upload_url)
    img.save(qr_img_path)

    return render_template("qr.html", token=token, upload_url=upload_url, qr_img=url_for("static", filename=f"qr/{token}.png"))

@app.route("/u/<token>", methods=["GET"])
def phone_upload_page(token):
    # 단순 토큰 유효성: 폴더 만들어두기
    token_dir(token)
    return render_template("upload_phone.html", token=token)

@app.route("/u/<token>", methods=["POST"])
def phone_upload_post(token):
    token_path = token_dir(token)

    if "file" not in request.files:
        abort(400, "No file field")
    f = request.files["file"]
    if not f.filename:
        abort(400, "Empty filename")
    if not allowed(f.filename):
        abort(400, "Unsupported file type")

    ext = f.filename.rsplit(".", 1)[1].lower()
    original_path = token_path / f"original.{ext}"
    f.save(original_path)

    # 커스터마이징용 프로토타입(그레이스케일) 생성
    # processed_path = token_path / "processed.png"
    # save_grayscale(original_path, processed_path)
    # 커스터마이징용 프로토타입(색상 적용) 생성
    processed_path = token_path / "processed.png"
    #generate_stipple(original_path, processed_path, 50, color=True)
    generate_stipple(
        img_path=original_path,
        edge_prob=0.5,
        inner_density=0.01,
        color_mode="bw",
        save_preview_path=processed_path,
        show_preview=False
    )


    return render_template("upload_done.html")
    #return redirect(url_for("customize_page", token=token))

# -------------------------
# 2) 사진 촬영하기 (태블릿 카메라)
# -------------------------
@app.route("/camera")
def camera_page():
    token = new_token()
    token_dir(token)
    return render_template("camera.html", token=token)

@app.route("/camera_upload/<token>", methods=["POST"])
def camera_upload(token):
    token_path = token_dir(token)
    if "image" not in request.files:
        abort(400, "No image field")

    f = request.files["image"]
    # 캡처는 png로 받는다고 가정
    original_path = token_path / "original.png"
    f.save(original_path)

    processed_path = token_path / "processed.png"
    #save_grayscale(original_path, processed_path)
    #generate_stipple(original_path, processed_path, 50, color=True)
    generate_stipple(
        img_path=original_path,
        edge_prob=0.5,
        inner_density=0.01,
        color_mode="bw",
        save_preview_path=processed_path,
        show_preview=False
    )

    return jsonify({"ok": True, "next": url_for("customize_page", token=token)})

# -------------------------
# 3) 전시관 사진 불러오기 (폴더 목록)
# -------------------------
@app.route("/gallery")
def gallery_page():
    token = new_token()
    token_dir(token)

    # static/gallery 내 이미지 목록
    items = []
    for p in sorted(GALLERY_DIR.glob("*")):
        if p.suffix.lower().lstrip(".") in ALLOWED_EXT:
            items.append(p.name)

    return render_template("gallery.html", token=token, items=items)

@app.route("/select_gallery/<token>", methods=["POST"])
def select_gallery(token):
    token_path = token_dir(token)
    filename = request.form.get("filename", "")
    if not filename:
        abort(400, "No filename")

    src = GALLERY_DIR / filename
    if not src.exists():
        abort(404, "File not found")

    # 선택한 파일을 uploads/token/original.ext 로 복사
    ext = src.suffix.lower().lstrip(".")
    original_path = token_path / f"original.{ext}"
    shutil.copyfile(src, original_path)

    processed_path = token_path / "processed.png"
    #save_grayscale(original_path, processed_path)
    #generate_stipple(original_path, processed_path, 50, color=True)
    generate_stipple(
        img_path=original_path,
        edge_prob=0.5,
        inner_density=0.01,
        color_mode="bw",
        save_preview_path=processed_path,
        show_preview=False
    )

    return redirect(url_for("customize_page", token=token))

# -------------------------
# 커스터마이징(로봇 작동 전) 페이지
# -------------------------
# @app.route("/customize/<token>")
# def customize_page(token):
#     token_path = token_dir(token)

#     # 원본 찾기
#     originals = list(token_path.glob("original.*"))
#     if not originals:
#         return "No image yet for this token.", 404

#     original = originals[0].name
#     processed = "processed.png"
#     return render_template(
#         "customize.html",
#         token=token,
#         original_url=url_for("uploaded_file", token=token, filename=original),
#         processed_url=url_for("uploaded_file", token=token, filename=processed),
#     )
@app.route("/customize/<token>")
def customize_page(token):
    token_path = token_dir(token)

    originals = list(token_path.glob("original.*"))
    if not originals:
        return "No image yet for this token.", 404

    original = originals[0]
    processed_path = token_path / "processed.png"

    # ✅ processed가 아직 없으면 여기서 생성 (타이밍 문제 방지)
    if not processed_path.exists():
        #save_grayscale(original, processed_path)
        #generate_stipple(original, processed_path, 50, color=True)
        generate_stipple(
            img_path=original,
            edge_prob=0.5,
            inner_density=0.01,
            color_mode="bw",
            save_preview_path=processed_path,
            show_preview=False
        )
    original_url = url_for("uploaded_file", token=token, filename=original.name)
    # ✅ 캐시 무효화 쿼리 (캐시 문제 방지)
    processed_url = url_for("uploaded_file", token=token, filename="processed.png") + f"?v={int(time.time())}"

    return render_template(
        "customize.html",
        token=token,
        original_url=original_url,
        processed_url=processed_url,
    )




@app.route("/uploads/<token>/<path:filename>")
def uploaded_file(token, filename):
    return send_from_directory(UPLOAD_DIR / token, filename)

@app.route("/check_upload/<token>")
def check_upload(token):
    token_path = token_dir(token)
    originals = list(token_path.glob("original.*"))
    return {"uploaded": len(originals) > 0}


# 점묘화용 추가
# @app.route("/process/<token>", methods=["POST"])
# def process_image(token):
#     token_path = token_dir(token)

#     originals = list(token_path.glob("original.*"))
#     if not originals:
#         abort(404, "No original image")

#     data = request.get_json(silent=True) or {}
#     density = int(data.get("density", 50))

#     original = originals[0]
#     processed_path = token_path / "processed.png"

#     # ✅ 점묘화 생성(실험용)
#     generate_stipple(original, processed_path, density)

#     # ✅ 캐시 무효화해서 브라우저가 새 파일을 다시 받게 함
#     processed_url = url_for("uploaded_file", token=token, filename="processed.png") + f"?v={int(time.time())}"
#     return jsonify({"ok": True, "processed_url": processed_url})

@app.route("/process/<token>", methods=["POST"])
def process_image(token):
    token_path = token_dir(token)

    originals = list(token_path.glob("original.*"))
    if not originals:
        abort(404, "No original image")

    data = request.get_json(silent=True) or {}

    # ✅ 새 파라미터 3개 받기 (기본값 포함)
    edge_prob = float(data.get("edge_prob", 0.5))          # 0.1 ~ 1.0
    inner_density = float(data.get("inner_density", 0.01)) # 0.00 ~ 0.50
    color_mode = data.get("color_mode", "bw")              # "bw" | "color"

    original = originals[0]
    processed_path = token_path / "processed.png"

    # ✅ 점묘화 생성 (웹용: 저장 경로를 save_preview_path로)
    generate_stipple(
        img_path=original,
        edge_prob=edge_prob,
        inner_density=inner_density,
        color_mode=color_mode,
        save_preview_path=processed_path,
        show_preview=False
    )

    # ✅ 캐시 무효화
    processed_url = (
        url_for("uploaded_file", token=token, filename="processed.png")
        + f"?v={int(time.time())}"
    )
    return jsonify({"ok": True, "processed_url": processed_url})



# if __name__ == "__main__":
#     app.run(host="0.0.0.0", port=5000, debug=True)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True, ssl_context="adhoc")