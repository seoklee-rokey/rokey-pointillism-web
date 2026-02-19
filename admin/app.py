from flask import Flask, abort, render_template, request, redirect, url_for, session, flash, send_from_directory
import os
import time
from werkzeug.utils import secure_filename

from config import *
from firebase_db import create_artwork, list_artworks

from datetime import datetime

from firebase_db import *



app = Flask(__name__)
app.secret_key = SECRET_KEY
app.config["MAX_CONTENT_LENGTH"] = MAX_CONTENT_LENGTH

os.makedirs(UPLOAD_DIR, exist_ok=True)

# 시간 포멧 변환
def ts_to_str(ts):
    try:
        return datetime.fromtimestamp(float(ts)).strftime("%Y-%m-%d %H:%M:%S")
    except:
        return ""
    

@app.route("/artworks/<artwork_id>/edit", methods=["GET", "POST"])
def artwork_edit(artwork_id):
    if not require_login():
        return redirect(url_for("login"))

    art = get_artwork(artwork_id)
    if not art:
        abort(404)

    if request.method == "POST":
        title = (request.form.get("title") or "").strip()
        description = (request.form.get("description") or "").strip()

        if not title:
            flash("제목은 필수입니다.", "error")
            return redirect(url_for("artwork_edit", artwork_id=artwork_id))

        # 기존 이미지
        old_filename = (art.get("image_filename") or "").strip()
        new_filename = old_filename

        # 새 이미지 업로드가 있으면 교체
        f = request.files.get("image")
        if f and f.filename:
            # 확장자 체크(네 allowed_file 함수 그대로 사용)
            if not allowed_file(f.filename):
                flash("이미지는 png/jpg/jpeg/webp만 가능합니다.", "error")
                return redirect(url_for("artwork_edit", artwork_id=artwork_id))

            # 새 파일 저장
            candidate = make_upload_filename(f.filename, UPLOAD_DIR)
            f.save(os.path.join(UPLOAD_DIR, candidate))
            new_filename = candidate

            # 기존 파일 삭제(있으면)
            if old_filename:
                old_path = os.path.join(UPLOAD_DIR, old_filename)
                try:
                    if os.path.exists(old_path):
                        os.remove(old_path)
                except Exception as e:
                    print(f"[WARN] failed to delete old file: {old_path} ({e})")

        # Firebase 업데이트
        update_artwork(artwork_id, {
            "title": title,
            "description": description,
            "image_filename": new_filename,
            # created_at은 유지. 수정시간이 필요하면 updated_at 추가 추천.
        })

        return redirect(url_for("main_page"))

    # GET: 수정 폼 렌더
    return render_template("artwork_edit.html", art=art)


# 단일 삭제
@app.post("/artworks/<artwork_id>/delete")
def artwork_delete(artwork_id):
    if not require_login():
        return redirect(url_for("login"))

    art = get_artwork(artwork_id)
    if not art:
        abort(404)

    # 1) uploads 파일 삭제 (있으면)
    filename = (art.get("image_filename") or "").strip()
    if filename:
        path = os.path.join(UPLOAD_DIR, filename)
        try:
            if os.path.exists(path):
                os.remove(path)
        except Exception as e:
            # 파일 삭제 실패해도 DB 삭제는 진행 (운영이면 로그 남기는 게 좋음)
            print(f"[WARN] failed to delete file: {path} ({e})")

    # 2) Firebase DB 삭제
    delete_artwork(artwork_id)

    flash("삭제되었습니다.", "success")
    return redirect(url_for("main_page"))


# 단일 조회
@app.route("/artworks/<artwork_id>")
def artwork_detail(artwork_id):
    if not require_login():
        return redirect(url_for("login"))

    art = get_artwork(artwork_id)
    if not art:
        abort(404)

    # 생성 시간 포멧 변환
    art['created_at'] = ts_to_str(art['created_at'])

    return render_template("artwork_detail.html", art=art)



def split_name_ext(original_filename: str):
    # 경로 제거(브라우저에 따라 포함될 수 있음)
    base = os.path.basename(original_filename)
    name, ext = os.path.splitext(base)
    ext = ext.lower()
    return name, ext

def make_upload_filename(original_filename: str, upload_dir: str) -> str:
    """
    규칙:
    - 저장명: 원본파일명_YYMMDD.ext
    - 중복시: 원본파일명_YYMMDD(1).ext, (2)...
    예) 파이리.jpg -> 파이리_260217.jpg -> 파이리_260217(1).jpg
    """
    name, ext = split_name_ext(original_filename)
    date_tag = datetime.now().strftime("%y%m%d")  # 260217

    base = f"{name}_{date_tag}"
    candidate = f"{base}{ext}"

    i = 1
    while os.path.exists(os.path.join(upload_dir, candidate)):
        candidate = f"{base}({i}){ext}"
        i += 1

    return candidate


def allowed_file(filename: str) -> bool:
    if "." not in filename:
        return False
    ext = filename.rsplit(".", 1)[1].lower()
    return ext in ALLOWED_EXTS


def require_login() -> bool:
    return "username" in session


@app.route("/")
def home():
    if "username" in session:
        return redirect(url_for("main_page"))
    return redirect(url_for("login"))


@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        username = request.form.get("username", "")
        password = request.form.get("password", "")

        if username == USERNAME and password == PASSWORD:
            session["username"] = username
            return redirect(url_for("main_page"))
        else:
            flash("Invalid username or password", "error")
            return redirect(url_for("login"))

    return render_template("login.html")


@app.route("/logout")
def logout():
    session.pop("username", None)
    return redirect(url_for("login"))


@app.route("/uploads/<path:filename>")
def uploaded_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)


# ✅ 메인(작품 리스트)
@app.route("/main_page")
def main_page():
    if not require_login():
        return redirect(url_for("login"))

    artworks = list_artworks()
    return render_template("main_page.html", username=session["username"], artworks=artworks)


# ✅ 작품 추가 페이지
@app.route("/artworks/new", methods=["GET", "POST"])
def artwork_new():
    if not require_login():
        return redirect(url_for("login"))

    if request.method == "POST":
        title = (request.form.get("title") or "").strip()
        description = (request.form.get("description") or "").strip()

        if not title:
            flash("제목은 필수입니다.", "error")
            return redirect(url_for("artwork_new"))

        image_filename = ""
        f = request.files.get("image")

        if f and f.filename:
            if not allowed_file(f.filename):
                flash("이미지는 png/jpg/jpeg/webp만 가능합니다.", "error")
                return redirect(url_for("artwork_new"))

            original_name = f.filename  # 예: 파이리.jpg
            candidate = make_upload_filename(original_name, UPLOAD_DIR)
            f.save(os.path.join(UPLOAD_DIR, candidate))
            image_filename = candidate

        # ✅ Firebase Realtime DB에 저장
        create_artwork(title=title, description=description, image_filename=image_filename)

        flash("작품이 등록되었습니다.", "success")
        return redirect(url_for("main_page"))

    return render_template("artwork_new.html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5168, debug=True)
