#app.py
import os
import sys
import json
import secrets
import shutil
import time
import random
import requests
import qrcode

from pathlib import Path
from PIL import Image, ImageOps
from flask import Flask, render_template, request, redirect, url_for, send_from_directory, jsonify, abort

import draw_stipple
#from preview_utils import save_preview_from_points
from preview_utils import save_preview_robot_style

app = Flask(__name__)
print("### APP.PY SIGNATURE: 2026-02-23 00:xx ###")
# -------------------------------------------------
# Path / Dir
# -------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent          # .../client
UPLOAD_DIR = BASE_DIR / "uploads"                  # .../client/uploads
QR_DIR = BASE_DIR / "static" / "qr"                # .../client/static/qr

# ✅ admin/uploads (client 기준 상위 폴더의 admin/uploads)
ADMIN_DIR = (BASE_DIR / "../admin").resolve()
GALLERY_DIR = (ADMIN_DIR / "uploads").resolve()

UPLOAD_DIR.mkdir(exist_ok=True)
QR_DIR.mkdir(parents=True, exist_ok=True)
GALLERY_DIR.mkdir(parents=True, exist_ok=True)

ALLOWED_EXT = {"png", "jpg", "jpeg", "webp"}

# -------------------------------------------------
# ✅ DB(Firebase) 모듈 import (admin 폴더에서 가져옴)
# -------------------------------------------------
if str(ADMIN_DIR) not in sys.path:
    sys.path.insert(0, str(ADMIN_DIR))

from firebase_db import list_artworks, get_artwork  # noqa: E402

# -------------------------------------------------
# Utils
# -------------------------------------------------
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

# -----------------------------
# ✅ token별 마지막 옵션 저장/로드
# -----------------------------
DEFAULT_PARAMS = {"edge_prob": 0.5, "inner_density": 0.01, "color_mode": "bw","remove_background": False,"canny_low": 100,"canny_high": 250,}

def params_path(token: str) -> Path:
    return token_dir(token) / "params.json"

def load_params(token: str) -> dict:
    p = params_path(token)
    if p.exists():
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            # 누락 키 기본값 보완
            return {
                "edge_prob": float(d.get("edge_prob", DEFAULT_PARAMS["edge_prob"])),
                "inner_density": float(d.get("inner_density", DEFAULT_PARAMS["inner_density"])),
                "color_mode": d.get("color_mode", DEFAULT_PARAMS["color_mode"]),
                "remove_background": bool(d.get("remove_background", DEFAULT_PARAMS["remove_background"])),
                "canny_low": int(d.get("canny_low", DEFAULT_PARAMS["canny_low"])),
                "canny_high": int(d.get("canny_high", DEFAULT_PARAMS["canny_high"])),
            }
        except Exception:
            pass
    return dict(DEFAULT_PARAMS)

def save_params(token: str, edge_prob: float, inner_density: float, color_mode: str, remove_background: bool, canny_low: int, canny_high: int):
    p = params_path(token)
    p.write_text(
        json.dumps(
            {"edge_prob": edge_prob, "inner_density": inner_density, "color_mode": color_mode,"remove_background": remove_background,"canny_low": canny_low,"canny_high": canny_high,},
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

# -----------------------------
# ✅ draw_stipple 결과를 “항상 재현 가능”하게 만드는 seed
# (미리보기 생성할 때와 로봇 실행할 때 점이 달라지면 안 되니까)
# -----------------------------
def _stable_seed(token: str, edge_prob: float, inner_density: float, color_mode: str, remove_background: bool, canny_low: int, canny_high: int) -> int:
    key = f"{token}|{edge_prob:.3f}|{inner_density:.3f}|{color_mode}|{int(remove_background)}|{canny_low}|{canny_high}"
    return (hash(key) & 0xFFFFFFFF)

def build_preview_and_robot_points(
    token: str,
    original_path: Path,
    edge_prob: float,
    inner_density: float,
    color_mode: str,
    processed_path: Path,
    max_size: int = 400,
    canny_low: int = 100,
    canny_high: int = 250,
    remove_background: bool = False,   # 🔥 추가
):
    seed = _stable_seed(token, edge_prob, inner_density, color_mode, remove_background, canny_low, canny_high)

    state = random.getstate()
    random.seed(seed)
    try:
        # 회전여부까지 가져옴
        points_px, w, h, rotated = draw_stipple.generate_stipple_points(
            str(original_path),
            edge_prob=edge_prob,
            inner_density=inner_density,
            color_mode=("color" if color_mode == "color" else "bw"),
            max_size=max_size,
            canny_low=canny_low,
            canny_high=canny_high,
            remove_background=remove_background,   # 🔥 추가
        )
    finally:
        random.setstate(state)

    # ✅ 프리뷰는 점 재생성 없이 그대로 그림
    #save_preview_from_points(points_px, w, h, processed_path)#, radius=3)
    save_preview_robot_style(points_px, w, h, processed_path, pad_to_original_aspect=True)

    # ✅ 로봇 좌표 변환(딱 1번)
    robot_points = draw_stipple.convert_to_robot_coords(points_px, w, h)
    # 회전 여부도 리턴함
    return robot_points, rotated

def ensure_processed_exists(token: str) -> tuple[Path, Path]:
    """원본/processed 경로를 준비하고, processed가 없으면 params 기준으로 생성"""
    token_path = token_dir(token)
    originals = list(token_path.glob("original.*"))
    if not originals:
        abort(404, "No original image")

    original = originals[0]
    processed_path = token_path / "processed.png"

    if not processed_path.exists():
        params = load_params(token)
        build_preview_and_robot_points(
            token=token,
            original_path=original,
            edge_prob=float(params["edge_prob"]),
            inner_density=float(params["inner_density"]),
            color_mode=params["color_mode"],
            processed_path=processed_path,
            max_size=400,

            # ✅ 추가
            remove_background=bool(params.get("remove_background", False)),
            canny_low=int(params.get("canny_low", 100)),
            canny_high=int(params.get("canny_high", 250)),
        )

    return original, processed_path

# -------------------------------------------------
# Static serving: admin/uploads 를 /gallery/<filename> 으로 제공
# -------------------------------------------------
@app.route("/gallery/<path:filename>")
def gallery_file(filename):
    return send_from_directory(GALLERY_DIR, filename)

@app.route("/")
def index():
    return render_template("index.html")

# -------------------------
# 1) 사진 업로드하기 (QR)
# -------------------------
@app.route("/qr")
def qr_page():
    token = new_token()
    upload_url = url_for("phone_upload_page", token=token, _external=True)

    qr_img_path = QR_DIR / f"{token}.png"
    img = qrcode.make(upload_url)
    img.save(qr_img_path)

    return render_template(
        "qr.html",
        token=token,
        upload_url=upload_url,
        qr_img=url_for("static", filename=f"qr/{token}.png"),
    )

@app.route("/u/<token>", methods=["GET"])
def phone_upload_page(token):
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

    # ✅ 업로드 직후: params 기준으로 1회 프리뷰 생성(없으면 기본값)
    params = load_params(token)
    processed_path = token_path / "processed.png"
    build_preview_and_robot_points(
        token=token,
        original_path=original_path,
        edge_prob=float(params["edge_prob"]),
        inner_density=float(params["inner_density"]),
        color_mode=params["color_mode"],
        processed_path=processed_path,
        max_size=400,
        remove_background=bool(params.get("remove_background", False)),
        canny_low=int(params.get("canny_low", 100)),
        canny_high=int(params.get("canny_high", 250)),
    )

    return redirect(url_for("upload_done_page", token=token))

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
    original_path = token_path / "original.png"
    f.save(original_path)

    params = load_params(token)
    processed_path = token_path / "processed.png"
    build_preview_and_robot_points(
        token=token,
        original_path=original_path,
        edge_prob=float(params["edge_prob"]),
        inner_density=float(params["inner_density"]),
        color_mode=params["color_mode"],
        processed_path=processed_path,
        max_size=400,
        remove_background=bool(params.get("remove_background", False)),
        canny_low=int(params.get("canny_low", 100)),
        canny_high=int(params.get("canny_high", 250)),
    )

    return jsonify({"ok": True, "next": url_for("customize_page", token=token)})

# -------------------------
# 3) 전시관 사진 불러오기 (DB + admin/uploads)
# -------------------------
@app.route("/gallery")
def gallery_page():
    token = new_token()
    token_dir(token)

    artworks = list_artworks(limit=500)

    items = []
    for a in artworks:
        filename = (a.get("image_filename") or "").strip()
        if not filename:
            continue
        if not allowed(filename):
            continue
        if not (GALLERY_DIR / filename).exists():
            continue
        items.append({
            "id": a.get("id"),
            "title": a.get("title") or filename,
            "image_filename": filename,
            "created_at": a.get("created_at", 0),
        })

    return render_template("gallery.html", token=token, items=items)

@app.route("/select_gallery/<token>", methods=["POST"])
def select_gallery(token):
    token_path = token_dir(token)

    artwork_id = (request.form.get("artwork_id") or "").strip()
    if not artwork_id:
        abort(400, "No artwork_id")

    art = get_artwork(artwork_id)
    if not art:
        abort(404, "Artwork not found")

    filename = (art.get("image_filename") or "").strip()
    if not filename:
        abort(404, "No image for this artwork")

    src = GALLERY_DIR / filename
    if not src.exists():
        abort(404, "File not found in admin/uploads")

    ext = src.suffix.lower().lstrip(".")
    original_path = token_path / f"original.{ext}"
    shutil.copyfile(src, original_path)

    params = load_params(token)
    processed_path = token_path / "processed.png"
    build_preview_and_robot_points(
        token=token,
        original_path=original_path,
        edge_prob=float(params["edge_prob"]),
        inner_density=float(params["inner_density"]),
        color_mode=params["color_mode"],
        processed_path=processed_path,
        max_size=400,
        remove_background=bool(params.get("remove_background", False)),
        canny_low=int(params.get("canny_low", 100)),
        canny_high=int(params.get("canny_high", 250)),
    )

    return redirect(url_for("customize_page", token=token))

# -------------------------
# 커스터마이징(로봇 작동 전) 페이지
# -------------------------
@app.route("/customize/<token>")
def customize_page(token):
    original, processed_path = ensure_processed_exists(token)

    # 회전 적용 
    # ✅ 현재 옵션 로드
    params = load_params(token)

    # ✅ rotated 얻기 (로봇 좌표는 안 쓰고 rotated만 씀)
    _robot_points, rotated = build_preview_and_robot_points(
        token=token,
        original_path=original,
        edge_prob=float(params["edge_prob"]),
        inner_density=float(params["inner_density"]),
        color_mode=params["color_mode"],
        processed_path=processed_path,
        max_size=400,
        remove_background=bool(params.get("remove_background", False)),
        canny_low=int(params.get("canny_low", 100)),
        canny_high=int(params.get("canny_high", 250)),
    )

    original_url = url_for("uploaded_file", token=token, filename=original.name)
    processed_url = url_for("uploaded_file", token=token, filename="processed.png") + f"?v={int(time.time())}"

    # (선택) 페이지 로드시 현재 params를 넘기고 싶으면 여기서 같이 넘겨도 됨
    # params = load_params(token)

    return render_template(
        "customize.html",
        token=token,
        original_url=original_url,
        processed_url=processed_url,
        # params=params,
        rotated=rotated,   # ✅ 추가
    )

@app.route("/uploads/<token>/<path:filename>")
def uploaded_file(token, filename):
    return send_from_directory(UPLOAD_DIR / token, filename)

@app.route("/check_upload/<token>")
def check_upload(token):
    token_path = token_dir(token)
    originals = list(token_path.glob("original.*"))
    return {"uploaded": len(originals) > 0}

# -------------------------
# ✅ 옵션 변경 시 처리(프리뷰 갱신)
# -------------------------
@app.route("/process/<token>", methods=["POST"])
def process_image(token):
    token_path = token_dir(token)

    originals = list(token_path.glob("original.*"))
    if not originals:
        abort(404, "No original image")

    data = request.get_json(silent=True) or {}

    edge_prob = float(data.get("edge_prob", DEFAULT_PARAMS["edge_prob"]))
    inner_density = float(data.get("inner_density", DEFAULT_PARAMS["inner_density"]))
    color_mode = data.get("color_mode", DEFAULT_PARAMS["color_mode"])

    # ✅ 추가: 배경제거 + canny
    remove_background = bool(data.get("remove_background", DEFAULT_PARAMS["remove_background"]))
    canny_low = int(data.get("canny_low", DEFAULT_PARAMS["canny_low"]))
    canny_high = int(data.get("canny_high", DEFAULT_PARAMS["canny_high"]))

    if canny_low >= canny_high:
        canny_high = min(300, canny_low + 1)

    # ✅ 마지막 설정 저장(계속 갱신)
    save_params(
        token,
        edge_prob=edge_prob,
        inner_density=inner_density,
        color_mode=color_mode,
        remove_background=remove_background,
        canny_low=canny_low,
        canny_high=canny_high,
    )

    original = originals[0]
    processed_path = token_path / "processed.png"

    # build_preview_and_robot_points(
    #     token=token,
    #     original_path=original,
    #     edge_prob=edge_prob,
    #     inner_density=inner_density,
    #     color_mode=color_mode,
    #     processed_path=processed_path,
    #     max_size=400,

    #     # ✅ 추가 전달
    #     remove_background=remove_background,
    #     canny_low=canny_low,
    #     canny_high=canny_high,
    # )

    # processed_url = url_for("uploaded_file", token=token, filename="processed.png") + f"?v={int(time.time())}"
    # return jsonify({"ok": True, "processed_url": processed_url})

    # 회전 여부 적용
    robot_points, rotated = build_preview_and_robot_points(
        token=token,
        original_path=original,
        edge_prob=edge_prob,
        inner_density=inner_density,
        color_mode=color_mode,
        processed_path=processed_path,
        max_size=400,
        remove_background=remove_background,
        canny_low=canny_low,
        canny_high=canny_high,
    )

    processed_url = url_for("uploaded_file", token=token, filename="processed.png") + f"?v={int(time.time())}"
    return jsonify({"ok": True, "processed_url": processed_url, "rotated": rotated})

# -------------------------
# 로봇 실행/상태/취소
# -------------------------
#JOB_TOKEN_MAP = {}
JOB_META = {}
ROBOT_BRIDGE_URL = "http://127.0.0.1:8089"

@app.post("/api/robot/run/<token>")
def api_robot_run(token):
    token_path = token_dir(token)

    originals = list(token_path.glob("original.*"))
    if not originals:
        abort(404, "No original image")

    data = request.get_json(silent=True) or {}

    saved = load_params(token)

    edge_prob = float(data.get("edge_prob", saved["edge_prob"]))
    inner_density = float(data.get("inner_density", saved["inner_density"]))
    color_mode = data.get("color_mode", saved["color_mode"])

    # ✅ 추가: 배경 제거 + canny low/high도 run에서 동일하게 적용
    remove_background = bool(data.get("remove_background", saved.get("remove_background", False)))
    canny_low = int(data.get("canny_low", saved.get("canny_low", 100)))
    canny_high = int(data.get("canny_high", saved.get("canny_high", 250)))

    # ✅ 안전장치 (프론트에서도 했겠지만 서버에서도 한 번 더)
    if canny_low >= canny_high:
        canny_high = min(300, canny_low + 1)

    # ✅ run 눌렀을 때도 “최종값” 저장 (save_params 시그니처도 확장돼 있어야 함)
    save_params(
        token,
        edge_prob=edge_prob,
        inner_density=inner_density,
        color_mode=color_mode,
        remove_background=remove_background,
        canny_low=canny_low,
        canny_high=canny_high,
    )

    original = originals[0]
    processed_path = token_path / "processed.png"

    # ✅ 프리뷰와 동일 로직(동일 점) + 동일 파라미터 전달
    # 회전 여부도 받아오지만 사용은 안함
    robot_points, _rotated = build_preview_and_robot_points(
        token=token,
        original_path=original,
        edge_prob=edge_prob,
        inner_density=inner_density,
        color_mode=color_mode,
        processed_path=processed_path,
        max_size=400,

        # ✅ 추가
        remove_background=remove_background,
        canny_low=canny_low,
        canny_high=canny_high,
    )

    try:
        r = requests.post(
            f"{ROBOT_BRIDGE_URL}/run",
            json={
                "token": token,
                "dots": robot_points,
                "color_mode": color_mode,
            },
            timeout=10.0
        )
        r.raise_for_status()
        payload = r.json()
    except Exception as e:
        return jsonify({"ok": False, "error": f"Bridge error: {e}"}), 502
    
    # job_id = payload.get("job_id")

    # if job_id:
    #     JOB_TOKEN_MAP[job_id] = token

    job_id = payload.get("job_id")
    if job_id:
        JOB_META[job_id] = {"token": token, "rotated": bool(_rotated)}

    return jsonify({
        "ok": True,
        "sent": len(robot_points),
        "job_id": job_id
    })

    # return jsonify({
    #     "ok": True,
    #     "sent": len(robot_points),
    #     "job_id": payload.get("job_id")
    # })

@app.get("/upload_done/<token>")
def upload_done_page(token):
    return render_template("upload_done.html", token=token)

@app.get("/api/robot/status/<job_id>")
def api_robot_status(job_id):
    try:
        r = requests.get(f"{ROBOT_BRIDGE_URL}/status/{job_id}", timeout=5.0)
        r.raise_for_status()
        payload = r.json()
    except Exception as e:
        return jsonify({"ok": False, "error": f"Bridge error: {e}"}), 502
    return jsonify(payload)

# @app.get("/robot/run/<job_id>")
# def robot_run_page(job_id):
#     token = JOB_TOKEN_MAP.get(job_id)
#     return render_template("robot_run.html", job_id=job_id, token=token)

@app.get("/robot/run/<job_id>")
def robot_run_page(job_id):
    meta = JOB_META.get(job_id, {})
    return render_template(
        "robot_run.html",
        job_id=job_id,
        token=meta.get("token"),
        rotated=meta.get("rotated", False),
    )

@app.get("/robot/done/<job_id>")
def robot_done(job_id):
    return render_template("robot_done.html", job_id=job_id)


# ✅ 여기 추가!
@app.post("/api/robot/resume/<job_id>")
def api_robot_resume(job_id):
    try:
        r = requests.post(
            f"{ROBOT_BRIDGE_URL}/resume",
            json={"job_id": job_id},
            timeout=5.0
        )
        r.raise_for_status()
        return jsonify(r.json())
    except Exception as e:
        return jsonify({"ok": False, "error": f"Bridge error: {e}"}), 502

@app.post("/api/robot/cancel/<job_id>")
def api_robot_cancel(job_id):
    try:
        r = requests.post(f"{ROBOT_BRIDGE_URL}/cancel", json={"job_id": job_id}, timeout=5.0)
        r.raise_for_status()
        return jsonify(r.json())
    except Exception as e:
        return jsonify({"ok": False, "error": f"Bridge error: {e}"}), 502

if __name__ == "__main__":
    # 네가 원래 쓰던 포트/SSL 유지
    app.run(host="0.0.0.0", port=5169, debug=True, ssl_context="adhoc")