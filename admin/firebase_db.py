import time
import firebase_admin
from firebase_admin import credentials, db
from config import SERVICE_ACCOUNT_KEY_PATH, DATABASE_URL

def init_firebase():
    """
    write_to_firebase.py / reat_from_firebase.py 방식 그대로:
    initialize_app 중복 초기화 방지
    """
    try:
        cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
        firebase_admin.initialize_app(cred, {"databaseURL": DATABASE_URL})
    except ValueError:
        # 이미 초기화됨
        pass

def artworks_ref():
    init_firebase()
    return db.reference("/artworks")

def create_artwork(title: str, description: str | None, image_filename: str | None):
    ref = artworks_ref()

    data = {
        "title": title,
        "description": description or "",
        "image_filename": image_filename or "",
        "created_at": time.time(),  # timestamp
    }

    # Realtime DB의 push() = 자동 key 생성
    new_ref = ref.push(data)
    return new_ref.key

def list_artworks(limit: int = 200):
    ref = artworks_ref()

    # dict 형태로 옴: {key: {...}, key2: {...}}
    raw = ref.get() or {}
    items = []
    for k, v in raw.items():
        v = v or {}
        items.append({
            "id": k,
            "title": v.get("title", ""),
            "description": v.get("description", ""),
            "image_filename": v.get("image_filename", ""),
            "created_at": v.get("created_at", 0),
        })

    # 최신순 정렬
    items.sort(key=lambda x: x["created_at"], reverse=True)
    return items[:limit]



def get_artwork(artwork_id: str):
    ref = artworks_ref().child(artwork_id)
    v = ref.get()
    if not v:
        return None

    return {
        "id": artwork_id,
        "title": v.get("title", ""),
        "description": v.get("description", ""),
        "image_filename": v.get("image_filename", ""),
        "created_at": v.get("created_at", 0),
    }

def delete_artwork(artwork_id: str):
    """Realtime DB에서 단일 작품 삭제"""
    init_firebase()
    artworks_ref().child(artwork_id).delete()


def update_artwork(artwork_id: str, data: dict):
    """
    data 예: {"title": "...", "description": "...", "image_filename": "..."}
    """
    init_firebase()
    artworks_ref().child(artwork_id).update(data)
