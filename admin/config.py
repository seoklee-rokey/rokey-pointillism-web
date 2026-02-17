import os

SECRET_KEY = "your-secret-key"
USERNAME = "admin"
PASSWORD = "1234"

# ✅ Firebase Realtime Database
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

SERVICE_ACCOUNT_KEY_PATH = os.path.join(
    BASE_DIR,
    "rokey-c1fc2-firebase-adminsdk-fbsvc-5ba5f78dcd.json"
)

DATABASE_URL = "https://rokey-c1fc2-default-rtdb.asia-southeast1.firebasedatabase.app" 


# ✅ 업로드(로컬)
UPLOAD_DIR = "./uploads"
ALLOWED_EXTS = {"png", "jpg", "jpeg", "webp"}
MAX_CONTENT_LENGTH = 10 * 1024 * 1024  # 10MB
