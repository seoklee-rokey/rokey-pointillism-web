// ===============================
// 모달 제어
// ===============================
const openBtn = document.getElementById("openDialogBtn");
const modal = document.getElementById("photoModal");
const backdrop = document.getElementById("modalBackdrop");
const closeBtn = document.getElementById("closeModalBtn");

function openModal() {
  modal.classList.remove("hidden");
  backdrop.classList.remove("hidden");
}

function closeModal() {
  modal.classList.add("hidden");
  backdrop.classList.add("hidden");
}

openBtn.addEventListener("click", openModal);
closeBtn.addEventListener("click", closeModal);
backdrop.addEventListener("click", closeModal);

// ESC 키로 모달 닫기
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape") closeModal();
});


// ===============================
// 3개 버튼 → 각 페이지로 이동
// ===============================

// 📷 사진 촬영하기
document.getElementById("btnCapture").addEventListener("click", () => {
  window.location.href = "/camera";
});

// 📱 사진 업로드하기 (QR)
document.getElementById("btnUpload").addEventListener("click", () => {
  window.location.href = "/qr";
});

// 🖼 전시관 사진 불러오기
document.getElementById("btnGallery").addEventListener("click", () => {
  window.location.href = "/gallery";
});
