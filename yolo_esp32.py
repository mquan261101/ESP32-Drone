import cv2
from ultralytics import YOLO

# Tải mô hình 
model = YOLO("yolov8n.pt") 

# Khai báo địa chỉ luồng Stream
stream_url = "http://192.168.4.1:81/stream" 

# Kết nối VideoCapture
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Lỗi: Không thể kết nối đến luồng Stream của ESP32-CAM.")
    exit()

print("Bắt đầu nhận diện. Nhấn phím 'q' để thoát.")

while True:
    # Đọc từng khung hình từ luồng Wi-Fi
    success, frame = cap.read()
    
    if not success:
        print("Cảnh báo: Mất tín hiệu video.")
        break

    # Đưa khung hình qua mạng nơ-ron để tìm vật thể
    results = model(frame, stream=True, conf=0.5)

    # Duyệt qua các kết quả và trích xuất khung hình
    for r in results:
        annotated_frame = r.plot()

        # Hiển thị cửa sổ kết quả
        cv2.imshow("ESP32-CAM YOLO Detection", annotated_frame)

    # Thoát vòng lặp nếu nhấn 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()