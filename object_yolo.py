import cv2 as cv
import torch
import time
import numpy as np

yolo_path = r''
model_path = r'robot5s.pt'

# Load the YOLOv5 model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = torch.hub.load(yolo_path, 'custom', path=model_path, source="local").to(device)

camera = cv.VideoCapture(0, cv.CAP_DSHOW) 
camera_size = (640, 480)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, camera_size[0])
camera.set(cv.CAP_PROP_FRAME_WIDTH, camera_size[1])

# fps counter
prev_frame = 0
new_frame = 0

while camera.isOpened():
    ret, frame = camera.read()
    if not ret:
        break

    # Apply ROI
    circle_mask = np.zeros_like(frame)
    cv.circle(img=circle_mask, center=(frame.shape[1]// 2, (frame.shape[0]// 2)-30), radius=230, color=(255, 255, 255), thickness=-1)
    circle_roi = cv.bitwise_and(frame, circle_mask)
    output = circle_roi.copy()

    # Model detection
    detections = model(circle_roi[..., ::-1])
    detect = detections.pandas().xyxy[0].to_dict(orient="records")
    for info in detect:
        id_class, x1, y1, x2, y2 = info["class"], int(info['xmin']), int(info['ymin']), int(info['xmax']), int(info['ymax'])
        class_name = model.names[id_class]

        #centeroid
        mid_x = (x1 + x2)/2
        mid_y = (y1 + y2)/2

        if class_name == "ROBOT":
            cv.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
        elif class_name == "BOLA":
            cv.rectangle(output, (x1, y1), (x2, y2), (0, 140, 255), 2)
            cv.circle(output, (int(mid_x), int(mid_y)), 2, (255,164,0), -1)
            print(f'CBola: {int(mid_x)}, {int(mid_y)}')
            
        elif class_name == "PENGHALANG":
            cv.rectangle(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
        elif class_name == "GAWANG":
            cv.rectangle(output, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # FPS calculation
    new_frame = time.time() 
    fps = 1/(new_frame-prev_frame) 
    prev_frame = new_frame
    cv.putText(output, f"FPS: {fps:.0f}", (10,30), cv.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)


    cv.imshow("Object Detection", output)    
    if cv.waitKey(1) & 0xFF == ord("x"):
        break

camera.release()
cv.destroyAllWindows()