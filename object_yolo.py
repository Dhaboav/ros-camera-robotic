import cv2 as cv
import torch
import time
import numpy as np
# from vision.field import field_color, field_contour, field_display

yolo_path = r''
model_path = r''

# # Load the YOLOv5 model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = torch.hub.load(yolo_path, 'custom', path=model_path, source="local").to(device)

camera = cv.VideoCapture(1, cv.CAP_DSHOW) 
camera_size = (640, 480)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, camera_size[0])
camera.set(cv.CAP_PROP_FRAME_WIDTH, camera_size[1])


# used to record the time when we processed last frame 
prev_frame_time = 0
  
# used to record the time at which we processed current frame 
new_frame_time = 0

# green_lower = np.array([48, 25, 0])
# green_upper = np.array([86, 255, 202])

while camera.isOpened():
    _, frame = camera.read()
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    output = frame.copy()
    # color_field = field_color(frame_hsv, green_upper, green_lower)
    # color_morph = field_contour(color_field, copy_frame)
    # output = field_display(copy_frame, color_morph)

    
    # Perform object detection
    detections = model(frame[..., ::-1])

    # Extract and draw bounding boxes
    results = detections.pandas().xyxy[0].to_dict(orient="records")
    for result in results:
        con, cs, x1, y1, x2, y2 = result["confidence"], result["class"], int(result['xmin']), int(result['ymin']), int(result['xmax']), int(result['ymax'])
        # Get class name based on the class index and convert confidence to percentage
        class_name = model.names[int(cs)]
        percentage = con * 100

        #centeroid
        mid_x = (x1 + x2)/2
        mid_y = (y1 + y2)/2

        if class_name == "ROBOT":
            cv.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.circle(output, (int(mid_x), int(mid_y)), 10, (255,255,255), -1)
            cv.putText(output, f"{class_name} {percentage:.0f} %", (x1, y1-10), cv.FONT_HERSHEY_PLAIN,
                    1.0, (0, 255, 0), 2)
            
        elif class_name == "BOLA":
            cv.rectangle(output, (x1, y1), (x2, y2), (0, 140, 255), 2)
            cv.circle(output, (int(mid_x), int(mid_y)), 2, (255,164,0), -1)
            cv.putText(output, f"{class_name} {percentage:.0f} %", (x1, y1-10), cv.FONT_HERSHEY_PLAIN,
                    1.0, (0, 140, 255), 2)
            print(f'CBola: {int(mid_x)}, {int(mid_y)}')
            
        elif class_name == "PENGHALANG":
            cv.rectangle(output, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv.circle(output, (int(mid_x), int(mid_y)), 10, (0,0,0), -1)
            cv.putText(output, f"{class_name} {percentage:.0f} %", (x1, y1-10), cv.FONT_HERSHEY_PLAIN,
                    1.0, (255, 0, 0), 2)
            
        elif class_name == "GAWANG":
            cv.rectangle(output, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv.circle(output, (int(mid_x), int(mid_y)), 10, (255,255,255), -1)
            cv.putText(output, f"{class_name} {percentage:.0f} %", (x1, y1-10), cv.FONT_HERSHEY_PLAIN,
                    1.0, (255, 0, 0), 2)
        else:
            pass
            
    # time when we finish processing for this frame 
    new_frame_time = time.time() 
    fps = 1/(new_frame_time-prev_frame_time) 
    prev_frame_time = new_frame_time
    cv.putText(output, f"FPS: {fps:.0f}", (10,30), cv.FONT_HERSHEY_COMPLEX,
               1, (255, 255, 255), 2)
    
    # Display the image with bounding boxes
    cv.imshow("Object Detection", output)

    # Press 'q' to exit the loop
    if cv.waitKey(1) & 0xFF == ord("x"):
        break

camera.release()
cv.destroyWindow("Object Detection")
