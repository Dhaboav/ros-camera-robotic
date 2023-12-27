import cv2 as cv
import numpy as np
import jetson_inference
import jetson_utils


# Setting env to load model
model_path = r'/home/nano/jetson-inference/python/training/detection/ssd/models/test2/ssd-mobilenet.onnx'
labels_path = r'/home/nano/jetson-inference/python/training/detection/ssd/models/test2/labels.txt'
threshold = 0.5

net = jetson_inference.detectNet(
    model=model_path, 
    labels=labels_path,
    input_blob='input_0',
    output_cvg='scores',
    output_bbox='boxes', 
    threshold=threshold
    )

camera = cv.VideoCapture(0)
camera.set(3, 640)
camera.set(4, 480)

while True:
    ret, frame = camera.read()
    if not ret:
        break
    
    # Apply ROI
    circle_mask = np.zeros_like(frame)
    cv.circle(img=circle_mask, center=(frame.shape[1]// 2, (frame.shape[0]// 2)-30), radius=230, color=(255, 255, 255), thickness=-1)
    circle_roi = cv.bitwise_and(frame, circle_mask)

    
    frame_cuda = jetson_utils.cudaFromNumpy(circle_roi)
    detections = net.Detect(frame_cuda)

    for info in detections:
        x1, y1, x2, y2, centeroid = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom), info.Center
        class_name = net.GetClassDesc(info.ClassID)

        if class_name == 'ROBOT':
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 255, 0), -1)
            cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)

        elif class_name == 'BOLA':
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
            cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 165, 255), 2)
            cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 0, 0), -1)

        elif class_name == 'PENGHALANG':
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 2)
            cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 0, 255), -1)

        elif class_name == 'GAWANG':
            cv.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 0, 0), 2)
            cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (255, 0, 0), -1)

    fps_text = "FPS: {:.0f}".format(net.GetNetworkFPS())
    cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

    cv.imshow("test", circle_roi)
    if cv.waitKey(1) & 0xFF == ord("x"):
        break

camera.release()
cv.destroyAllWindows()