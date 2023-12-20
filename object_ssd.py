import cv2 as cv
import jetson_inference
import jetson_utils

net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = cv.VideoCapture(0)
camera.set(3, 640)
camera.set(4, 480)

while True:
    ret, frame = camera.read()
    if not ret:
        break

    frame_cuda = jetson_utils.cudaFromNumpy(frame)
    detections = net.Detect(frame_cuda)

    for info in detections:
        x1, y1, x2, y2, centeroid = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom), info.Center
        class_name = net.GetClassDesc(info.ClassID)

        if class_name == 'person':
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.circle(frame, (int(centeroid[0]), int(centeroid[1])), 5, (0, 255, 0), -1)
            cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
        else:
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv.putText(frame, 'Unknown', (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 255), 2)

    fps_text = "FPS: {:.0f}".format(net.GetNetworkFPS())
    cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)

    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord("x"):
        break

camera.release()
cv.destroyAllWindows()