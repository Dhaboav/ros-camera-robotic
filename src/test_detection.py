import cv2 as cv
import jetson.inference
import jetson.utils

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = cv.VideoCapture(0)
camera.set(3, 640)
camera.set(4, 480)

while True:
    ret, frame = camera.read()
    if not ret:
        break

    frame_cuda = jetson.utils.cudaFromNumpy(frame)
    detections = net.Detect(frame_cuda)

    for info in detections:
        x1, y1, x2, y2 = int(info.Left), int(info.Top), int(info.Right), int(info.Bottom)
        class_name = net.GetClassDesc(info.ClassID)
        cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.putText(frame, class_name, (x1, y1-10), cv.FONT_HERSHEY_PLAIN, 0.5, (0, 255, 0), 2)

    fps_text = "FPS: {:.0f}".format(net.GetNetworkFPS())
    cv.putText(frame, fps_text, (10, 20), cv.FONT_HERSHEY_PLAIN, 0.5, (0, 0, 0), 2)

    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord("x"):
        break

camera.release()
cv.destroyAllWindows()