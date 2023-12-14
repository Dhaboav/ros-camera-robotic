import cv2 as cv
import os
import time


class TakeData:
    def __init__(self, camera, width, height):
        self.camera = camera
        self.width = width
        self.height = height

        self.output = os.path.join('runs', 'take_data')
        self.folder_name = 'result'
        self.img_counter = 1
        self.base_folder_path = self.check_folder()
        
        self.interval = 5
        self.start_time = time.time()
        

    def check_folder(self):
        folder_path = os.path.join(self.output, self.folder_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            return folder_path

        count = 2
        while True:
            new_folder_name = f'{self.folder_name}{count}'
            new_folder_path = os.path.join(self.output, new_folder_name)
            if not os.path.exists(new_folder_path):
                os.makedirs(new_folder_path)
                return new_folder_path
            count += 1


    def save_data(self, img):
        file_name = f'image-{self.img_counter}.jpg'
        save_img = os.path.join(self.base_folder_path, file_name)
        cv.imwrite(save_img, img)
        self.img_counter += 1
        print(file_name)


    def run(self):
        cap = cv.VideoCapture(self.camera)
        cap.set(3, self.width)
        cap.set(4, self.height)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            cv.imshow('Take', frame)
            key = cv.waitKey(1) & 0xFF
            if key == ord('S') or (time.time() - self.start_time) > self.interval:
                self.save_data(frame)
                self.start_time = time.time()
            elif key == ord('x'):
                break

        cap.release()
        cv.destroyWindow('Take')
        print('Done!')