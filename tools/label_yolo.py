from .label_xml import LabelCheckXML
import cv2 as cv
import os
import glob


class LabelCheckYOLO(LabelCheckXML):
    def yolo_to_opencv(self, x, y, width, height, image_width, image_height):
        x_min, y_min = int((x - width / 2) * image_width), int((y - height / 2) * image_height)
        x_max, y_max = int((x + width / 2) * image_width), int((y + height / 2) * image_height)
        return x_min, y_min, x_max, y_max
    

    def label_to_img(self, train=True):
        if train:
            folder_path = os.path.join(self.path, 'train', 'images', '*[jpn]*g')
        else:
            folder_path = os.path.join(self.path, 'val', 'images', '*[jpn]*g')
        
        self.total_img = 0
        for img_file in glob.glob(folder_path):
            label_file = img_file.replace('images', 'labels').replace(os.path.splitext(img_file)[1], '.txt')
            try:
                with open(label_file, "r") as file:
                    label_lines = file.readlines()
                    image = cv.imread(img_file)
                    self.total_img += 1
                    image_width, image_height = image.shape[1], image.shape[0]

                    for label_line in label_lines:
                        data = label_line.strip().split(" ")
                        class_id, x_coor, y_coor, width, height = (int(data[0]), float(data[1]), float(data[2]), float(data[3]),float(data[4]))
                        x_min, y_min, x_max, y_max = self.yolo_to_opencv(x_coor, y_coor, width, height, image_width, image_height)

                        # Draw rectangle with corresponding color
                        bounding_color = self.class_color[class_id]
                        cv.rectangle(image, (x_min, y_min), (x_max, y_max), bounding_color, 2)
                        self.class_count[class_id] += 1

                    self.save_img(image, img_file)
                    self.class_dist(self.total_img, self.classes_name, self.class_count)

            except FileNotFoundError:
                self.no_label(label_file)
                continue
