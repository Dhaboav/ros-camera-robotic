from .label_xml import LabelCheckXML
import cv2 as cv
import os
import glob
import sys


class LabelCheckYOLO(LabelCheckXML):
    def yolo_to_opencv(self, x, y, width, height, image_width, image_height):
        x_min, y_min = int((x - width / 2) * image_width), int((y - height / 2) * image_height)
        x_max, y_max = int((x + width / 2) * image_width), int((y + height / 2) * image_height)
        return x_min, y_min, x_max, y_max
    
    def label_to_image(self, train=True):
        if train:
             data_category = os.path.join(self.dataset_path, 'train', 'images', '*[jpn]*g')
        else:
             data_category = os.path.join(self.dataset_path, 'val', 'images', '*[jpn]*g')
        
        self.total_image = len(glob.glob(data_category))
        for image in glob.glob( data_category):
            yolo_file = image.replace('images', 'labels').replace(os.path.splitext(image)[1], '.txt')
            try:
                with open(yolo_file, "r") as label:
                    label_lines = label.readlines()
                    read_image = cv.imread(image)
                    image_width, image_height = read_image.shape[1], read_image.shape[0]

                    # Print out progress bar
                    self.image_counter += 1
                    progress = int((self.image_counter / self.total_image) * 40)
                    sys.stdout.write('\r[' + '.' * progress + ' ' * (40 - progress) + f'] {self.image_counter}/{self.total_image}')
                    sys.stdout.flush()

                    for label_line in label_lines:
                        data = label_line.strip().split(" ")
                        class_id, x_coor, y_coor, width, height = (int(data[0]), float(data[1]), float(data[2]), float(data[3]),float(data[4]))
                        x_min, y_min, x_max, y_max = self.yolo_to_opencv(x_coor, y_coor, width, height, image_width, image_height)
                        bounding_color = self.class_colors[class_id]
                        cv.rectangle(read_image, (x_min, y_min), (x_max, y_max), bounding_color, 2)
                        self.class_counts[class_id] += 1

                    self.write_label_image(image, read_image)
                    self.class_plt(self.image_counter, self.class_names, self.class_counts)

            except FileNotFoundError:
                self.no_label(yolo_file)
                continue