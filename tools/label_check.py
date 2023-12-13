import cv2 as cv
import xml.etree.ElementTree as ET
import os
import glob


class LabelCheck:
    def __init__(self, dataset_path):
        self.path = dataset_path
        self.output = 'runs\label'
        self.folder_name = 'result'


    def classes (self, class_name, class_color):
        self.classes_name = class_name
        self.class_color = class_color


    def xml_format(self, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        size = root.find('size')
        img_width = int(size.find('width').text)
        img_height = int(size.find('height').text)
        boxes = []
        for obj in root.findall('object'):
            class_name = obj.find('name').text
            class_id = self.classes_name.index(class_name)
            box = obj.find('bndbox')
            x_min = int(box.find('xmin').text)
            y_min = int(box.find('ymin').text)
            x_max = int(box.find('xmax').text)
            y_max = int(box.find('ymax').text)
            boxes.append((class_id, x_min, y_min, x_max, y_max))
        return img_width, img_height, boxes


    def save_img(self, img, img_file):
        base_name = os.path.basename(img_file)
        save_path = os.path.join(self.output, self.folder_name)
        

        # Check if the folder exists, if not, create it
        if not os.path.exists(save_path):
            self.save_path = save_path
            os.makedirs(save_path)

        # Save the image in the result folder
        save = os.path.join(save_path, base_name)

        # Handle the case where the file already exists
        count = 2
        while os.path.exists(save):
            filename, extension = os.path.splitext(base_name)
            folder = f'{self.folder_name}{count}'
            save_path = os.path.join(self.output, folder)
            self.save_path = save_path
            os.makedirs(save_path, exist_ok=True)
            save = os.path.join(save_path, base_name)
            count += 1
        cv.imwrite(save, img)


    def no_label(self, file_name):
        log_file = os.path.join(self.save_path, f'{self.folder_name}_log.txt')
        with open(log_file, 'a') as file:
            file.write('\n' + file_name)


    def label_to_img(self, train=True):
        if train:
            folder_path = os.path.join(self.path, 'train', 'images', '*[jpn]*g')
        else:
            folder_path = folder_path = os.path.join(self.path, 'val', 'images', '*[jpn]*g')

        for img_file in glob.glob(folder_path):
            xml_file = img_file.replace('images', 'labels').replace(os.path.splitext(img_file)[1], '.xml')
            try:
                img_width, img_height, boxes = self.xml_format(xml_file)
                img = cv.imread(img_file)
                for box in boxes:
                    class_id, x_min, y_min, x_max, y_max = box
                    bounding_color = self.class_color[class_id]
                    cv.rectangle(img, (x_min, y_min), (x_max, y_max), bounding_color, 2)
                self.save_img(img, img_file)

            except FileNotFoundError:
                self.no_label(xml_file)
                continue