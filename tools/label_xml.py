import cv2 as cv
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import os
import glob
import sys


class LabelCheckXML:
    def __init__(self, dataset_path):
        self.dataset_path   = dataset_path
        self.result_path    = os.path.join('runs', 'label')
        self.folder_name    = 'exp' 
        self.folder_path    = self.checking_folder_name()
        self.image_counter  = 0

    # Directory Stuff
    def checking_folder_name(self):
        folder = os.path.join(self.result_path, self.folder_name)
        if not os.path.exists(folder):
            os.makedirs(folder)
            return folder

        folder_count = 2
        while True:
            new_folder_name = f'{self.folder_name}{folder_count}'
            folder = os.path.join(self.result_path, new_folder_name)
            if not os.path.exists(folder):
                os.makedirs(folder)
                return folder
            folder_count += 1

    def write_label_image(self, image, result_image):
        image_path = os.path.join(self.folder_path, os.path.basename(image))
        cv.imwrite(image_path, result_image)

    def no_label(self, missing_label):
        log_file = os.path.join(self.folder_path, 'NoLabel.txt')
        with open(log_file, 'a') as file:
            file.write('\n' + missing_label)
    # ==============================================================================

    # Class Stuff     
    def class_data(self, class_name, class_color, class_count):
        self.class_names    = class_name
        self.class_colors   = class_color
        self.class_counts   = class_count

    def class_plt(self, total, class_name, class_number):
        plt.title(f"Class Distribution of {total} Images")
        plt.bar(class_name, class_number)
        plt_file = os.path.join(self.folder_path, 'PlotClass')
        plt.savefig(plt_file)
    # ==============================================================================

    # Core
    def read_xml(self, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        size = root.find('size')
        image_width = int(size.find('width').text)
        image_height = int(size.find('height').text)
        boxes = []
        for obj in root.findall('object'):
            name_of_class = obj.find('name').text
            id_of_class = self.class_names.index(name_of_class)
            box = obj.find('bndbox')
            x_min = int(box.find('xmin').text)
            y_min = int(box.find('ymin').text)
            x_max = int(box.find('xmax').text)
            y_max = int(box.find('ymax').text)
            boxes.append((id_of_class, x_min, y_min, x_max, y_max))

        return image_width, image_height, boxes

    def label_to_image(self, train=True):
        if train:
            data_category = os.path.join(self.dataset_path, 'train', 'images', '*[jpn]*g')
        else:
            data_category = os.path.join(self.dataset_path, 'val', 'images', '*[jpn]*g')
        
        self.total_image = len(glob.glob(data_category))
        for image in glob.glob(data_category):
            xml_file = image.replace('images', 'labels').replace(os.path.splitext(image)[1], '.xml')
            try:
                image_width, image_height, boxes = self.read_xml(xml_file)
                read_image = cv.imread(image)
                
                # Print out progress bar
                self.image_counter += 1
                progress = int((self.image_counter / self.total_image) * 40)
                sys.stdout.write('\r[' + '.' * progress + ' ' * (40 - progress) + f'] {self.image_counter}/{self.total_image}')
                sys.stdout.flush()

                # Read info from label
                for box in boxes:
                    class_id, x_min, y_min, x_max, y_max = box
                    bounding_color = self.class_colors[class_id]
                    cv.rectangle(read_image, (x_min, y_min), (x_max, y_max), bounding_color, 2)
                    self.class_counts[class_id] += 1

                self.write_label_image(image, read_image)
                self.class_plt(self.image_counter, self.class_names, self.class_counts)

            except FileNotFoundError:
                self.no_label(xml_file)
                continue
    # ==============================================================================

    def run(self, train_boolean, class_name, class_color, class_counter):
        self.class_data(class_name, class_color, class_counter)
        self.label_to_image(train_boolean)