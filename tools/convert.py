import xml.etree.ElementTree as ET
import os


class ConvertXML2YOLO:
    def __init__(self, xml_labels_path, class_mapping):
        self.xml_folder = xml_labels_path
        self.class_mapping = class_mapping
        self.yolo_folder = os.path.join('runs', 'convert')
        self.folder_name = 'result'
        self.base_folder_path = self.check_folder()
    

    def check_folder(self):
        folder_path = os.path.join(self.yolo_folder, self.folder_name)
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            return folder_path

        count = 2
        while True:
            new_folder_name = f'{self.folder_name}{count}'
            new_folder_path = os.path.join(self.yolo_folder, new_folder_name)
            if not os.path.exists(new_folder_path):
                os.makedirs(new_folder_path)
                return new_folder_path
            count += 1


    def xml_to_yolo(self, xml_path, output_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()

        image_width = int(root.find('size/width').text)
        image_height = int(root.find('size/height').text)

        with open(output_path, 'w') as output_file:
            for obj in root.findall('object'):
                class_label = obj.find('name').text
                if class_label not in self.class_mapping:
                    continue

                xmin = int(obj.find('bndbox/xmin').text)
                ymin = int(obj.find('bndbox/ymin').text)
                xmax = int(obj.find('bndbox/xmax').text)
                ymax = int(obj.find('bndbox/ymax').text)

                x_center = (xmin + xmax) / (2.0 * image_width)
                y_center = (ymin + ymax) / (2.0 * image_height)
                width = (xmax - xmin) / image_width
                height = (ymax - ymin) / image_height

                output_file.write(f"{self.class_mapping[class_label]} {x_center} {y_center} {width} {height}\n")


    def run(self):
        count = 0
        for xml_file in os.listdir(self.xml_folder):
            if xml_file.endswith('.xml'):
                xml_path = os.path.join(self.xml_folder, xml_file)
                output_path = os.path.join(self.base_folder_path, xml_file.replace('.xml', '.txt'))
                self.xml_to_yolo(xml_path, output_path)
                count += 1
        print(f'Done convert {count} labels!')