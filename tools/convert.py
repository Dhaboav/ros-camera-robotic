import xml.etree.ElementTree as ET
import os
import sys


class ConvertXML2YOLO:
    def __init__(self, xml_folder_path, class_mapping):
        self.xml_folder  = xml_folder_path
        self.classes     = class_mapping
        self.yolo_folder = os.path.join('runs', 'convert')
        self.folder_name = 'exp'
        self.folder_path = self.checking_folder_name()
    
    # Directory Stuff
    def checking_folder_name(self):
        folder = os.path.join(self.yolo_folder, self.folder_name)
        if not os.path.exists(folder):
            os.makedirs(folder)
            return folder

        folder_count = 2
        while True:
            new_folder_name = f'{self.folder_name}{folder_count}'
            folder = os.path.join(self.yolo_folder, new_folder_name)
            if not os.path.exists(folder):
                os.makedirs(folder)
                return folder
            folder_count += 1
    # ==============================================================================

    # Core
    def xml_to_yolo(self, xml, yolo):
        tree = ET.parse(xml)
        root = tree.getroot()
        image_width = int(root.find('size/width').text)
        image_height = int(root.find('size/height').text)

        with open(yolo, 'w') as write_yolo:
            for obj in root.findall('object'):
                class_name = obj.find('name').text
                if class_name not in self.classes:
                    continue

                xmin = int(obj.find('bndbox/xmin').text)
                ymin = int(obj.find('bndbox/ymin').text)
                xmax = int(obj.find('bndbox/xmax').text)
                ymax = int(obj.find('bndbox/ymax').text)

                # Yolo formating
                x_center = (xmin + xmax) / (2.0 * image_width)
                y_center = (ymin + ymax) / (2.0 * image_height)
                width = (xmax - xmin) / image_width
                height = (ymax - ymin) / image_height

                write_yolo.write(f'{self.classes[class_name]} {x_center} {y_center} {width} {height}\n')
    # ==============================================================================
                
    def run(self):
        progress_count = 0
        total_xml = len(os.listdir(self.xml_folder))
        for xml_file in os.listdir(self.xml_folder):
            if xml_file.endswith('.xml'):
                xml_path = os.path.join(self.xml_folder, xml_file)
                save = os.path.join(self.folder_path, xml_file.replace('.xml', '.txt'))
                self.xml_to_yolo(xml_path, save)

                # Print out progress bar
                progress_count += 1
                progress = int((progress_count / total_xml) * 40)
                sys.stdout.write('\r[' + '.' * progress + ' ' * (40 - progress) + f'] {progress_count}/{total_xml}')
                sys.stdout.flush()