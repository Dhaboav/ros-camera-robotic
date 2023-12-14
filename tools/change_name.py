import os


class ChangeName:
    def __init__(self, folder_path, name_format):
        self.folder_path = folder_path
        self.file_name_format = name_format
        self.counter = 1

    def run(self):
        for file in os.listdir(self.folder_path):
            old_file_path = os.path.join(self.folder_path, file)
            new_file_name = self.file_name_format.format(self.counter)
            new_file_path = os.path.join(self.folder_path, new_file_name)
            
            os.rename(old_file_path, new_file_path)
            self.counter += 1

        print(f'Selesai mengganti nama dari {self.counter-1} file')