import os
import random

class TrainRatio:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.result_path = os.path.join('runs', 'label')
        self.folder_name = 'Main'
        self.folder_path = self.checking_folder_name()

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

    def save_to_file(self, data, file_name):
        save = os.path.join(self.folder_path)
        with open(os.path.join(save, file_name), 'w') as file:
            for item in data:
                file.write('%s\n' % item)
    # ==============================================================================

    # Core         
    def run(self):
        file_names = [os.path.splitext(f)[0] for f in os.listdir(self.dataset_path) if os.path.isfile(os.path.join(self.dataset_path, f))]
        random.shuffle(file_names)

        num_samples = len(file_names)
        num_train = int(num_samples * 0.8)
        num_val = int(num_samples * 0.1)

        # Split the file names into train, val, and test sets
        train_set = file_names[:num_train]
        val_set = file_names[num_train:num_train + num_val]
        test_set = file_names[num_train + num_val:]

        # Save the sets into separate text files
        self.save_to_file(train_set, 'train.txt')
        self.save_to_file(val_set, 'val.txt')
        self.save_to_file(test_set, 'test.txt')

        # Create trainval.txt by combining items from train.txt and val.txt
        trainval_set = train_set + val_set
        self.save_to_file(trainval_set, 'trainval.txt')

        # Printout
        print(f'Done split {len(file_names)} datas')