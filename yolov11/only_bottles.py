import os
import shutil

# current path
current_path = os.path.abspath(os.getcwd())

# current datasets path
dataset_path = current_path + '/datasets/VOC'

# Paths for images
image_paths = {
    'test2007': dataset_path + '/images/test2007',
    'train2007': dataset_path + '/images/train2007',
    'train2012': dataset_path + '/images/train2012',
    'val2007': dataset_path + '/images/val2007',
    'val2012': dataset_path + '/images/val2012',
}

# Paths for labels
label_paths = {
    'test2007': dataset_path + '/labels/test2007',
    'train2007': dataset_path + '/labels/train2007',
    'train2012': dataset_path + '/labels/train2012',
    'val2007': dataset_path + '/labels/val2007',
    'val2012': dataset_path + '/labels/val2012',
}

# New datasets path
bottle_datasets_path = current_path + '/bottle_datasets'

# Create new directories for images and labels
for set_name in image_paths:
    os.makedirs(os.path.join(bottle_datasets_path, f'images/{set_name}'), exist_ok=True)
    os.makedirs(os.path.join(bottle_datasets_path, f'labels/{set_name}'), exist_ok=True)

# Filter function to process each dataset
def filter_bottle_labels(set_name):
    labels_path = label_paths[set_name]
    imgs_path = image_paths[set_name]
    labels_save = os.path.join(bottle_datasets_path, f'labels/{set_name}')
    imgs_save = os.path.join(bottle_datasets_path, f'images/{set_name}')
    
    for file in os.listdir(labels_path):
        label_file_path = os.path.join(labels_path, file)
        new_label_lines = []
        
        with open(label_file_path, 'r') as f:
            for line in f.readlines():
                if line.split()[0] == '4':  # Assuming class 4 corresponds to 'bottle'
                    new_label_lines.append(line)

        # If there are lines containing 'bottle' label, save the label and image
        if new_label_lines:
            # Save the filtered label file
            new_label_file_path = os.path.join(labels_save, file)
            with open(new_label_file_path, 'w') as new_label_file:
                new_label_file.writelines(new_label_lines)
            
            # Copy the corresponding image file
            img_file_name = file.replace('.txt', '.jpg')  # Assuming images are .jpg
            img_file_path = os.path.join(imgs_path, img_file_name)
            new_img_file_path = os.path.join(imgs_save, img_file_name)

            if os.path.exists(img_file_path):
                shutil.copy(img_file_path, new_img_file_path)

# Apply filtering to all datasets
for set_name in image_paths.keys():
    filter_bottle_labels(set_name)

print("Filtering and copying completed.")
