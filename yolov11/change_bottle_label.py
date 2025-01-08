# import os

# # Define the folder path
# #folder_path = r"C:\Users\stu10\yolov11\train\labels"
# folder_path = r"C:\Users\stu10\yolov11\valid\labels"

# # Iterate through all files in the folder
# for filename in os.listdir(folder_path):
#     if filename.endswith(".txt"):
#         file_path = os.path.join(folder_path, filename)

#         # Read the content of the file
#         with open(file_path, "r") as file:
#             lines = file.readlines()

#         # Check and update the first character of the first line
#         if lines:
#             if lines[0].startswith("0"):
#                 lines[0] = lines[0].replace("0", "20", 1)
#             elif lines[0].startswith("1"):
#                 lines[0] = lines[0].replace("1", "21", 1)

#         # Write the updated content back to the file
#         with open(file_path, "w") as file:
#             file.writelines(lines)

# print("Update completed!")

import os

# Define the list of folder paths
folders = [
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\test2007",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\train_bottle",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\train2007",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\train2012",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\val2007",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\val2012",
    r"C:\Users\stu10\yolov11\bottle_datasets_3\labels\valid_bottle"
]

# Function to process files in a folder
def process_folder(folder_path):
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)

            # Read the content of the file
            with open(file_path, "r") as file:
                lines = file.readlines()

            # Update the content based on specified rules
            for i in range(len(lines)):
                if lines[i].startswith("4"):
                    lines[i] = lines[i].replace("4", "0", 1)
                elif lines[i].startswith("20"):
                    lines[i] = lines[i].replace("20", "1", 1)
                elif lines[i].startswith("21"):
                    lines[i] = lines[i].replace("21", "2", 1)

            # Write the updated content back to the file
            with open(file_path, "w") as file:
                file.writelines(lines)

# Process all folders
for folder in folders:
    if os.path.exists(folder):
        process_folder(folder)
        print(f"Processed folder: {folder}")
    else:
        print(f"Folder not found: {folder}")

print("Update completed!")
