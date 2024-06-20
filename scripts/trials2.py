import os
import cv2

path_to_images = "/home/jostan/Downloads/jazz_dataset_yolov8/valid/images"
path_to_labels = "/home/jostan/Downloads/jazz_dataset_yolov8/valid/labels"
largest_width = 0
largest_height = 0

for filename in os.listdir(path_to_images):
    img = cv2.imread(os.path.join(path_to_images, filename))
    height, width, _ = img.shape
    if width > 640:
        path_img = os.path.join(path_to_images, filename)
        label_filename = filename[:-3] + 'txt'
        path_label = os.path.join(path_to_labels, label_filename)
        # remove the image
        os.remove(path_img)
        os.remove(path_label)

print(largest_width, largest_height)