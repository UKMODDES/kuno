import numpy as np
import imutils
import cv2
import os

lower_red_1 = np.array([0, 110, 0])
upper_red_1 = np.array([10, 255, 255])

lower_red_2 = np.array([160, 110, 0])
upper_red_2 = np.array([180, 255, 255])

lower_yellow = np.array([10, 110, 0])
upper_yellow = np.array([32,255,255])

lower_green = np.array([32, 110, 0])
upper_green = np.array([78, 255, 255])

# lower_blue = np.array([199, 85.3, 100])
# upper_blue = np.array([203, 100, 73.3])

lower_blue = np.array([80, 110, 0])
upper_blue = np.array([110, 255, 255])

lower_purple = np.array([143, 80, 0])
upper_purple = np.array([160, 255, 255])

class Shape_Info:
    def __init__(self, cX, cY, contour, colour):
        self.cX = cX
        self.cY = cY
        self.contour = contour
        self.colour = colour
        self.shape_type = None

    def add_shape_type(self, shape_type):
        self.shape_type = shape_type

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images

def resize(images):
    processed_images = []
    ratio = None
    for image in images:
        resized = imutils.resize(image, width=300)
        if ratio is None:
            ratio = image.shape[0] / float(resized.shape[0]) 
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        #thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_TOZERO)[1]
        processed_images.append(blurred)
    
    return processed_images, ratio

def filter_all_colours(images):
    filtered_images = []
    masks = []
    shapes_info = []
    combined = []
    for i, image in enumerate(images):
        shapes = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        colored_objects_masks = {}
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        if cv2.countNonZero(yellow_mask) > 0:
            colored_objects_masks["yellow"] = yellow_mask
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        if cv2.countNonZero(green_mask) > 0:
            colored_objects_masks["green"] = green_mask
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        if cv2.countNonZero(blue_mask) > 0:
            colored_objects_masks["blue"] = blue_mask
        red_mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        red_mask = red_mask_1 | red_mask_2
        if cv2.countNonZero(red_mask) > 0:
            colored_objects_masks["red"] = red_mask
        purple_mask = cv2.inRange(hsv, lower_purple, upper_purple)
        if cv2.countNonZero(purple_mask) > 0:
            colored_objects_masks["purple"] = purple_mask

        objects_mask = yellow_mask | green_mask | blue_mask | red_mask | purple_mask
        res = cv2.bitwise_and(image, image, mask=objects_mask)

        for colour, mask in colored_objects_masks.items():
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)

            for contour in contours:
                if cv2.contourArea(contour) > 300:
                    M = cv2.moments(contour)
                    if M["m00"] == 0:
                        continue
                    cX = int((M["m10"] / M["m00"]) )
                    cY = int((M["m01"] / M["m00"]))
                    shapes.append(Shape_Info(cX, cY, contour, colour))
                    print("countour area: ", contour.shape)
        #shapes_info.append(shapes)

        combined.append((objects_mask, shapes))
        
        #filtered_images.append(res)
        #masks.append(objects_mask)

    #print(shapes_info)
    return combined

def main():
    directory_path = "test_dataset"
    images = load_images_from_folder(directory_path)
    resized, ratio = resize(images)
    processed_images = filter_all_colours(resized)
if __name__ == '__main__':
    main()