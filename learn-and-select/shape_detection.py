import argparse
from hashlib import new
import imutils
import cv2
import os
import numpy as np

from colour_detection import resize, filter_all_colours


def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images

def preprocess_images(images):
    processed_images = []
    ratio = None
    for image in images:
        resized = imutils.resize(image, width=300)
        if ratio is None:
            ratio = image.shape[0] / float(resized.shape[0]) 
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        processed_images.append(thresh)
    
    return processed_images, ratio

def invert_images(images):
    inverted = []
    for img in images:
        inverted.append(cv2.bitwise_not(img))

    return inverted
#def compute_centres(countours, ratio):

# def to_grayscale(images):
#     for image in images:
#         gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
#         processed_images.append(thresh)

def create_comnvex_hull(contour):
    hull = cv2.convexHull(contour)

def filter_shape(approximate_side_number, contour, image):
    if(approximate_side_number == 3 or approximate_side_number == 4):
        return "cone"
    elif(approximate_side_number == 6):
        return "skittle"
    else:
        mask = np.zeros(image.shape, np.uint8)
        hull = cv2.convexHull(contour)
        #print("hull: ", np.squeeze(hull))
        cv2.drawContours(mask, [np.squeeze(hull)], -1, 255, cv2.FILLED)
        
        mean = cv2.mean(image, mask=mask)

        #print("mean: ", mean)

        return "ball" if mean[0]>=100 else "ring"

def get_polygon_lines(countour):
    peri = cv2.arcLength(countour, True)
    approx = cv2.approxPolyDP(countour, 0.03 * peri, True)
    return approx

def detect_shapes(masks_and_shapes, ratio):
    print("masks_and_shapes: ", len(masks_and_shapes))
    for i, mask_and_shapes in enumerate(masks_and_shapes): 
        #contours = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #contours = imutils.grab_contours(contours)
        #print("contours: ", contours)
 
        # for contour in contours:
        #     M = cv2.moments(contour)
        #     if M["m00"] == 0:
        #         continue
        #     cX = int((M["m10"] / M["m00"]) )
        #     cY = int((M["m01"] / M["m00"]))
        mask, shapes = mask_and_shapes
        print("shapes: ", len(shapes))
        for shape in shapes:
            
            print("shape contour len", shape.contour)
            #contour = shape.contour.astype("float")
            contour = shape.contour
            #contour = contour.astype("float")
            line_number = len(get_polygon_lines(contour))
            shape_type = filter_shape(line_number, contour, mask)
            print("SHAPE TYPE: ", shape_type)
            
            #ontour *= ratio

            backtorgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
            cv2.circle(backtorgb, (shape.cX, shape.cY), 7, (0, 0, 255), -1)
            print("cx, xy: ", shape.cX, shape.cY)
            print(ratio)

            contour = contour.astype("int")
            
            print("line number approx: ", line_number)
            shape.add_shape_type(shape_type)
            
            # cv2.drawContours(backtorgb, contour, -1, (20,255,0), 3)
            # cv2.putText(backtorgb, shape_type + " " + shape.colour, (shape.cX, shape.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # # show the output image
            # cv2.imshow("Image" + str(i), backtorgb)
            # cv2.waitKey(0)
        return shapes

def detect_shapes_2(mask_and_shapes, ratio):

    mask, shapes = mask_and_shapes
    print("shapes: ", len(shapes))
    for shape in shapes:
        
        print("shape contour len", shape.contour)
        #contour = shape.contour.astype("float")
        contour = shape.contour
        #contour = contour.astype("float")
        line_number = len(get_polygon_lines(contour))
        shape_type = filter_shape(line_number, contour, mask)
        print("SHAPE TYPE: ", shape_type)
        
        #ontour *= ratio

        backtorgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
        cv2.circle(backtorgb, (shape.cX, shape.cY), 7, (0, 0, 255), -1)
        print("cx, xy: ", shape.cX, shape.cY)
        print(ratio)

        contour = contour.astype("int")
        
        print("line number approx: ", line_number)
        
        cv2.drawContours(backtorgb, contour, -1, (20,255,0), 3)
        cv2.putText(backtorgb, shape_type + " " + shape.colour, (shape.cX, shape.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # show the output image
        cv2.imshow("Image" + str(i), backtorgb)
        cv2.waitKey(0)


def main():
    directory_path = "test_dataset"
    images = load_images_from_folder(directory_path)
    resized_images, ratio = resize(images)
    print(ratio)
    #ratio = get_ratio(images[0])
    masks_and_shapes = filter_all_colours(resized_images)
    #inverted = invert_images(masked_objects_images)

    shapes = detect_shapes(masks_and_shapes, ratio)
    

if __name__ == '__main__':
    main()