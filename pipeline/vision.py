import cv2
import os 
import matplotlib.pyplot as plt
import numpy as np

#copied functions from ipynb 

def load_image(filename, folder="../data"):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.abspath(os.path.join(script_dir, folder, filename))
    image = cv2.imread(filepath)

    if image is None:
        print(f"No image imported at '{filepath}'.")
    
    return image

def get_rgb_image(image):
    if image is None: 
        return None
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

def get_hsv_image(image):
    if image is None: 
        return None
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def extract_road_mask(hsvimage):   
    if hsvimage is None:
        return None
    
    lower_thres_grey = np.array([0, 0, 150])
    upper_thres_grey = np.array([179, 50, 220])
    
    road_mask = cv2.inRange(hsvimage, lower_thres_grey, upper_thres_grey)
    
    return road_mask

def extract_start_mask(rgbimage):
    if rgbimage is None: 
        return None
    
    lower_thres_red = np.array([150, 0, 0])
    upper_thres_red = np.array([255, 100, 100])

    start_mask = cv2.inRange(rgbimage, lower_thres_red, upper_thres_red)

    return start_mask

def extract_end_mask(rgbimage):
    if rgbimage is None: 
        return None
    
    lower_thres_green = np.array([50, 200, 0])
    upper_thres_green = np.array([180, 255, 100])

    goal_mask = cv2.inRange(rgbimage, lower_thres_green, upper_thres_green)

    return goal_mask

def filter_road_mask(road_mask):
    if road_mask is None:
        return None
    #These remove the text in between the road
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4)) # 3 had one tiny spot left
    cleaned_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_mask = np.zeros_like(road_mask)

    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse = True ) #Sort first
        cv2.drawContours(filtered_mask, [contours[0]], -1, 255, -1) #The largest contour is the road
        filtered_mask = cv2.bitwise_and(cleaned_mask, filtered_mask) # Without that it removed also the middle of the roundabout

    return filtered_mask

def get_marker_centers(marker_mask):
    if marker_mask is None: 
        return None
    
    contours, _ = cv2.findContours(marker_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours: 
        return None
    
    contours = sorted(contours, key=cv2.contourArea, reverse=True) 
    marker = contours[0]

    (x, y), radius = cv2.minEnclosingCircle(marker)
    center = (int(x), int(y))

    return center

def apply_safety_margin(road_mask, margin = 2):
    if road_mask is None:
        return None
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (margin*2+1, margin*2+1))

    safe_mask = cv2.erode(road_mask, kernel, iterations=1)

    return safe_mask
    
def vision_pipeline(filename, folder="../data"): 
    background_image = load_image(filename, folder)
    rgb_image = get_rgb_image(background_image)
    hsv_image = get_hsv_image(background_image)

    raw_road_mask = extract_road_mask(hsv_image)
    filtered_road_mask = filter_road_mask(raw_road_mask)
    safe_road_mask = apply_safety_margin(filtered_road_mask, margin=2)

    start_mask = extract_start_mask(rgb_image)
    end_mask = extract_end_mask(rgb_image)

    start_coordinates = get_marker_centers(start_mask)
    end_coordinates = get_marker_centers(end_mask)

    if start_coordinates: #Draw white circle around start and end
        cv2.circle(filtered_road_mask, start_coordinates, 13, 255, -1) 
    if end_coordinates:
        cv2.circle(filtered_road_mask, end_coordinates, 14, 255, -1)
    
    safe_road_mask = apply_safety_margin(filtered_road_mask, margin=2)

    return safe_road_mask, start_coordinates, end_coordinates
    

