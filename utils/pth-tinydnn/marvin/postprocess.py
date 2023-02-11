import cv2
import numpy as np

'''Returns list of bounding boxes'''
def find_box(thresh_image): # (H, W) with values being either 0 or 1.
    # Area of image
    image_area = thresh_image.size
    
    # Compute contours
    contours, _ = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Use contours to get bounding boxes
    bbox_list = []
    for cnt in contours:
        bbox = cv2.boundingRect(cnt)
        bbox_x, bbox_y, bbox_w, bbox_h = bbox
        
        # Only consider bounding box candidates that are not too small/big
        if 0.02 * image_area < bbox_w * bbox_h < 0.5 * image_area:
            
            # Reject bounding box candidates that have a low percentage of 1 pixels
            if last_check(bbox, thresh_image):
                bbox_list.append(bbox)
                
    return bbox_list


'''Returns true if at least 50% of the pixels in the bbox are 1'''
def last_check(bbox, thresh_image):
    bbox_x, bbox_y, bbox_w, bbox_h = bbox
    
    count = 0 # number of 1 pixels in bbox
    for y in range(bbox_y, bbox_y + bbox_h):
        for x in range(bbox_x, bbox_x + bbox_w):
            pixel = thresh_image.item(y, x)
            if pixel == 1:
                count += 1
    
    return count / (bbox_h * bbox_w) > 0.5



'''Main function that returns a list of the bounding boxes and the input image with the bounding box predictions'''
def bbox_predictions(data, pred, thresh):
    '''
    `data`, numpy array of the input image of shape (H, W).
    `pred`, numpy array of the predicted regions of shape (H, W).
    `thresh`, the threshold used to predict pixel labels.
    '''
      
    # Threshold the prediction
    _, thresh_image = cv2.threshold(pred, thresh, 1, cv2.THRESH_BINARY) # convert to binary
    
    # Convert to type int8 (to be compatible with findContour) 
    thresh_image = np.array(thresh_image, dtype=np.uint8)
    
    # Get bounding boxes
    bbox_list = find_box(thresh_image)
    
    # Draw bounding boxes onto image
    data_copy = data.copy()
    for bbox in bbox_list:
        bbox_x, bbox_y, bbox_w, bbox_h = bbox
        cv2.rectangle(data_copy,(bbox_x, bbox_y), (bbox_x + bbox_w, bbox_y + bbox_h), color=(0), thickness=2) # color=(0) is black

    return bbox_list, data_copy

