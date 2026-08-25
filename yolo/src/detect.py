# ============================================================
# detect.py (decision algorithm)
# Detects boxes with YOLO, filters/scores them, selects the 
# best one, and prepares it for handoff to the OMX robot arm.
# ============================================================
from ultralytics import YOLO

# Load model once at import time (not inside functions) — avoids reloading the model on every single detection call
model = YOLO("best.pt")

# ------------------------------------------------------------
# Step 1: Run YOLO detection on an image
# ------------------------------------------------------------

def run_detection(image_path, confidence_threshold=0.001):
    """
    Runs YOLO detection on the given image and returns the detected boxes.

    Args:
        image_path (str): Path to the input image.
        confidence_threshold (float): Minimum confidence score for detections.
        
    """

    results = model.predict(image_path, conf=confidence_threshold)

    detections = []
    for box in results[0].boxes:
        x,y,w,h = box.xywh[0].tolist()  # Get box coordinates
        class_id = int(box.cls[0])  # Get class ID( 0 = normal box , 1= defect(damaged box))
        class_name = results[0].names[class_id]  # 숫자를 실제 이름("normal", "defect")으로 변환
        confidence = float(box.conf[0])  # Get confidence score

        detections.append({
            "class": class_name,
            "confidence": confidence,
            "xywh": (x, y, w, h)
        })
    return detections

"""  ****sample output of run_detection function***
detections = [
    {"class": "normal", "confidence": 0.9, "xywh": (100,100,50,50)},
    {"class": "defect", "confidence": 0.8, "xywh": (120,120,50,50)}
]
"""

# ------------------------------------------------------------
# Step 2: Filter out detections below confidence threshold
# ------------------------------------------------------------

def filter_detections(detections, confidence_threshold=0.6):

    #confidence_threshold=0.6은 기본값 — 나중에 실제 model 성능 보고 조정 가능
    """
    Filters out detections below the confidence threshold.

    Args:
        detections (list): List of detected boxes.
        confidence_threshold (float): Minimum confidence score for detections.
        
    Returns:
        list: Filtered list of detections above the confidence threshold.
    """

    filtered = [d for d in detections if d["confidence"] >= confidence_threshold]
    return filtered


# ------------------------------------------------------------
# Step 3a: Helper — calculate overlap between two boxes
# ------------------------------------------------------------

def calculate_overlap_fraction(box1, box2):
    """
    Calculates the overlap fraction between two boxes.

    Args:
        box1 (tuple): (x, y, w, h) for the first box.
        box2 (tuple): (x, y, w, h) for the second box.

    Returns:
        float: Overlap fraction between the two boxes.
    """

    x1_min = box1[0] - box1[2] / 2
    x1_max = box1[0] + box1[2] / 2
    y1_min = box1[1] - box1[3] / 2
    y1_max = box1[1] + box1[3] / 2

    x2_min = box2[0] - box2[2] / 2
    x2_max = box2[0] + box2[2] / 2
    y2_min = box2[1] - box2[3] / 2
    y2_max = box2[1] + box2[3] / 2

    # Calculate intersection
    inter_x_min = max(x1_min, x2_min)
    inter_x_max = min(x1_max, x2_max)
    inter_y_min = max(y1_min, y2_min)
    inter_y_max = min(y1_max, y2_max)

    if inter_x_max <= inter_x_min or inter_y_max <= inter_y_min:
        return 0.0  # No overlap

    inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
    box1_area = box1[2] * box1[3]
    return inter_area / box1_area  # Overlap fraction relative to box1


# ------------------------------------------------------------
# Step 3b: Calculate occlusion score for each detection
# ------------------------------------------------------------

def calculate_occlusion_score(detections):
    """
    Calculates the occlusion score for each detection based on overlap with other boxes.

    Args:
        detections (list): List of detected boxes.  
    
    detected box (0 = fully isolated, closer to 1 = heavily covered).

    """
    boxes = [d["xywh"] for d in detections]

    for d in detections:
        max_overlap = 0.0
        for other_box in boxes:
            if other_box == d["xywh"]: #비교 대상이 자기 자신이면 건너뜀
                continue
            overlap = calculate_overlap_fraction(d["xywh"], other_box) #box(d)가 other_box한테 얼마나 가려졌는지 계산
            max_overlap = max(max_overlap, overlap)
        d["occlusion_score"] = max_overlap
    
    return detections

""" ****sample output of calculate_occlusion_score function***
[
    {"class": "normal", "confidence": 0.9, "xywh": (100,100,50,50), "occlusion_score": 0.36},
    {"class": "defect", "confidence": 0.8, "xywh": (120,120,50,50), "occlusion_score": 0.36}
]
"""


# ------------------------------------------------------------
# Step 4: Decide if each detection is safe enough to pick
# ------------------------------------------------------------

def check_pickability(detections, occlusion_threshold=0.5): #occlusion_threshold :실제 테스트 결과 보고 조정
    """
    Checks if each detection is safe enough to pick based on occlusion score.

    Args:
        detections (list): List of detected boxes with occlusion scores.
        occlusion_threshold (float): Maximum allowed occlusion score for picking.
    
    (Confidence filtering already happened in filter_by_confidence().)
    """

    for  d in detections:
        d["pickable"] = d["occlusion_score"] < occlusion_threshold

    return detections

# ------------------------------------------------------------
# Step 5: Calculate a ranking score for pickable detections
# ------------------------------------------------------------

def calculate_priority_score(detections,confidence_weight=0.6, occlusion_weight=0.4): 
    #confidence_weight, occlusion_weight : 실제 테스트 결과 보고 조정
    """
    Calculates a priority score for each detection based on confidence and occlusion.
    Higher confidence and lower occlusion = higher priority.
    Non-pickable detections get priority_score = -1 (never selected).
    
    """

    for d in detections:
        if d["pickable"]:
            d["priority_score"] = (confidence_weight * d["confidence"]) - (occlusion_weight * d["occlusion_score"])
        else:
            d["priority_score"] = -1
    return detections


# ------------------------------------------------------------
# Step 6: Select the single best box to pick
# ------------------------------------------------------------

def select_best_box(detections):
    """
    Selects the single best box to pick based on priority score.
    Returns None if no pickable boxes are available.
    """

    pickable_detections = [d for d in detections if d["pickable"]]
    if not pickable_detections:
        return None  # No pickable boxes

    best_box = max(pickable_detections, key=lambda d: d["priority_score"])
    return best_box

#best_box = {"class": "defect", "priority_score": 0.82}  # 결과

# ------------------------------------------------------------
# Step 7: Convert pixel coordinates to robot coordinates and 
# format the final output for the OMX team
# ------------------------------------------------------------



def send_to_robot(best_box,img_width, img_height, workspace_size, corner_x,corner_y):
    """
    Converts pixel coordinates to robot coordinates and formats the final output for the OMX team.

    Args:
        best_box (dict): The best box selected for picking.
        img_width (int): Width of the image in pixels.
        img_height (int): Height of the image in pixels.
        workspace_size (tuple): Size of the robot workspace in mm (width, height).
        corner_x (float): X coordinate of the workspace corner in mm.
        corner_y (float): Y coordinate of the workspace corner in mm.

    Returns:
        dict: Formatted output with robot coordinates and box information.

    Returns None if best_box is None (nothing pickable).  
    """

    if best_box is None:
        return None  # No box to send

    # Convert pixel coordinates to robot coordinates
    x_pixel, y_pixel, w_pixel, h_pixel = best_box["xywh"]
    
    # Calculate scaling factors
    scale_x = workspace_size[0] / img_width
    scale_y = workspace_size[1] / img_height

    # Convert to robot coordinates
    x_robot = corner_x + (x_pixel * scale_x)
    y_robot = corner_y + (y_pixel * scale_y)

    # Prepare final output
    output = {
        "class": best_box["class"],
        "confidence": best_box["confidence"],
        "occlusion_score": best_box["occlusion_score"],
        "priority_score": best_box["priority_score"],
        "robot_coordinates": {
            "x": x_robot,
            "y": y_robot,
            "width": w_pixel * scale_x,
            "height": h_pixel * scale_y
        }
    }

    return output


# ------------------------------------------------------------
# Full pipeline: combines all steps into one function
# ------------------------------------------------------------
def detect_and_select(image_path, img_width, img_height, 
                       mm_per_pixel_x, mm_per_pixel_y, 
                       origin_x, origin_y):
    detections = run_detection(image_path)
    detections = filter_by_confidence(detections)
    detections = calculate_occlusion_score(detections)
    detections = check_pickability(detections)
    detections = calculate_priority_score(detections)
    best_box = select_best_box(detections)
    result = send_to_robot(best_box, img_width, img_height, 
                            mm_per_pixel_x, mm_per_pixel_y, 
                            origin_x, origin_y)
    return result



