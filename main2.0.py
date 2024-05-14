import numpy as np
import cv2
from djitellopy import Tello
from enum import Enum

class Step(Enum):
    TAKEOFF = 1
    MOVE_FORWARD = 2
    MOVE_LEFT = 3
    MOVE_RIGHT = 4
    MOVE_BACKWARD = 5
    MOVE_UP = 6
    MOVE_DOWN = 7
    MOVE_CUSTOM =8
    ROTATE = 9
    LAND = 10
    EMERGENCY = 11
debug = True  # Set to False to enable actual flying

def initialize_tello_drone():
    global me
    me = Tello()
    me.connect()
    print(f"Battery level: {me.get_battery()}%")
    me.streamoff()
    me.streamon()

def get_frame_read():
    if debug:
        ret, frame = cap.read()
        return frame if ret else None
    else:
        frame_read = me.get_frame_read()
        return frame_read.frame if frame_read else None

def move_drone(x, y, z, yaw):
    if debug:
        print(f"Moving drone: x={x}, y={y}, z={z}, yaw={yaw}")
    else:
        me.send_rc_control(x, y, z, yaw)
    
    steps_taken.append({
        "step": Step.MOVE_CUSTOM,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw
    })

def rotate_drone(angle):
    if debug:
        print(f"Rotating drone: angle={angle}")
    else:
        me.rotate_clockwise(angle)
    
    steps_taken.append({
        "step": Step.ROTATE,
        "angle": angle
    })

def move_forward(distance):
    if debug:
        print(f"Moving forward: distance={distance}")
    else:
        me.move_forward(distance)
    
    steps_taken.append({
        "step": Step.MOVE_FORWARD,
        "distance": distance
    })

def move_backward(distance):
    if debug:
        print(f"Moving backward: distance={distance}")
    else:
        me.move_back(distance)
    
    steps_taken.append({
        "step": Step.MOVE_BACKWARD,
        "distance": distance
    })

def move_left(distance):
    if debug:
        print(f"Moving left: distance={distance}")
    else:
        me.move_left(distance)
    
    steps_taken.append({
        "step": Step.MOVE_LEFT,
        "distance": distance
    })

def move_right(distance):
    if debug:
        print(f"Moving right: distance={distance}")
    else:
        me.move_right(distance)
    
    steps_taken.append({
        "step": Step.MOVE_RIGHT,
        "distance": distance
    })

def move_up(distance):
    if debug:
        print(f"Moving up: distance={distance}")
    else:
        me.move_up(distance)

    steps_taken.append({
        "step": Step.MOVE_UP,
        "distance": distance
    })

def move_down(distance):
    if debug:
        print(f"Moving down: distance={distance}")
    else:
        me.move_down(distance)
    
    steps_taken.append({
        "step": Step.MOVE_DOWN,
        "distance": distance
    })

def land():
    if debug:
        print("Landing drone.")
    else:
        me.land()
    
    steps_taken.append({
        "step": Step.LAND
    })

def takeoff():
    if debug:
        print("Taking off.")
    else:
        me.takeoff()
    
    steps_taken.append({
        "step": Step.TAKEOFF
    })

def emergency():
    if debug:
        print("Emergency stop.")
    else:
        me.emergency()
    
    steps_taken.append({
        "step": Step.EMERGENCY
    })
    
def set_speed(speed):
    if debug:
        print(f"Setting speed: speed={speed}")
    else:
        me.set_speed(speed)
    
def get_battery():
    if debug:
        print(f"Getting battery level.")
    else:
        return me.get_battery()
    
def calculate_target_location():
    pass

if debug:
    cap = cv2.VideoCapture(1)  
    if not cap.isOpened():
        print("Error: Camera could not be opened.")
        quit()
    else:
        print("Camera opened successfully.")
else:
    initialize_tello_drone()

# Constants for frame size
width, height = 640, 480

# Load the reference image and prepare ORB
ref_image = cv2.imread('test3.png')
orb = cv2.ORB_create(nfeatures=1000)
kp1, des1 = orb.detectAndCompute(ref_image, None)

# FLANN matcher with LSH index for binary descriptors
index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

matches_found = []
steps_taken = []

while True:
    myFrame = get_frame_read()
    if myFrame is None:
        print("Failed to capture frame; skipping.")
        continue

    img = cv2.resize(myFrame, (width, height))
    kp2, des2 = orb.detectAndCompute(img, None)

    if des1 is not None and des2 is not None and len(des2) >= 2:  # Ensure there are enough descriptors
        matches = flann.knnMatch(des1, des2, k=2)

        # Apply ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:  # Ensure there are exactly two matched features
                m, n = match_pair  # m is the best match, n is the second best match
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        # low amount of matches threshold for far away objects. will have false positives 
        if len(good_matches) > 5:
            # Processing here to use matches
            img_matches = cv2.drawMatches(ref_image, kp1, img, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow('Good Matches', img_matches) # Show the matches
            print(f"Good matches found: {len(good_matches)}")
        else:
            print(f"Not enough good matches found({len(matches)}, {len(good_matches)}/5")

       
        cv2.imshow('Object Tracking', img)  # Show the current frame

        if debug:
            if cv2.waitKey(1) & 0xFF == ord('p'):
                img_stream = img.copy()
                print("saving match")
                # save the image and matches as well as the tello me cordinates
                matches_found.append({
                    "img": img_stream,
                    "matches": good_matches
                })
                

    if cv2.waitKey(1) & 0xFF == ord('q'):
        if not debug:
            me.land()
        break

if debug:
    cap.release()
cv2.destroyAllWindows()

# Save the matches found
for i, match in enumerate(matches_found):
    cv2.imwrite(f"matches/match{i}.png", match["img"])
    print(f"Match {i} saved.")
