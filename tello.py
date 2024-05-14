
from djitellopy import Tello
from enum import Enum
import cv2

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

class Tello:

    def __init__(self, debug=True):
        self.debug = debug
        self.me = None
        self.steps_taken = []
        if debug:
            self.cap = cv2.VideoCapture(1)  
            if not self.cap.isOpened():
                print("Error: Camera could not be opened.")
                quit()
            else:
                print("Camera opened successfully.")

    def initialize_tello_drone(self):
        global me
        me = Tello()
        me.connect()
        print(f"Battery level: {me.get_battery()}%")
        me.streamoff()
        me.streamon()

    def get_frame_read(self):
        if self.debug:
            ret, frame = self.cap.read()
            return frame if ret else None
        else:
            frame_read = me.get_frame_read()
            return frame_read.frame if frame_read else None

    def move_drone(self, x, y, z, yaw):
        if self.debug:
            print(f"Moving drone: x={x}, y={y}, z={z}, yaw={yaw}")
        else:
            me.send_rc_control(x, y, z, yaw)
        
        self.steps_taken.append({
            "step": Step.MOVE_CUSTOM,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw
        })

    def rotate_drone(self, angle):
        if self.debug:
            print(f"Rotating drone: angle={angle}")
        else:
            me.rotate_clockwise(angle)
        
        self.steps_taken.append({
            "step": Step.ROTATE,
            "angle": angle
        })

    def move_forward(self, distance):
        if self.debug:
            print(f"Moving forward: distance={distance}")
        else:
            me.move_forward(distance)
        
        self.steps_taken.append({
            "step": Step.MOVE_FORWARD,
            "distance": distance
        })

    def move_backward(self, distance):
        if self.debug:
            print(f"Moving backward: distance={distance}")
        else:
            me.move_back(distance)
        
        self.steps_taken.append({
            "step": Step.MOVE_BACKWARD,
            "distance": distance
        })

    def move_left(self, distance):
        if self.debug:
            print(f"Moving left: distance={distance}")
        else:
            me.move_left(distance)
        
        self.steps_taken.append({
            "step": Step.MOVE_LEFT,
            "distance": distance
        })

    def move_right(self, distance):
        if self.debug:
            print(f"Moving right: distance={distance}")
        else:
            me.move_right(distance)
        
        self.steps_taken.append({
            "step": Step.MOVE_RIGHT,
            "distance": distance
        })

    def move_up(self, distance):
        if self.debug:
            print(f"Moving up: distance={distance}")
        else:
            me.move_up(distance)

        self.steps_taken.append({
            "step": Step.MOVE_UP,
            "distance": distance
        })

    def move_down(self, distance):
        if self.debug:
            print(f"Moving down: distance={distance}")
        else:
            me.move_down(distance)
        
        self.steps_taken.append({
            "step": Step.MOVE_DOWN,
            "distance": distance
        })

    def land(self):
        if self.debug:
            print("Landing drone.")
        else:
            me.land()
        
        self.steps_taken.append({
            "step": Step.LAND
        })

    def takeoff(self):
        if self.debug:
            print("Taking off.")
        else:
            me.takeoff()
        
        self.steps_taken.append({
            "step": Step.TAKEOFF
        })

    def emergency(self):
        if self.debug:
            print("Emergency stop.")
        else:
            me.emergency()
        
        self.steps_taken.append({
            "step": Step.EMERGENCY
        })
        
    def set_speed(self, speed):
        if self.debug:
            print(f"Setting speed: speed={speed}")
        else:
            me.set_speed(speed)
        
    def get_battery(self):
        if self.debug:
            print(f"Getting battery level.")
        else:
            return me.get_battery()
        
    def calculate_target_location(self):
        pass

    def move_to_target(self):
        pass
    
    def backtrack(self):
        steps_taken = self.steps_taken.copy()
        steps_taken.reverse()
        for step in steps_taken:
            if step["step"] == Step.MOVE_FORWARD:
                self.move_backward(step["distance"])
            elif step["step"] == Step.MOVE_BACKWARD:
                self.move_forward(step["distance"])
            elif step["step"] == Step.MOVE_LEFT:
                self.move_right(step["distance"])
            elif step["step"] == Step.MOVE_RIGHT:
                self.move_left(step["distance"])
            elif step["step"] == Step.MOVE_UP:
                self.move_down(step["distance"])
            elif step["step"] == Step.MOVE_DOWN:
                self.move_up(step["distance"])
            elif step["step"] == Step.ROTATE:
                self.rotate_drone(-step["angle"])
            elif step["step"] == Step.LAND:
                self.takeoff()
            elif step["step"] == Step.TAKEOFF:
                self.land()
            elif step["step"] == Step.EMERGENCY:
                self.emergency()
            elif step["step"] == Step.MOVE_CUSTOM:
                self.move_drone(-step["x"], -step["y"], -step["z"], -step["yaw"])

    def reset_steps_taken(self):
        self.steps_taken = []