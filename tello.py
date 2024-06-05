
import asyncio
from djitellopy import Tello
from enum import Enum
import cv2
import time
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

class _Tello:

    id = 0

    def __init__(self, address=None, debug=2, cap_src = 0, speed=10, verbose=False):
        self.id = _Tello.id
        _Tello.id += 1

        self.address = address
        self.debug = debug
        self.verbose = verbose
        self.speed = speed
        self.moving = False
        self.stage = 0
        self.orientation = 0
        self.position = (0, 0)
        self.iteration = 0
        self.correction = 10
        
        if debug <2:
            self.initialize_tello_drone()
        self.steps_taken = []
        if debug == 2:
            self.cap = cv2.VideoCapture(cap_src)  
            if not self.cap.isOpened():
                print(f"[drone: {self.id}] Error: Camera could not be opened.")
                quit()
            else:
                print(f"[drone: {self.id}] Camera opened successfully.")

    def initialize_tello_drone(self):
        global me
        me = Tello()
        if self.address is not None:
            me.address = self.address
        me.connect()
        bat = me.get_battery()
        print(f"[drone: {self.id}] Battery level: {bat}%")
        if bat < 10:
            print(f"[done: {self.id}] Battery is too low. Please charge now")
            quit()
        good_vid = False
        while not good_vid:
            self.try_init_video()
            try:
                me.get_frame_read()
                print("frame stream opened")
                good_vid = True
            except:
                print("failed to get frame")
            
        me.set_speed(self.speed)
    
    def try_init_video(self):
        if self.debug == 2:
            return True
        else:
            me.set_video_direction(0)
            me.streamoff()
            me.streamon()
            me.streamoff()
            me.set_video_direction(1)
            me.streamon()

    async def get_frame_read(self):
        if self.debug == 2:
            ret, frame = self.cap.read()
            return frame if ret else None
        else:
            frame_read = me.get_frame_read()
            return frame_read.frame if frame_read else None

    async def rotate_90(self, mult):
        await self.rotate_drone(90*-mult)
        
        self.orientation = self.orientation + -mult
        if self.orientation > 3:
            self.orientation = self.orientation % 4
        
        while self.orientation < 0:
            self.orientation = 4 + self.orientation

    async def orient_north(self):

        if self.orientation == 1:
            await self.rotate_90(1)
        elif self.orientation == 2:
            await self.rotate_90(-2)
        elif self.orientation == 3:
            await self.rotate_90(-1)


    async def rotate_drone(self, angle):
        if self.verbose: 
            print(f"[drone: {self.id}] Rotating drone: angle={angle}")
        
        if self.debug == 0:
            if angle < 0:
                me.rotate_counter_clockwise(-(int)(angle))
            else:
                me.rotate_clockwise((int)(angle))
        
        self.steps_taken.append({
            "step": Step.ROTATE,
            "angle": angle
        })
        if self.debug > 0:
            await asyncio.sleep(abs(angle)/self.speed)

    async def move_forward(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving forward: distance={distance}")
        if self.debug == 0:
            me.move_forward((int)(distance-self.correction))
        
        self.steps_taken.append({
            "step": Step.MOVE_FORWARD,
            "distance": distance
        })
        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

        # depending on orientation, update position
        if self.orientation == 0:
            self.position = (self.position[0], self.position[1] + distance)
        elif self.orientation == 1:
            self.position = (self.position[0] + distance, self.position[1])
        elif self.orientation == 2:
            self.position = (self.position[0], self.position[1] - distance)
        elif self.orientation == 3:
            self.position = (self.position[0] - distance, self.position[1])

    async def move_backward(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving backward: distance={distance}")
        if self.debug == 0:
            me.move_back((int)(distance-self.correction))
        
        self.steps_taken.append({
            "step": Step.MOVE_BACKWARD,
            "distance": distance
        })
        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

        # depending on orientation, update position
        if self.orientation == 0:
            self.position = (self.position[0], self.position[1] - distance)
        elif self.orientation == 1:
            self.position = (self.position[0] - distance, self.position[1])
        elif self.orientation == 2:
            self.position = (self.position[0], self.position[1] + distance)
        elif self.orientation == 3:
            self.position = (self.position[0] + distance, self.position[1])


    async def move_left(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving left: distance={distance}")
        if self.debug == 0:
            me.move_left((int)(distance-(self.correction/2)))
        
        self.steps_taken.append({
            "step": Step.MOVE_LEFT,
            "distance": distance
        })
        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

        # depending on orientation, update position
        if self.orientation == 0:
            self.position = (self.position[0] - distance, self.position[1])
        elif self.orientation == 1:
            self.position = (self.position[0], self.position[1] + distance)
        elif self.orientation == 2:
            self.position = (self.position[0] + distance, self.position[1])
        elif self.orientation == 3:
            self.position = (self.position[0], self.position[1] - distance)

    async def move_right(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving right: distance={distance}")
        if self.debug == 0:
            me.move_right((int)(distance-(self.correction/2)))
        
        self.steps_taken.append({
            "step": Step.MOVE_RIGHT,
            "distance": distance
        })
        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

        # depending on orientation, update position
        if self.orientation == 0:
            self.position = (self.position[0] + distance, self.position[1])
        elif self.orientation == 1:
            self.position = (self.position[0], self.position[1] - distance)
        elif self.orientation == 2:
            self.position = (self.position[0] - distance, self.position[1])
        elif self.orientation == 3:
            self.position = (self.position[0], self.position[1] + distance)

    async def move_up(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving up: distance={distance}")
        if self.debug == 0: 
            me.move_up((int)(distance))

        self.steps_taken.append({
            "step": Step.MOVE_UP,
            "distance": distance
        })
        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

    async def move_down(self, distance):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Moving down: distance={distance}")
            
        if self.debug == 0:
            me.move_down((int)(distance))
        
        self.steps_taken.append({
            "step": Step.MOVE_DOWN,
            "distance": distance
        })

        if self.debug > 0:
            await asyncio.sleep(distance/self.speed)
        self.moving = False

    async def stop(self):
        if self.verbose:
            print(f"[drone: {self.id}] Stopping drone.")
        if self.debug == 0:
            me.send_rc_control(0, 0, 0, 0)
        self.moving = False

    async def land(self):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Landing drone.")
            
        if self.debug == 0:
            me.land()
        
        self.steps_taken.append({
            "step": Step.LAND
        })
        if self.debug > 0:
            await asyncio.sleep(5)
        self.moving = False

    async def takeoff(self):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Taking off.")
            
        if self.debug == 0:
            me.takeoff()
        
        self.steps_taken.append({
            "step": Step.TAKEOFF
        })
        if self.debug > 0:
            await asyncio.sleep(5)
        self.moving = False

    async def emergency(self):
        self.moving = True
        if self.verbose:
            print(f"[drone: {self.id}] Emergency stop.")
            
        if self.debug == 0:
            me.emergency()
        
        self.steps_taken.append({
            "step": Step.EMERGENCY
        })
        if self.debug > 0:
            await asyncio.sleep(5)
        self.moving = False
        
    async def set_speed(self, speed):
        if self.verbose:
            print(f"[drone: {self.id}] Setting speed: speed={speed}")
        
        if self.debug == 0:
            me.set_speed(speed)
        
    async def get_battery(self):
        if self.verbose:
            print(f"[drone: {self.id}] Getting battery level.")

        if self.debug == 0:
            return me.get_battery()

        return 100
    
    async def get_height(self):
        height = me.get_height() if self.debug == 0 else 40

        if self.verbose:
            print(f"[drone: {self.id}] height: {height}")

        return height

        
            
    async def backtrack(self):
        steps_taken = self.steps_taken.copy()
        steps_taken.reverse()
        for step in steps_taken:
            self.moving = True
            if step["step"] == Step.MOVE_FORWARD:
                await self.move_backward(step["distance"])
            elif step["step"] == Step.MOVE_BACKWARD:
                await self.move_forward(step["distance"])
            elif step["step"] == Step.MOVE_LEFT:
                await self.move_right(step["distance"])
            elif step["step"] == Step.MOVE_RIGHT:
                await self.move_left(step["distance"])
            elif step["step"] == Step.MOVE_UP:
                await self.move_down(step["distance"])
            elif step["step"] == Step.MOVE_DOWN:
                await self.move_up(step["distance"])
            elif step["step"] == Step.ROTATE:
                await self.rotate_drone(-step["angle"])
            elif step["step"] == Step.LAND:
                await self.takeoff()
            elif step["step"] == Step.TAKEOFF:
                await self.land()
            elif step["step"] == Step.EMERGENCY:
                await self.emergency()
            elif step["step"] == Step.MOVE_CUSTOM:
                await self.move_drone(-step["x"], -step["y"], -step["z"], -step["yaw"])
        self.moving = False

    async def off(self):
        if self.debug == 2:
            self.cap.release()
            cv2.destroyAllWindows()
        else:
            me.streamoff()
            me.end()

    async def reset_steps_taken(self):
        self.steps_taken = []

    async def is_moving(self):
        return self.moving