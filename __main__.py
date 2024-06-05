import math
import os
import cv2
import cv2.legacy
from tello import _Tello
import numpy as np
import asyncio

class DroneSar:
    
    def __init__(self, 
                image_name='test.png', 
                num_drones =1, 
                addresses=None, 
                bound_x=3, 
                bound_y=3, 
                debug=2,
                wanted_height=50,
                size_of_blocks=50,
                speed=10,
                start_matches = 70,
                verbose=False):
        
        if num_drones < 1:
            raise ValueError("Number of drones must be at least 1.")
        
        if num_drones > 4:
            raise ValueError("Number of drones must be at most 4.")
        
        if num_drones > 1 and (addresses is None or len(addresses) != num_drones):
            raise ValueError("Number of addresses must match the number of drones.")

        self.debug = debug
        self.verbose = verbose
        self.num_drones = num_drones
        self.drones = []
        self.no_items = 1
        self.wanted_height = wanted_height
        self.start_matches = start_matches
        self.min_height = 30

        self.size_of_blocks = size_of_blocks
        # Initialize the Tello drones

        for i in range(num_drones):
            if num_drones == 1:
                self.drones.append(_Tello(debug=debug, cap_src=1, speed=speed, verbose=verbose))
            else:
                self.drones.append(_Tello(debug=debug, address=addresses[i], cap_src=1, speed=speed, verbose=verbose))

        # Constants for frame size
        self.width, self.height = 640, 480

        # Load the reference image and prepare ORB
        self.ref_image = cv2.imread(image_name, 1)
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.kp1, self.des1 = self.orb.detectAndCompute(self.ref_image, None)

        self.index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        self.search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

        self.prev_matches = {}

        self.bounds = {"x":bound_x*100,"y":bound_y*100}
        
        self.completions = {}
        
        for drone in self.drones:
            self.completions[drone.id] = {"x":0,"y":0}
        
        self.quadrants = {}
        btbx = self.bounds['x']//self.size_of_blocks
        btby = self.bounds['y']//self.size_of_blocks

        if num_drones == 1:
            # One drone covers the entire area
            self.quadrants[self.drones[0].id] = {'start': (0, 0), 'end': (btbx, btby)}

        elif num_drones == 2:
            # Split along the x-axis
            end_of_d1_x = btbx // 2
            self.quadrants[self.drones[0].id] = {'start': (0, 0), 'end': (end_of_d1_x, btby)}
            self.quadrants[self.drones[1].id] = {'start': (end_of_d1_x, 0), 'end': (btbx, btby)}

        elif num_drones == 3:
            # One drone covers half the area, other two split the other half
            end_of_d1_x = btbx // 2
            end_of_d2_y = btby // 2

            self.quadrants[self.drones[0].id] = {'start': (0, 0), 'end': (end_of_d1_x, btby)}
            self.quadrants[self.drones[1].id] = {'start': (end_of_d1_x, 0), 'end': (btbx, end_of_d2_y)}
            self.quadrants[self.drones[2].id] = {'start': (end_of_d1_x, end_of_d2_y), 'end': (btbx, btby)}

        elif num_drones == 4:
            # Each drone takes a quarter
            end_of_d1_x = btbx // 2
            end_of_d2_y = btby // 2

            self.quadrants[self.drones[0].id] = {'start': (0, 0), 'end': (end_of_d1_x, end_of_d2_y)}
            self.quadrants[self.drones[1].id] = {'start': (end_of_d1_x, 0), 'end': (btbx, end_of_d2_y)}
            self.quadrants[self.drones[2].id] = {'start': (0, end_of_d2_y), 'end': (end_of_d1_x, btby)}
            self.quadrants[self.drones[3].id] = {'start': (end_of_d1_x, end_of_d2_y), 'end': (btbx, btby)}


        self.next_dir = {}

    async def get_matches(self, myFrame):
        img = cv2.resize(myFrame, (self.width, self.height))
        kp2, des2 = self.orb.detectAndCompute(img, None)

        if self.des1 is not None and des2 is not None and len(des2) >= 2:  # Ensure there are enough descriptors
            matches = self.flann.knnMatch(self.des1, des2, k=2)

            # Apply ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:  # Ensure there are exactly two matched features
                    m, n = match_pair  # m is the best match, n is the second best match
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

            return img, good_matches, kp2
        else:
            return img, None, kp2

    async def move_to_start(self, drone):
        print(f"Moving drone {drone.id} to start position")
        # Move along the x-axis first
        await drone.orient_north()
        current_x = drone.position[0] / self.size_of_blocks
        target_x = self.quadrants[drone.id]['start'][0]

        if current_x != target_x:
            offset_x = target_x - current_x
            if offset_x > 0:
                # move right
                await drone.move_right(offset_x * self.size_of_blocks)
            else:
                # move left
                await drone.move_left(abs(offset_x) * self.size_of_blocks)

        # Move along the y-axis next
        current_y = drone.position[1] / self.size_of_blocks
        target_y = self.quadrants[drone.id]['start'][1]

        if current_y != target_y:
            offset_y = target_y - current_y
            if offset_y > 0:
                # move forward
                await drone.move_forward(offset_y * self.size_of_blocks)
            else:
                # move backward
                await drone.move_backward(abs(offset_y) * self.size_of_blocks)

        print(f"Drone {drone.id} is now at the start position")

    async def move_next_stage_2(self, drone):
        pos_x = drone.position[0]//self.size_of_blocks
        pos_y = drone.position[1]//self.size_of_blocks

        end_x = self.quadrants[drone.id]['end'][0]-1
        end_y = self.quadrants[drone.id]['end'][1]-1

        rng_x = end_x - self.quadrants[drone.id]['start'][0]

        end_pos_x = end_x
        end_pos_y = end_y

        if rng_x % 2 != 0:
            end_pos_y = self.quadrants[drone.id]['start'][1]

        if pos_x == end_pos_x and pos_y == end_pos_y:
            await self.move_to_start(drone)
            self.next_dir[drone.id] = 'up'
            drone.iteration += 1
            height = await drone.get_height()
            if height > self.min_height:
                new_height = height - 10
                if new_height < self.min_height:
                    new_height = self.min_height+1

                await drone.move_down(height - new_height)
            return
        
        if drone.id not in self.next_dir:
            self.next_dir[drone.id] = 'up'

        if self.next_dir[drone.id] == 'up':
            if pos_y == self.quadrants[drone.id]['end'][1]-1:
                self.next_dir[drone.id] = 'down'
                await drone.move_right(self.size_of_blocks)
                await drone.rotate_90(2)
            else:
                await drone.move_forward(self.size_of_blocks)
        
        elif self.next_dir[drone.id] == 'down':
            if pos_y == self.quadrants[drone.id]['start'][1]:
                self.next_dir[drone.id] = 'up'
                await drone.move_left(self.size_of_blocks)
                await drone.rotate_90(2)
            else:
                await drone.move_forward(self.size_of_blocks)

    async def drone_logic(self, drone):
        while drone.stage != 9:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                await drone.land()
                await drone.off()
                drone.stage = 8

            # take off
            if drone.stage < 1:
                if self.verbose:
                    print("stage 1")
                await drone.takeoff()
                height = await drone.get_height()
                if height < self.wanted_height:
                    await drone.move_up(self.wanted_height - height)
                else:
                    await drone.move_down(height - self.wanted_height)
                
                await self.move_to_start(drone)
                drone.stage = 2
                await drone.reset_steps_taken()

            # check surroundings for the item
            if drone.stage == 2:
                if self.verbose:
                    print("stage 2")
                if not await drone.is_moving():
                    duration = 5
                    finish_time = asyncio.get_event_loop().time() + (duration)
                    while drone.stage == 2 and (asyncio.get_event_loop().time() < finish_time):

                        myFrame = await drone.get_frame_read()
                        if myFrame is None:
                            print(f"[{drone.id}] Failed to capture frame; skipping.")
                            continue

                        img, matches, kp2 = await self.get_matches(myFrame)
                        if matches is not None:
                            # Processing here to use matches
                            img_matches = cv2.drawMatches(self.ref_image, self.kp1, img, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                            cv2.imshow(f'Object Tracking [{drone.id}]', img_matches) # Show the matches
                            if self.verbose: print(f"[{drone.id}] Good matches found: {len(matches)}")
                            self.prev_matches[drone.id] = len(matches)
                            if len(matches) > (self.start_matches - (drone.iteration**2)):
                                drone.stage = 7
                        await asyncio.sleep(0.25)
                    
                    await self.move_next_stage_2(drone)
                        
                cv2.waitKey(1)

            if drone.stage == 7:

                if self.verbose:
                    print("stage 7")
                # item found. save coordinates and image and go to home
                
                myFrame = await drone.get_frame_read()
                if myFrame is None:
                    print("Failed to capture frame; skipping.")
                    continue
                
                img, matches, kp2 = await self.get_matches(myFrame)
                drone.stage = 8
                # draw a box around the item on frame
                if matches is not None:
                    for match in matches:
                        x, y = kp2[match.trainIdx].pt
                        cv2.circle(img, (int(x), int(y)), 10, (0, 255, 0), 3)
                    cv2.imshow('Object Tracking', img)


                os.makedirs('output', exist_ok=True)
                cv2.imwrite(f'output/{drone.id}_item.png', img)

                self.completions[drone.id] = drone.position
                if self.no_items == 1:
                    # send other drones to home
                    for other_drone in self.drones:
                        if other_drone.id != drone.id:
                            other_drone.stage = 8
                drone.stage = 8

            if  drone.stage == 8:

                if self.verbose:
                    print("stage 8")
                await self.move_to_start(drone)
                await drone.land()
                await drone.off()

                drone.stage = 9

    async def update_table(self):
        print("Starting table update task")
        while True:
            self.show_table_of_drones()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            await asyncio.sleep(0.1)

    def show_table_of_drones(self):
        # Initialize the area with ones (255 in grayscale, which is white)
        area = np.ones((math.floor(self.bounds['y'] / self.size_of_blocks), 
                        math.floor(self.bounds['x'] / self.size_of_blocks)), dtype=np.uint8) * 255
        
        # Place drones in the area
        for drone in self.drones:
            x_index = int(drone.position[0] / self.size_of_blocks)
            y_index = int(drone.position[1] / self.size_of_blocks)
            
            # Flip the y-coordinate to make (0,0) the bottom-left corner
            y_index = area.shape[0] - 1 - y_index
            
            # Ensure indices are within the bounds of the area array
            if 0 <= x_index < area.shape[1] and 0 <= y_index < area.shape[0]:
                area[y_index][x_index] = drone.id
        
        # Scale up the area for visualization
        scale_factor = 20  # Scale up by a factor of 20 for better visualization
        area_large = cv2.resize(area, (0, 0), fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        
        # Convert the scaled up area to a 3-channel BGR image
        area_large_color = cv2.cvtColor(area_large, cv2.COLOR_GRAY2BGR)
        
        # Assign colors to each drone
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]  # Red, Green, Blue, Yellow
        for i, drone in enumerate(self.drones):
            mask = area_large == drone.id
            area_large_color[mask] = colors[i]
        
        # Draw grid lines
        rows, cols = area_large_color.shape[:2]
        for i in range(0, rows, scale_factor):
            cv2.line(area_large_color, (0, i), (cols, i), (200, 200, 200), 1)
        for j in range(0, cols, scale_factor):
            cv2.line(area_large_color, (j, 0), (j, rows), (200, 200, 200), 1)
        
        # Show the table in UI
        cv2.imshow("Table of drones", area_large_color)
    
    async def safety(self):
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Safety activated")
                for drone in self.drones:
                    drone.stage = 8
                    await drone.land()
                    await drone.off()

                self.drones = []
                break

            await asyncio.sleep(0.1)

    async def start(self):
        
        # Create a task for each drone
        drone_tasks = [asyncio.create_task(self.drone_logic(drone)) for drone in self.drones]

        # Create a task for updating the table
        table_task = asyncio.create_task(self.update_table())

        safety_task = asyncio.create_task(self.safety())

        # Combine all tasks
        all_tasks = [table_task] + drone_tasks + [safety_task]

        # Wait for all tasks to complete
        await asyncio.gather(*all_tasks)

        # Close OpenCV windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    addresses = ['123','123']
    dronesar = DroneSar(
                        image_name = 'test.JPG', 
                        num_drones=1, 
                        addresses=addresses, 
                        bound_x=1.4, 
                        bound_y=2.2, 
                        debug=0, 
                        size_of_blocks=44, 
                        start_matches=100,
                        wanted_height=50,
                        verbose=True,
                        )
    asyncio.run(dronesar.start())
    
    print("Program ended.")
    exit(0)

