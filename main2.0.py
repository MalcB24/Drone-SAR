import cv2
from tello import Tello
debug = True

# Initialize the Tello drone
drone = Tello(debug)

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

counter = 0
prev_matches = 0

while True:
    myFrame = drone.get_frame_read()
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
            prev_matches = len(good_matches)
        else:
            print(f"Not enough good matches found({len(matches)}, {len(good_matches)}/5")
            prev_matches = 0

        cv2.imshow('Object Tracking', img)  # Show the current frame

            
                # img_stream = img.copy()
                # print("saving match")
                # # save the image and matches as well as the tello me cordinates
                # matches_found.append({
                #     "img": img_stream,
                #     "matches": good_matches
                # })
                

    if cv2.waitKey(1) & 0xFF == ord('q'):
        if not debug:
            me.land()
        break

cv2.destroyAllWindows()

# Save the matches found
for i, match in enumerate(matches_found):
    cv2.imwrite(f"matches/match{i}.png", match["img"])
    print(f"Match {i} saved.")
