import cv2
import numpy as np

# 1. Load the image
image = cv2.imread("object.png")

# 2. Convert from BGR to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 3. Define range for the target color (e.g., Green)
# Note: OpenCV Hue range is 0-180, Saturation 0-255, Value 0-255
# A tight, accurate range for this specific dark purple
lower_color = np.array([130, 90, 30])
upper_color = np.array([155, 180, 95])

# 4. Threshold the HSV image to get only target colors
mask = cv2.inRange(hsv, lower_color, upper_color)
cv2.imshow("mask1", mask)
# 5. Clean up noise using morphological operations
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Removes small dots
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fills tiny holes

# 6. Find contours (the color blobs)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 7. Iterate through and filter/draw blobs
for contour in contours:
    # Filter out tiny noise blobs by areakkkkkkkkkkkk
    area = cv2.contourArea(contour)
    if area > 100:  # Minimum pixel area threshold
        # Calculate the center (centroid) of the blob
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draw a bounding rectangle or contour outline
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

# Display results
cv2.imshow("Mask", mask)
cv2.imshow("Detected Blobs", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
