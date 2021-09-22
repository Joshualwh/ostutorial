import numpy as np
import cv2

def get_hole_center2d(image):
  # Process images
  # Read images
  # image = cv2.imread('left_image.png')
  # Blur images
  image_blur = cv2.blur(image, (5,5))
  # Canny Edge Detection 
  image_edges = cv2.Canny(image_blur, 60, 120)
  # Threshold is done to make the image binary
  thresh, image_bw = cv2.threshold(image_edges, 80, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

  contours, hierarchy = cv2.findContours(image_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  contour_image = cv2.cvtColor(image_bw, cv2.COLOR_GRAY2RGB)

  # cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)
  # cv2.imshow('Contours', contour_image)

  for i,cnt in enumerate(contours):
    if len(np.squeeze(cnt)) > 5:
      rect = cv2.minAreaRect(cnt)
      (delta_u, delta_v) = rect[1]
      diameter_pixel = max(delta_u,delta_v)
      circularity = delta_u/delta_v if delta_v > delta_u else delta_v/delta_u
      good_circularity = circularity > 0.8
      good_diameter = diameter_pixel > 30
      if good_circularity and good_diameter:
        cv2.drawContours(contour_image, [cnt], 0, [0,0,255], 2)
        cv2.imshow('image_contours.png',contour_image)
        return True, rect
  return False, False

limage = cv2.imread('left_image.png')
success, lres = get_hole_center2d(limage)

# To stop it from ending directly
cv2.waitKey(0)
cv2.destroyAllWindows()