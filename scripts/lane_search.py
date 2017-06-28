from __future__ import print_function

import cv2
import numpy as np
from collections import deque

# Most thanks to https://github.com/jasonhuh/CarND-Advanced-Lane-Lines/blob/master/lib/carlane.py

class LaneSearch:

	search_window_x_margin = 100
	search_poly_x_margin   = 40
	minpix_window          = 100
	minpix_full            = 1000
	
	def __init__(self):
		self.visualize_window = 'lane_search'
		self.visualize_delay = 1
		# self.visualize = True
		self.visualize = False
		self.middled = False

		self.counter = 0 # Tick tick tick
		self.left = Line()
		self.right = Line()
		self.vehicle_position = 0.0
		self.radius_of_curvature = 0.0
		self.vehicle_position_text = '' # TODO: Move to a separate view model class
		self.radius_of_curvature_text = '' # TODO: Move to a separate view model class

	def find_lane_pixels(self, binary_image):
		self.counter += 1
		# print('Check for pixels')
		# if self.left.line_fit is None or self.right.line_fit is None:
		if not self.left.detected or not self.right.detected:
			print('Not detected')
			return self.__find_lane_pixels_scan_window(binary_image)
		# axis line is in the middle
		elif self.middled or np.array_equal(self.left.allx, self.right.allx):
			print('Middled')
			return self.__find_lane_pixels_scan_window(binary_image)
		else:
			print('Fast')
			return self.__find_lane_pixels_fast(binary_image, self.left.line_fit, self.right.line_fit)

	def get_lane_polynomials(self, binary_image):
		leftx, lefty, rightx, righty = self.find_lane_pixels(binary_image)

		if self.visualize:
			draw_frame = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
			for i in range(len(leftx)):
				cv2.circle(draw_frame, (int(leftx[i]), int(lefty[i])), 1, (255, 255, 0))

			for i in range(len(rightx)):
				cv2.circle(draw_frame, (int(rightx[i]), int(righty[i])), 1, (255, 0, 255))

			cv2.imshow(self.visualize_window,draw_frame)
			cv2.waitKey(self.visualize_delay)

		left_fitx  = self.left.polyfit_lines(leftx, lefty, binary_image.shape)
		right_fitx = self.right.polyfit_lines(rightx, righty, binary_image.shape)

		return left_fitx, right_fitx, self.right.ploty


	def __find_lane_pixels_scan_window(self, binary_image):
		# Take a histogram of the bottom half of the image
		histogram = np.sum(binary_image[binary_image.shape[0] / 2:, :], axis=0)
		# Find the peak of the left and right halves of the histogram
		# These will be the starting point for the left and right lines
		midpoint = np.int(histogram.shape[0] / 2)
		leftx_base = np.argmax(histogram[:midpoint])
		rightx_base = np.argmax(histogram[midpoint:]) + midpoint

		# Set height of windows
		nwindows = 10  # Choose the number of sliding windows
		window_height = np.int(binary_image.shape[0] / nwindows)
		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = binary_image.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		# Current positions to be updated for each window
		leftx_current = leftx_base
		rightx_current = rightx_base

		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []
		right_lane_inds = []

		same_square_counter = 0

		# Step through the windows one by one
		for window in range(nwindows):
			# Identify window boundaries in x and y (and right and left)
			win_y_low = binary_image.shape[0] - (window + 1) * window_height
			win_y_high = binary_image.shape[0] - window * window_height
			win_xleft_low = leftx_current - LaneSearch.search_window_x_margin
			win_xleft_high = leftx_current + LaneSearch.search_window_x_margin
			win_xright_low = rightx_current - LaneSearch.search_window_x_margin
			win_xright_high = rightx_current + LaneSearch.search_window_x_margin

			# print()
			if abs(rightx_current - leftx_current) < 3:
				same_square_counter += 1

			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
			nonzerox < win_xleft_high)).nonzero()[0]
			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
			nonzerox < win_xright_high)).nonzero()[0]
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			right_lane_inds.append(good_right_inds)
			# If you found > minpix pixels, recenter next window on their mean position
			if self.visualize:
				draw_frame = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)

			if len(good_left_inds) > LaneSearch.minpix_window:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
				if self.visualize: cv2.rectangle(draw_frame, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
			else:
				# print('No left pixels')
				if self.visualize: cv2.rectangle(draw_frame, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 0, 255), 2)

			if len(good_right_inds) > LaneSearch.minpix_window:
				rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
				if self.visualize: cv2.rectangle(draw_frame, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
			else:
				# print('No right pixels')
				if self.visualize: cv2.rectangle(draw_frame, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 0, 255), 2)

			if self.visualize:
				cv2.imshow(self.visualize_window,draw_frame)
				if cv2.waitKey(self.visualize_delay) == ord('q'):
					exit(1)

		# Concatenate the arrays of indices
		left_lane_inds = np.concatenate(left_lane_inds)
		right_lane_inds = np.concatenate(right_lane_inds)

		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]

		# print(len(leftx), len(rightx))

		# TODO: Recover
		self.left.detected = len(leftx) > LaneSearch.minpix_full
		self.right.detected = len(rightx) > LaneSearch.minpix_full

		self.middled = same_square_counter > nwindows / 2

		return leftx, lefty, rightx, righty

	def __find_lane_pixels_fast(self, binary_image, left_fit, right_fit):
		""" Find lane line pixels efficiently by skipping the sliding windows step """

		img_height, img_width = binary_image.shape

		nonzero = binary_image.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])

		left_border_lpoly  = (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] - LaneSearch.search_poly_x_margin)
		right_border_lpoly = (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy + left_fit[2] + LaneSearch.search_poly_x_margin)

		left_border_rpoly  = (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] - LaneSearch.search_poly_x_margin)
		right_border_rpoly = (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy + right_fit[2] + LaneSearch.search_poly_x_margin)

		left_lane_inds  = ( (nonzerox > left_border_lpoly) & (nonzerox < right_border_lpoly) )
		right_lane_inds = ( (nonzerox > left_border_rpoly) & (nonzerox < right_border_rpoly) )
		
		if self.visualize:
			draw_frame = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
			for i in range(len(nonzeroy)):
				cv2.circle(draw_frame, (int(left_border_lpoly[i]), int(nonzeroy[i])), 1, (255, 0, 0))
				cv2.circle(draw_frame, (int(right_border_lpoly[i]), int(nonzeroy[i])), 1, (255, 0, 0))
				cv2.circle(draw_frame, (int(left_border_rpoly[i]), int(nonzeroy[i])), 1, (255, 0, 0))
				cv2.circle(draw_frame, (int(right_border_rpoly[i]), int(nonzeroy[i])), 1, (255, 0, 0))

			cv2.imshow(self.visualize_window,draw_frame)
			if cv2.waitKey(self.visualize_delay) == ord('q'):
				exit(1)

		if not self.right.detected or not self.left.detected:
			print('Achtung!!!!')

		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]
		rightx = nonzerox[right_lane_inds]
		righty = nonzeroy[right_lane_inds]

		# TODO: Recover
		self.left.detected = len(leftx) > LaneSearch.minpix_full
		self.right.detected = len(rightx) > LaneSearch.minpix_full

		# print(len(leftx), len(rightx))

		return leftx, lefty, rightx, righty


class Line:

	MAX_QUEUE_LENGTH = 3

	def __init__(self):
		self.detected = False
		self.radius_of_curvature = None
		#distance in meters of vehicle center from the line
		self.line_base_pos = None
		#difference in fit coefficients between last and new fits
		self.diffs = np.array([0,0,0], dtype='float')
		#x values for detected line pixels
		self.allx = None
		#y values for detected line pixels
		self.ally = None

		self.line_fit = None # Store the line fit

		# Store recent polynomial coefficients for averaging across frames
		self.line_fit0_queue = deque(maxlen=Line.MAX_QUEUE_LENGTH)
		self.line_fit1_queue = deque(maxlen=Line.MAX_QUEUE_LENGTH)
		self.line_fit2_queue = deque(maxlen=Line.MAX_QUEUE_LENGTH)

		self.ploty = None

	def polyfit_lines(self, allx, ally, image_shape):
		""" Find the polynomial fit based on the discovered line points coordinates """

		#flatten = lambda l: [item for sublist in l for item in sublist]

		if len(allx) > 0 and len(ally) > 0:
			self.allx = allx
			self.ally = ally

			# Fit a second order polynomial to each
			self.line_fit = np.polyfit(ally, allx, 2)
			self.line_fit0_queue.append(self.line_fit[0])
			self.line_fit1_queue.append(self.line_fit[1])
			self.line_fit2_queue.append(self.line_fit[2])
		else: # Recover the missing x or y using the previous polyfit data
			print('Recover using previous points')
			self.line_fit = [np.mean(self.line_fit0_queue), np.mean(self.line_fit1_queue), np.mean(self.line_fit2_queue)]

		if self.ploty is None:
			self.ploty = np.linspace(0, image_shape[0] - 1, image_shape[0])
		line_fitx = self.line_fit[0] * self.ploty ** 2 + self.line_fit[1] * self.ploty + self.line_fit[2]
		line_fitx_int = self.line_fit[0] * image_shape[0] ** 2 + self.line_fit[1] * image_shape[0] + self.line_fit[2]

		# y_eval = np.max(self.ploty)

		# # Fit new polynomials to x,y in world space
		# line_fit_cr = np.polyfit(self.ally * CarLane.ym_per_pix, self.allx * CarLane.xm_per_pix, 2)
		# #     # Calculate the new radii of curvature
		# line_curverad = ((1 + (2 * line_fit_cr[0] * y_eval * CarLane.ym_per_pix + line_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * line_fit_cr[0])
		# self.radius_of_curvature = line_curverad
		self.line_base_pos = line_fitx_int

		return line_fitx