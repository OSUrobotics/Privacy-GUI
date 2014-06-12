from cv_bridge import CvBridge
import cv, cv2
import numpy

#Class that takes in 
class rosCV():
#BRIDGE/GENERAL FUNCTIONS -----
	def __init__(self):
		self.bridge = CvBridge()
	#converts ROS image to cv2 numpy array.
	def toCv2(self, image_in):
		image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
		image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
		return image_cv2
	
	#converts cv2 numpy array to ROS image.
	def toRos(self,image_cv2): 
		image_cv = cv.fromarray(image_cv2)
		image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8')   
		return image_out
	def imshow(self,image):
		cv2.imshow("image", image)
		cv2.waitKey(3)
#Contour Stuff ---------

	#returns an array of contours in an image of matched colors within the range (lowerb, upperb)
	def colorContours(self, image, lowerb, upperb):
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		image_hsv = cv2.blur(image_hsv, (5, 5))
		is_color = cv2.inRange(image_hsv, lowerb, upperb)
		contours, hierarchy = cv2.findContours(is_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		return contours
#Facial recognition stuff

	#Pass it the image and the haar xml file and it finds faces accordingly.
	def findFaces(self, image, haar="/nfs/attic/smartw/users/reume02/catkin_ws/src/privacy-interfaces/privacy/config/haar/haarcascade_frontalface_alt.xml"):
		cascade = cv2.CascadeClassifier(haar)
		rects = cascade.detectMultiScale(image, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))
		return rects


#IMAGE MANIPULATION FUNCTIONS
	#takes a cv2 numpy array and two 2-tuples and creates a black box.
	def redact(self, image, topLeft, bottomRight):
		cv2.rectangle(image, topLeft, bottomRight, -1, -1)
		return image
	def npr(self, image, topLeft, bottomRight):
		x1 = topLeft[0]
		x2 = bottomRight[0]

		y1 = topLeft[1]
		y2 = bottomRight[1]
		try:
			crop_image = image[y1:y2, x1:x2]
			if crop_image.size > 0:
				#crop_image = cv2.medianBlur(crop_image, 5)
				for i in range(2):
					crop_image = cv2.bilateralFilter(crop_image, 3, 10, 10)
					crop_image = cv2.pyrMeanShiftFiltering(crop_image, 7, 20)
			image[y1:y2, x1:x2] = crop_image[:,:]
			#TypeError, ValueError
		except (TypeError):
			pass
		return image
	#takes a cv2 nupy array and two points and performs an inpainting filter.
	def inpaint(self, image, topLeft, bottomRight):
		x1 = topLeft[0]
		x2 = bottomRight[0]

		y1 = topLeft[1]
		y2 = bottomRight[1]

		mask = numpy.zeros((image.shape[0], image.shape[1]), numpy.uint8)
		mask[y1:y2, x1:x2] = 1
		
		image = cv2.inpaint(image, mask, 3, cv2.INPAINT_TELEA)
		return image
	#performs a blur operation on the selected square.
	def blur(self, image, topLeft, bottomRight):
		x1 = topLeft[0]
		y1 = topLeft[1]
		x2 = bottomRight[0]
		y2 = bottomRight[1]
		try:
			crop_image = image[y1:y2, x1:x2]
			crop_image = cv2.GaussianBlur(crop_image,(11,11), 100, 100)
			image[y1:y2, x1:x2] = crop_image[:,:]
		except (TypeError):
			pass
		return image

