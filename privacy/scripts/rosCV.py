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
	def imshow(self,image, title="image"):
		cv2.imshow(title, image)
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

# 	#Pass it the image and the haar xml file and it finds faces accordingly. Not working very well right now.
	def findFaces(self, image, scale=1, haar="/nfs/attic/smartw/users/reume02/catkin_ws/src/privacy-interfaces/privacy/config/haar/haarcascade_frontalface_default.xml"):
		#Factor by which we scale the image down in order for faster processing.

		cascade = cv2.CascadeClassifier(haar)
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		img = cv2.resize(image_gray,(0,0),fx=scale,fy=scale)
		rects = cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=1,
flags=cv.CV_HAAR_SCALE_IMAGE)
		# for x, y, w, h in rects:
		# 	x = x * (1/scale)
		# 	y = y * (1/scale)
		# 	w = w * (1/scale)
		# 	h = h * (1/scale)
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
	#takes a cv2 nupy array and two points and performs an inpainting filter. Takes a mask optionally
	def inpaint(self, image, topLeft=None, bottomRight=None,mask=None):
		if(mask is None):
			x1 = topLeft[0]
			x2 = bottomRight[0]

			y1 = topLeft[1]
			y2 = bottomRight[1]

			mask = numpy.zeros((image.shape[0], image.shape[1]), numpy.uint8)
			mask[y1:y2, x1:x2] = 1
			
			image = cv2.inpaint(image, mask, 3, cv2.INPAINT_TELEA)
		else:
			image = cv2.inpaint(image, mask, 3, cv2.INPAINT_NS)
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

