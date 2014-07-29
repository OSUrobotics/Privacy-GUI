from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import * 


class customView(QGraphicsView):
	def __init__(self, parent=None):
		super(customView, self).__init__(parent)
		self.setDragMode(QGraphicsView.ScrollHandDrag)
		self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

	def wheelEvent(self, event):
		self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
		scaleFactor = 1.15
		if (False):
			if event.delta = 0:
				#zoom in
				print ("zoom in")
				self.scale(scaleFactor, scaleFactor)
			else:
				#zoom out
				print ("zoom out")
				self.scale(1.0 / scaleFactor, 1.0 / scaleFactor)
		else:
			print  ("Scroll wheel derped")