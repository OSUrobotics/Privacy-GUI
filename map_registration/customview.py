from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import *
from PyQt4.QtCore import * 

#This is a custom graphics view class that allows zooming by using the scroll wheel.
class customView(QGraphicsView):
	def __init__(self, parent=None):
		super(customView, self).__init__(parent)
		# self.setDragMode(QGraphicsView.ScrollHandDrag)
		self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

	def wheelEvent(self, event):
		self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
		scaleFactor = 1.15
		if event.delta() > 0:
			#zoom in
			self.scale(scaleFactor, scaleFactor)
		else:
			#zoom out
			self.scale(1.0 / scaleFactor, 1.0 / scaleFactor)