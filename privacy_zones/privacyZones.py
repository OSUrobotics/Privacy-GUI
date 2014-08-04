#!/usr/bin/env python

## Imports
## ^^^^^^^
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QDialog
from pointSelect import Ui_Paths
from components import *


class MainWindow(QDialog, Ui_Paths):
	#List of zones for later use
	zoneList = []
	#Has the currentZone been saved?
	savedFlag = True
	#Does the zone at the very minimum have a name?
	namedFlag = False
	#Are we editing an existing zone?
	editFlag = False
	def __init__(self, parent=None):
		super(QDialog, self).__init__(parent)
		self.setupUi(self)
		self.scene=DrawMap(QImage("maps/lab_pretty.pgm"), self)
		self.map_view.setScene(self.scene)

		## Signals and Slots
		## ^^^^^^^
		self.new_zone_btn.clicked.connect(self.createZone)
		self.save_zone_btn.clicked.connect(self.onSaveClick)
		self.edit_zone.currentIndexChanged.connect(self.loadZone)

	def onSaveClick(self):
		self.saveZone(self.editFlag)

	def createZone(self):
		if self.savedFlag:
			if not self.zone_name.isEnabled():
				self.enableUI()
			self.zone_name.clear()
			self.currentZone = Zone()
			self.editFlag = False
			self.savedFlag = False
		elif self.editFlag:
			saveplz = QMessageBox()
			saveplz.setText("Please save your zone before making a new one.")
			saveplz.setIcon(QMessageBox.Warning)
			saveplz.exec_()

	def nameZone(self, name):
		# self.currentZone.name = "BillY"
		if (name == "" or name== None): 
			print "No Name"
			return
		self.currentZone.name = name
		self.namedFlag = True

	def saveZone(self, editing):
		#There are two kinds of save modes. The first mode is for a completely new zone
		self.nameZone(self.zone_name.text())
		if not self.namedFlag:
			return
		self.currentZone.mode = self.privacy_type.currentIndex()
		#Happens if the edit flag is false (not currently editing)
		if not editing:
			self.addtoList()
		else:
			i = self.edit_zone.currentIndex()
			self.zoneList[i] = self.currentZone
			self.edit_zone.setItemText(i, self.zoneList[i].name)
		self.zone_name.clear()
		self.savedFlag = True
		self.disableUI()


	def loadZone(self, index):
		self.currentZone = self.zoneList[index]
		self.zone_name.setText(self.currentZone.name)
		self.privacy_type.setCurrentIndex(self.currentZone.mode)
		self.scene.addPolygon(self.zoneList[index].drawPoly())
		self.editFlag = True
		self.enableUI()

	def addtoList(self):
		self.currentZone.import_points(self.scene.getPoints())
		self.zoneList.append(self.currentZone)
		recentIndex = len(self.zoneList) - 1
		self.edit_zone.addItem(self.zoneList[recentIndex].name)
		self.scene.addPolygon(self.zoneList[recentIndex].drawPoly())

	def enableUI(self):
		self.edit_zone.setEnabled(True)
		self.zone_name.setEnabled(True)
		self.privacy_type.setEnabled(True)
		self.save_zone_btn.setEnabled(True)
		self.map_view.setEnabled(True)
	def disableUI(self):
		self.save_zone_btn.setEnabled(False)
		self.zone_name.setEnabled(False)
		self.privacy_type.setEnabled(False)
		self.map_view.setEnabled(False)





if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    window = MainWindow()
    window.resize(1000, 500)
    window.show()
    sys.exit(app.exec_())