#!/usr/bin/env python
# Created by Suzanne Dazo for Oregon State University

## Imports
## ^^^^^^^
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QDialog
from privacy_zones.pointSelect import Ui_Paths
from privacy_zones.components import *
from os import path
import os
import yaml


class MainWindow(QDialog, Ui_Paths):
	
	namedFlag = False	#Does the zone at the very minimum have a name?
	savedFlag = True	#Has the currentZone been saved?
	editFlag = False	#Are we editing an already existing zone?

	def __init__(self, parent=None, argv=[]):
		super(QDialog, self).__init__(parent)
		self.setupUi(self)
		self.controller = ZoneHandler() #Deals with the multiple zones and selection
		mapPath = self.importMap()
		# mapPath = '/home/lazewatd/wheelchair_ws/src/Privacy-GUI/privacy_zones/maps/betterMap.pgm'
		# mapInfoPath = '/home/lazewatd/wu_maps/jolley4.yaml'
		# with open(mapInfoPath, 'r') as map_yaml:
		# 	mapInfo = yaml.load(map_yaml)
		# mapPath = os.path.join(os.path.split(mapInfoPath)[0], mapInfo['image'])
		self.scene = DrawMap(QImage(mapPath), self.controller, self)
		self.map_view.setScene(self.scene)

		## Signals and Slots
		## ^^^^^^^
		self.new_zone_btn.clicked.connect(self.createZone)
		self.save_zone_btn.clicked.connect(self.onSaveClick)
		self.edit_zone.currentRowChanged.connect(self.loadZone)
		self.export_btn.clicked.connect(self.showSaveAs)
		self.import_btn.clicked.connect(self.showOpen)
		self.delete_zone_btn.clicked.connect(self.delete_zone)

		self.delete_zone_btn.setEnabled(False)

	# Called when delete zone button is pressed. Deletes the currently active zone
	def delete_zone(self):
		self.scene.removeItem(self.currentZone)
		self.controller.zoneList.remove(self.currentZone)
		self.edit_zone.takeItem(self.edit_zone.currentRow())
		self.scene.update()
		self.zone_name.clear()
		self.editFlag = False
		self.savedFlag = True

	# What happens when you click the save button
	def onSaveClick(self):
		self.saveZone(self.editFlag)
		self.createZone()
		self.edit_zone.clearSelection()

	# Creates a new zone as long as the current zone has been saved
	# Creates an alert if the zone has not been saved.
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

	# Takes a name value and if it is a proper value, stores it in the current zone
	#and sets the flag to true.
	def nameZone(self, name):
		# self.currentZone.name = "BillY"
		if (name == "" or name== None): 
			print "No Name"
			return
		self.currentZone.name = name
		self.namedFlag = True

	# Stores all of the newly edited values (name, privacy type, points) into the zone object
	# and either updates the zone object in the list or 
	def saveZone(self, editing):
		#There are two kinds of save modes. The first mode is for a completely new zone
		self.nameZone(self.zone_name.text())
		if not self.namedFlag:
			return
		self.currentZone.mode = self.privacy_type.currentIndex()
		#Happens if the edit flag is false (not currently editing)
		if not editing:
			self.addtoList()
			self.editFlag = True
			index = len(self.controller.zoneList) - 1
		else:
			i = self.edit_zone.currentIndex().row()
			self.updateList(i)
		#self.zone_name.clear()
		self.savedFlag = True
		#self.disableUI()
		self.delete_zone_btn.setEnabled(True)

	# Sets the current zone to the specified object in the zone list
	# populates the 
	def loadZone(self, index):
		if len(self.controller.zoneList) == 0:
			self.delete_zone_btn.setEnabled(False)
			self.disableUI()
			return
		self.currentZone = self.controller.zoneList[index]
		self.zone_name.setText(self.currentZone.name)
		self.privacy_type.setCurrentIndex(self.currentZone.mode)
		self.controller.setActiveZone(index)
		# self.controller.zoneList[index].drawPoly()
		self.scene.update()
		self.editFlag = True
		self.enableUI()

	#Changes the data of the zoneList at a given index
	#Updates the zone's name, mode, and list of points
	# It also redraws the polygon into the graphics scene
	def updateList(self, index):
		self.currentZone.import_points(self.controller.getPoints())
		self.controller.zoneList[index] = self.currentZone
		# self.edit_zone.setItemText(index, self.controller.zoneList[index].name)
		self.edit_zone.currentItem().setText(self.controller.zoneList[index].name)
		# self.scene.addItem(self.controller.zoneList[i])
		self.controller.setActiveZone(index)
		self.scene.update()

	# Adds a new zone to the ZoneList and also adds the zone to the edit dropdown
	def addtoList(self):
		self.currentZone.import_points(self.controller.getPoints())
		self.controller.zoneList.append(self.currentZone)
		recentIndex = len(self.controller.zoneList) - 1
		self.edit_zone.addItem(self.controller.zoneList[recentIndex].name)
		self.scene.addItem(self.controller.zoneList[recentIndex])
		self.controller.setActiveZone(recentIndex)
		self.scene.update()

	#Enables various UI Elements
	def enableUI(self):
		self.edit_zone.setEnabled(True)
		self.zone_name.setEnabled(True)
		self.privacy_type.setEnabled(True)
		self.save_zone_btn.setEnabled(True)
		self.map_view.setEnabled(True)

	#Disables various UI Elements
	def disableUI(self):
		self.save_zone_btn.setEnabled(False)
		self.zone_name.setEnabled(False)
		self.privacy_type.setEnabled(False)
		self.map_view.setEnabled(False)

	def showOpen(self):
		fname = QFileDialog.getOpenFileName(self, 'Open File', "", 'YAML Files (*.yaml)')  
		if fname.isEmpty():
			return
		fin = open(fname, 'r')	    
		with fin:        
			# self.import_data = fin.read()
			myYaml = yaml.safe_load(fin)
			text = ""
			spacer = " \n"
			for i in range(0, len(myYaml['Zone List'])):
				text += myYaml['Zone List'][i]['Name'] + spacer
				text += self.privacyMode(myYaml['Zone List'][i]['Mode']) + spacer
				for j in range(0, len(myYaml['Zone List'][i]['Points'])):
					text += str(myYaml['Zone List'][i]['Points'][j]) + spacer
			print text

			# if self.controller.zoneList.isEmpty():	#If this is a new program, import all of the zones here
			for i in range (0, len(myYaml['Zone List'])):
				name = myYaml['Zone List'][i]['Name']
				mode = myYaml['Zone List'][i]['Mode']
				pointList = []
				for j in range(0, len(myYaml['Zone List'][i]['Points'])):
					x = myYaml['Zone List'][i]['Points'][j]['x']
					y = myYaml['Zone List'][i]['Points'][j]['y']
					point = [x, y]
					pointList.append(point)
				self.controller.importZone(name, mode, pointList)
				recentIndex = len(self.controller.zoneList) - 1
				self.edit_zone.addItem(self.controller.zoneList[recentIndex].name)
				self.scene.addItem(self.controller.zoneList[recentIndex])
				self.controller.setActiveZone(recentIndex)
				self.scene.update()


			#Import stuff from the yaml file here
		fin.close()	
	def showSaveAs(self):
		filename = QFileDialog.getSaveFileName(self, 'Save As...', '', '*.yaml')
		if not filename.endsWith(".yaml"):
			filename.append(".yaml")
		fout = open(filename, 'w')
		# Do stuff to write to the file here
		# # Format:
		# Image: 
		# Zone_name: 
		# mode: 
		# points: [an array of all of the points goes here]
		# ^^^ that part will loop for all of the points.

		data = self.controller.exportAll()
		fout.write(data)
		fout.close()

	def importMap(self):
		fname = QFileDialog.getOpenFileName(self, 'Open Map File', "", 'Images (*.pgm *.png *.bmp *.jpg *.jpeg *.gif)')  
		if fname.isEmpty():
			return "maps/floorplan.png"
		else:
			return fname
	def privacyMode(self, mode):
		if mode == 1:
			return "Private"
		elif mode == 2:
			return "Public"
		else: 
			return "No Filter"



if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    window = MainWindow(argv=sys.argv)
    window.resize(1000, 500)
    window.show()
    sys.exit(app.exec_())