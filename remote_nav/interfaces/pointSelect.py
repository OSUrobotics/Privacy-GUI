# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pointSelect.ui'
#
# Created: Fri Jul 25 14:37:47 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Paths(object):
    def setupUi(self, Paths):
        Paths.setObjectName(_fromUtf8("Paths"))
        Paths.resize(805, 616)
        self.horizontalLayout_3 = QtGui.QHBoxLayout(Paths)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.map_view = customView(Paths)
        self.map_view.setObjectName(_fromUtf8("map_view"))
        self.horizontalLayout_3.addWidget(self.map_view)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.new_zone_btn = QtGui.QPushButton(Paths)
        self.new_zone_btn.setObjectName(_fromUtf8("new_zone_btn"))
        self.horizontalLayout.addWidget(self.new_zone_btn)
        self.save_zone_btn = QtGui.QPushButton(Paths)
        self.save_zone_btn.setEnabled(False)
        self.save_zone_btn.setObjectName(_fromUtf8("save_zone_btn"))
        self.horizontalLayout.addWidget(self.save_zone_btn)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.zone_name = QtGui.QLineEdit(Paths)
        self.zone_name.setEnabled(False)
        self.zone_name.setMaxLength(50)
        self.zone_name.setFrame(True)
        self.zone_name.setObjectName(_fromUtf8("zone_name"))
        self.verticalLayout.addWidget(self.zone_name)
        self.privacy_type = QtGui.QComboBox(Paths)
        self.privacy_type.setEnabled(False)
        self.privacy_type.setMaxVisibleItems(10)
        self.privacy_type.setObjectName(_fromUtf8("privacy_type"))
        self.privacy_type.addItem(_fromUtf8(""))
        self.privacy_type.addItem(_fromUtf8(""))
        self.privacy_type.addItem(_fromUtf8(""))
        self.verticalLayout.addWidget(self.privacy_type)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label = QtGui.QLabel(Paths)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout_2.addWidget(self.label)
        self.edit_zone = QtGui.QComboBox(Paths)
        self.edit_zone.setEnabled(False)
        self.edit_zone.setEditable(False)
        self.edit_zone.setObjectName(_fromUtf8("edit_zone"))
        self.horizontalLayout_2.addWidget(self.edit_zone)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout_3.addLayout(self.verticalLayout)

        self.retranslateUi(Paths)
        QtCore.QMetaObject.connectSlotsByName(Paths)

    def retranslateUi(self, Paths):
        Paths.setWindowTitle(QtGui.QApplication.translate("Paths", "Define Zones", None, QtGui.QApplication.UnicodeUTF8))
        self.new_zone_btn.setToolTip(QtGui.QApplication.translate("Paths", "<html><head/><body><p>Create a <span style=\" font-weight:600;\">new zone</span> to draw on the map</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.new_zone_btn.setText(QtGui.QApplication.translate("Paths", "New Zone", None, QtGui.QApplication.UnicodeUTF8))
        self.save_zone_btn.setToolTip(QtGui.QApplication.translate("Paths", "<html><head/><body><p>Save the current zone settings</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.save_zone_btn.setText(QtGui.QApplication.translate("Paths", "Save Zone", None, QtGui.QApplication.UnicodeUTF8))
        self.zone_name.setToolTip(QtGui.QApplication.translate("Paths", "<html><head/><body><p>Enter a name for the zone</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.zone_name.setPlaceholderText(QtGui.QApplication.translate("Paths", "Zone Label", None, QtGui.QApplication.UnicodeUTF8))
        self.privacy_type.setToolTip(QtGui.QApplication.translate("Paths", "<html><head/><body><p>Add a privacy type to the zone. The default is no filter.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.privacy_type.setItemText(0, QtGui.QApplication.translate("Paths", "No Filter", None, QtGui.QApplication.UnicodeUTF8))
        self.privacy_type.setItemText(1, QtGui.QApplication.translate("Paths", "Private", None, QtGui.QApplication.UnicodeUTF8))
        self.privacy_type.setItemText(2, QtGui.QApplication.translate("Paths", "Public", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Paths", "Select a zone to edit:", None, QtGui.QApplication.UnicodeUTF8))
        self.edit_zone.setToolTip(QtGui.QApplication.translate("Paths", "<html><head/><body><p>Bring up the data for already saved zones.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))

from customview import customView

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Paths = QtGui.QWidget()
    ui = Ui_Paths()
    ui.setupUi(Paths)
    Paths.show()
    sys.exit(app.exec_())

