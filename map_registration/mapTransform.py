# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mapTransform.ui'
#
# Created: Thu Jul 24 09:31:28 2014
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Window(object):
    def setupUi(self, Window):
        Window.setObjectName(_fromUtf8("Window"))
        Window.resize(1027, 653)
        self.verticalLayout_4 = QtGui.QVBoxLayout(Window)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.splitter = QtGui.QSplitter(Window)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.source = customView(self.splitter)
        self.source.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.source.setMouseTracking(False)
        self.source.setObjectName(_fromUtf8("source"))
        self.destination = customView(self.splitter)
        self.destination.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.destination.setObjectName(_fromUtf8("destination"))
        self.layoutWidget = QtGui.QWidget(self.splitter)
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.export_btn = QtGui.QPushButton(self.layoutWidget)
        self.export_btn.setObjectName(_fromUtf8("export_btn"))
        self.verticalLayout.addWidget(self.export_btn)
        self.transform_btn = QtGui.QPushButton(self.layoutWidget)
        self.transform_btn.setObjectName(_fromUtf8("transform_btn"))
        self.verticalLayout.addWidget(self.transform_btn)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        self.verticalLayout.addItem(spacerItem)
        self.tools = QtGui.QTabWidget(self.layoutWidget)
        self.tools.setObjectName(_fromUtf8("tools"))
        self.affine_tool = QtGui.QWidget()
        self.affine_tool.setObjectName(_fromUtf8("affine_tool"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.affine_tool)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.affineLayout = QtGui.QVBoxLayout()
        self.affineLayout.setObjectName(_fromUtf8("affineLayout"))
        self.newPt_btn = QtGui.QPushButton(self.affine_tool)
        self.newPt_btn.setObjectName(_fromUtf8("newPt_btn"))
        self.affineLayout.addWidget(self.newPt_btn)
        self.point1 = QtGui.QRadioButton(self.affine_tool)
        self.point1.setAcceptDrops(False)
        self.point1.setStyleSheet(_fromUtf8("color:red"))
        self.point1.setChecked(True)
        self.point1.setObjectName(_fromUtf8("point1"))
        self.buttonGroup = QtGui.QButtonGroup(Window)
        self.buttonGroup.setObjectName(_fromUtf8("buttonGroup"))
        self.buttonGroup.addButton(self.point1, 1)
        self.affineLayout.addWidget(self.point1)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.p1_x1 = QtGui.QLineEdit(self.affine_tool)
        self.p1_x1.setReadOnly(True)
        self.p1_x1.setObjectName(_fromUtf8("p1_x1"))
        self.gridLayout.addWidget(self.p1_x1, 0, 0, 1, 1)
        self.p1_y1 = QtGui.QLineEdit(self.affine_tool)
        self.p1_y1.setReadOnly(True)
        self.p1_y1.setObjectName(_fromUtf8("p1_y1"))
        self.gridLayout.addWidget(self.p1_y1, 0, 1, 1, 1)
        self.p1_x2 = QtGui.QLineEdit(self.affine_tool)
        self.p1_x2.setReadOnly(True)
        self.p1_x2.setObjectName(_fromUtf8("p1_x2"))
        self.gridLayout.addWidget(self.p1_x2, 1, 0, 1, 1)
        self.p1_y2 = QtGui.QLineEdit(self.affine_tool)
        self.p1_y2.setReadOnly(True)
        self.p1_y2.setObjectName(_fromUtf8("p1_y2"))
        self.gridLayout.addWidget(self.p1_y2, 1, 1, 1, 1)
        self.affineLayout.addLayout(self.gridLayout)
        self.point2 = QtGui.QRadioButton(self.affine_tool)
        self.point2.setObjectName(_fromUtf8("point2"))
        self.buttonGroup.addButton(self.point2, 2)
        self.affineLayout.addWidget(self.point2)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.p2_x1 = QtGui.QLineEdit(self.affine_tool)
        self.p2_x1.setReadOnly(True)
        self.p2_x1.setObjectName(_fromUtf8("p2_x1"))
        self.gridLayout_2.addWidget(self.p2_x1, 0, 0, 1, 1)
        self.p2_y1 = QtGui.QLineEdit(self.affine_tool)
        self.p2_y1.setReadOnly(True)
        self.p2_y1.setObjectName(_fromUtf8("p2_y1"))
        self.gridLayout_2.addWidget(self.p2_y1, 0, 1, 1, 1)
        self.p2_x2 = QtGui.QLineEdit(self.affine_tool)
        self.p2_x2.setReadOnly(True)
        self.p2_x2.setObjectName(_fromUtf8("p2_x2"))
        self.gridLayout_2.addWidget(self.p2_x2, 1, 0, 1, 1)
        self.p2_y2 = QtGui.QLineEdit(self.affine_tool)
        self.p2_y2.setReadOnly(True)
        self.p2_y2.setObjectName(_fromUtf8("p2_y2"))
        self.gridLayout_2.addWidget(self.p2_y2, 1, 1, 1, 1)
        self.affineLayout.addLayout(self.gridLayout_2)
        self.point3 = QtGui.QRadioButton(self.affine_tool)
        self.point3.setObjectName(_fromUtf8("point3"))
        self.buttonGroup.addButton(self.point3, 3)
        self.affineLayout.addWidget(self.point3)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.p3_x1 = QtGui.QLineEdit(self.affine_tool)
        self.p3_x1.setReadOnly(True)
        self.p3_x1.setObjectName(_fromUtf8("p3_x1"))
        self.gridLayout_3.addWidget(self.p3_x1, 0, 0, 1, 1)
        self.p3_y1 = QtGui.QLineEdit(self.affine_tool)
        self.p3_y1.setReadOnly(True)
        self.p3_y1.setObjectName(_fromUtf8("p3_y1"))
        self.gridLayout_3.addWidget(self.p3_y1, 0, 1, 1, 1)
        self.p3_x2 = QtGui.QLineEdit(self.affine_tool)
        self.p3_x2.setReadOnly(True)
        self.p3_x2.setObjectName(_fromUtf8("p3_x2"))
        self.gridLayout_3.addWidget(self.p3_x2, 1, 0, 1, 1)
        self.p3_y2 = QtGui.QLineEdit(self.affine_tool)
        self.p3_y2.setReadOnly(True)
        self.p3_y2.setObjectName(_fromUtf8("p3_y2"))
        self.gridLayout_3.addWidget(self.p3_y2, 1, 1, 1, 1)
        self.affineLayout.addLayout(self.gridLayout_3)
        self.verticalLayout_3.addLayout(self.affineLayout)
        self.tools.addTab(self.affine_tool, _fromUtf8(""))
        self.bulge_tool = QtGui.QWidget()
        self.bulge_tool.setObjectName(_fromUtf8("bulge_tool"))
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.bulge_tool)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.bulge_btn = QtGui.QPushButton(self.bulge_tool)
        self.bulge_btn.setObjectName(_fromUtf8("bulge_btn"))
        self.horizontalLayout_7.addWidget(self.bulge_btn)
        self.indent_btn = QtGui.QPushButton(self.bulge_tool)
        self.indent_btn.setObjectName(_fromUtf8("indent_btn"))
        self.horizontalLayout_7.addWidget(self.indent_btn)
        self.verticalLayout_6.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.radius_lbl = QtGui.QLabel(self.bulge_tool)
        self.radius_lbl.setObjectName(_fromUtf8("radius_lbl"))
        self.horizontalLayout_2.addWidget(self.radius_lbl)
        self.bulgeRadius = QtGui.QSlider(self.bulge_tool)
        self.bulgeRadius.setMaximum(255)
        self.bulgeRadius.setSingleStep(5)
        self.bulgeRadius.setSliderPosition(10)
        self.bulgeRadius.setOrientation(QtCore.Qt.Horizontal)
        self.bulgeRadius.setTickPosition(QtGui.QSlider.TicksAbove)
        self.bulgeRadius.setTickInterval(10)
        self.bulgeRadius.setObjectName(_fromUtf8("bulgeRadius"))
        self.horizontalLayout_2.addWidget(self.bulgeRadius)
        self.verticalLayout_6.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.label_2 = QtGui.QLabel(self.bulge_tool)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_8.addWidget(self.label_2)
        self.bulge_x = QtGui.QLineEdit(self.bulge_tool)
        self.bulge_x.setReadOnly(True)
        self.bulge_x.setObjectName(_fromUtf8("bulge_x"))
        self.horizontalLayout_8.addWidget(self.bulge_x)
        self.bulge_y = QtGui.QLineEdit(self.bulge_tool)
        self.bulge_y.setReadOnly(True)
        self.bulge_y.setObjectName(_fromUtf8("bulge_y"))
        self.horizontalLayout_8.addWidget(self.bulge_y)
        self.verticalLayout_6.addLayout(self.horizontalLayout_8)
        self.tools.addTab(self.bulge_tool, _fromUtf8(""))
        self.verticalLayout.addWidget(self.tools)
        self.toggleRobot = QtGui.QCheckBox(self.layoutWidget)
        self.toggleRobot.setObjectName(_fromUtf8("toggleRobot"))
        self.verticalLayout.addWidget(self.toggleRobot)
        self.verticalLayout_4.addWidget(self.splitter)

        self.retranslateUi(Window)
        self.tools.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Window)
        Window.setTabOrder(self.source, self.destination)
        Window.setTabOrder(self.destination, self.export_btn)
        Window.setTabOrder(self.export_btn, self.transform_btn)
        Window.setTabOrder(self.transform_btn, self.newPt_btn)
        Window.setTabOrder(self.newPt_btn, self.point1)
        Window.setTabOrder(self.point1, self.point2)
        Window.setTabOrder(self.point2, self.point3)
        Window.setTabOrder(self.point3, self.p1_x1)
        Window.setTabOrder(self.p1_x1, self.p1_y1)
        Window.setTabOrder(self.p1_y1, self.p2_x1)
        Window.setTabOrder(self.p2_x1, self.p2_y1)
        Window.setTabOrder(self.p2_y1, self.p3_x1)
        Window.setTabOrder(self.p3_x1, self.p3_y1)
        Window.setTabOrder(self.p3_y1, self.toggleRobot)
        Window.setTabOrder(self.toggleRobot, self.bulge_btn)
        Window.setTabOrder(self.bulge_btn, self.indent_btn)
        Window.setTabOrder(self.indent_btn, self.bulgeRadius)
        Window.setTabOrder(self.bulgeRadius, self.bulge_x)
        Window.setTabOrder(self.bulge_x, self.bulge_y)

    def retranslateUi(self, Window):
        Window.setWindowTitle(QtGui.QApplication.translate("Window", "Dialog", None, QtGui.QApplication.UnicodeUTF8))
        self.export_btn.setText(QtGui.QApplication.translate("Window", "Export", None, QtGui.QApplication.UnicodeUTF8))
        self.transform_btn.setText(QtGui.QApplication.translate("Window", "Try Transform", None, QtGui.QApplication.UnicodeUTF8))
        self.newPt_btn.setText(QtGui.QApplication.translate("Window", "Register Points", None, QtGui.QApplication.UnicodeUTF8))
        self.point1.setText(QtGui.QApplication.translate("Window", "Point 1", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_x1.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_y1.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_x2.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_y2.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.point2.setText(QtGui.QApplication.translate("Window", "Point 2", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_x1.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_y1.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_x2.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_y2.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.point3.setText(QtGui.QApplication.translate("Window", "Point 3", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_x1.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_y1.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_x2.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_y2.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.tools.setTabText(self.tools.indexOf(self.affine_tool), QtGui.QApplication.translate("Window", "Affine", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_btn.setText(QtGui.QApplication.translate("Window", "Bulge", None, QtGui.QApplication.UnicodeUTF8))
        self.indent_btn.setText(QtGui.QApplication.translate("Window", "Indent", None, QtGui.QApplication.UnicodeUTF8))
        self.radius_lbl.setText(QtGui.QApplication.translate("Window", "Radius", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Window", "Position", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_x.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_y.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.tools.setTabText(self.tools.indexOf(self.bulge_tool), QtGui.QApplication.translate("Window", "Bulge", None, QtGui.QApplication.UnicodeUTF8))
        self.toggleRobot.setText(QtGui.QApplication.translate("Window", "Show Robot", None, QtGui.QApplication.UnicodeUTF8))

from customview import customView

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Window = QtGui.QDialog()
    ui = Ui_Window()
    ui.setupUi(Window)
    Window.show()
    sys.exit(app.exec_())

