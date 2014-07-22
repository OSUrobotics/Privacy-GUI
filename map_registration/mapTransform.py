# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mapTransform.ui'
#
# Created: Mon Jul 21 12:27:34 2014
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
        Window.resize(1027, 419)
        self.verticalLayout_4 = QtGui.QVBoxLayout(Window)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.splitter = QtGui.QSplitter(Window)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName(_fromUtf8("splitter"))
    ## source - QGraphicsView
        self.source = QtGui.QGraphicsView(self.splitter)
        self.source.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.source.setMouseTracking(False)
        self.source.setObjectName(_fromUtf8("source"))
    ## destination - QGraphicsView
        self.destination = QtGui.QGraphicsView(self.splitter)
        self.destination.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.destination.setObjectName(_fromUtf8("destination"))
        self.widget = QtGui.QWidget(self.splitter)
        self.widget.setObjectName(_fromUtf8("widget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.widget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
    ## export_btn - QPushButton
        self.export_btn = QtGui.QPushButton(self.widget)
        self.export_btn.setObjectName(_fromUtf8("export_btn"))
        self.verticalLayout.addWidget(self.export_btn)
    ## transform_btn - QPushButton
        self.transform_btn = QtGui.QPushButton(self.widget)
        self.transform_btn.setObjectName(_fromUtf8("transform_btn"))
        self.verticalLayout.addWidget(self.transform_btn)
        self.tools = QtGui.QTabWidget(self.widget)
        self.tools.setObjectName(_fromUtf8("tools"))
        self.affine_tool = QtGui.QWidget()
        self.affine_tool.setObjectName(_fromUtf8("affine_tool"))
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.affine_tool)
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
    ## editPt_btn - QPushButton
        self.editPt_btn = QtGui.QPushButton(self.affine_tool)
        self.editPt_btn.setObjectName(_fromUtf8("editPt_btn"))
        self.verticalLayout_3.addWidget(self.editPt_btn)
    ## newPt_btn  - QPushButton
        self.newPt_btn = QtGui.QPushButton(self.affine_tool)
        self.newPt_btn.setObjectName(_fromUtf8("newPt_btn"))
        self.verticalLayout_3.addWidget(self.newPt_btn)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
    ## point1 - QRadioButton
        self.point1 = QtGui.QRadioButton(self.affine_tool)
        self.point1.setStyleSheet(_fromUtf8("color:red"))
        self.point1.setObjectName(_fromUtf8("point1"))
        self.buttonGroup = QtGui.QButtonGroup(Window)
        self.buttonGroup.setObjectName(_fromUtf8("buttonGroup"))
        self.buttonGroup.addButton(self.point1, 1)
        self.verticalLayout_2.addWidget(self.point1)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
    ## p1_x - QLineEdit
        self.p1_x = QtGui.QLineEdit(self.affine_tool)
        self.p1_x.setReadOnly(True)
        self.p1_x.setObjectName(_fromUtf8("p1_x"))
        self.horizontalLayout_6.addWidget(self.p1_x)
    ## p1_y - QLineEdit
        self.p1_y = QtGui.QLineEdit(self.affine_tool)
        self.p1_y.setReadOnly(True)
        self.p1_y.setObjectName(_fromUtf8("p1_y"))
        self.horizontalLayout_6.addWidget(self.p1_y)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
    ## point2 - QRadioButton
        self.point2 = QtGui.QRadioButton(self.affine_tool)
        self.point2.setObjectName(_fromUtf8("point2"))
        self.buttonGroup.addButton(self.point2, 2)
        self.verticalLayout_2.addWidget(self.point2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
    ## p2_x - QLineEdit
        self.p2_x = QtGui.QLineEdit(self.affine_tool)
        self.p2_x.setReadOnly(True)
        self.p2_x.setObjectName(_fromUtf8("p2_x"))
        self.horizontalLayout_3.addWidget(self.p2_x)
    ## p2_y - QLineEdit
        self.p2_y = QtGui.QLineEdit(self.affine_tool)
        self.p2_y.setReadOnly(True)
        self.p2_y.setObjectName(_fromUtf8("p2_y"))
        self.horizontalLayout_3.addWidget(self.p2_y)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
    ## point3 - QRadioButton
        self.point3 = QtGui.QRadioButton(self.affine_tool)
        self.point3.setObjectName(_fromUtf8("point3"))
        self.buttonGroup.addButton(self.point3, 3)
        self.verticalLayout_2.addWidget(self.point3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.p3_x = QtGui.QLineEdit(self.affine_tool)
    ## p3_x - QLineEdit
        self.p3_x.setReadOnly(True)
        self.p3_x.setObjectName(_fromUtf8("p3_x"))
        self.horizontalLayout_4.addWidget(self.p3_x)
    ## p3_y - QLineEdit
        self.p3_y = QtGui.QLineEdit(self.affine_tool)
        self.p3_y.setReadOnly(True)
        self.p3_y.setObjectName(_fromUtf8("p3_y"))
        self.horizontalLayout_4.addWidget(self.p3_y)
        self.verticalLayout_2.addLayout(self.horizontalLayout_4)
        self.verticalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout_5.addLayout(self.verticalLayout_3)
        self.tools.addTab(self.affine_tool, _fromUtf8(""))
        self.bulge_tool = QtGui.QWidget()
        self.bulge_tool.setObjectName(_fromUtf8("bulge_tool"))
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.bulge_tool)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
    ## bulge_btn - QPushButton
        self.bulge_btn = QtGui.QPushButton(self.bulge_tool)
        self.bulge_btn.setObjectName(_fromUtf8("bulge_btn"))
        self.horizontalLayout_7.addWidget(self.bulge_btn)
    ## indent_btn - QPushButton
        self.indent_btn = QtGui.QPushButton(self.bulge_tool)
        self.indent_btn.setObjectName(_fromUtf8("indent_btn"))
        self.horizontalLayout_7.addWidget(self.indent_btn)
        self.verticalLayout_6.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
    ## radius_lbl - QLabel
        self.radius_lbl = QtGui.QLabel(self.bulge_tool)
        self.radius_lbl.setObjectName(_fromUtf8("radius_lbl"))
        self.horizontalLayout_2.addWidget(self.radius_lbl)
    ## bulgeRadius - QSlider
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
    ## bulge_x - QLineEdit
        self.bulge_x = QtGui.QLineEdit(self.bulge_tool)
        self.bulge_x.setReadOnly(True)
        self.bulge_x.setObjectName(_fromUtf8("bulge_x"))
        self.horizontalLayout_8.addWidget(self.bulge_x)
    ## bulge_y - QLineEdit
        self.bulge_y = QtGui.QLineEdit(self.bulge_tool)
        self.bulge_y.setReadOnly(True)
        self.bulge_y.setObjectName(_fromUtf8("bulge_y"))
        self.horizontalLayout_8.addWidget(self.bulge_y)
        self.verticalLayout_6.addLayout(self.horizontalLayout_8)
        self.tools.addTab(self.bulge_tool, _fromUtf8(""))
        self.verticalLayout.addWidget(self.tools)
    ## toggleRobot - QCheckbox
        self.toggleRobot = QtGui.QCheckBox(self.widget)
        self.toggleRobot.setObjectName(_fromUtf8("toggleRobot"))
        self.verticalLayout.addWidget(self.toggleRobot)
        self.verticalLayout_4.addWidget(self.splitter)

        self.retranslateUi(Window)
        self.tools.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Window)
        Window.setTabOrder(self.source, self.destination)
        Window.setTabOrder(self.destination, self.export_btn)
        Window.setTabOrder(self.export_btn, self.transform_btn)
        Window.setTabOrder(self.transform_btn, self.tools)
        Window.setTabOrder(self.tools, self.editPt_btn)
        Window.setTabOrder(self.editPt_btn, self.newPt_btn)
        Window.setTabOrder(self.newPt_btn, self.point1)
        Window.setTabOrder(self.point1, self.point2)
        Window.setTabOrder(self.point2, self.point3)
        Window.setTabOrder(self.point3, self.p1_x)
        Window.setTabOrder(self.p1_x, self.p1_y)
        Window.setTabOrder(self.p1_y, self.p2_x)
        Window.setTabOrder(self.p2_x, self.p2_y)
        Window.setTabOrder(self.p2_y, self.p3_x)
        Window.setTabOrder(self.p3_x, self.p3_y)
        Window.setTabOrder(self.p3_y, self.toggleRobot)
        Window.setTabOrder(self.toggleRobot, self.bulge_btn)
        Window.setTabOrder(self.bulge_btn, self.indent_btn)
        Window.setTabOrder(self.indent_btn, self.bulgeRadius)
        Window.setTabOrder(self.bulgeRadius, self.bulge_x)
        Window.setTabOrder(self.bulge_x, self.bulge_y)

    def retranslateUi(self, Window):
        Window.setWindowTitle(QtGui.QApplication.translate("Window", "Dialog", None, QtGui.QApplication.UnicodeUTF8))
        self.export_btn.setText(QtGui.QApplication.translate("Window", "Export", None, QtGui.QApplication.UnicodeUTF8))
        self.transform_btn.setText(QtGui.QApplication.translate("Window", "Try Transform", None, QtGui.QApplication.UnicodeUTF8))
        self.editPt_btn.setText(QtGui.QApplication.translate("Window", "Edit Point", None, QtGui.QApplication.UnicodeUTF8))
        self.newPt_btn.setText(QtGui.QApplication.translate("Window", "New Point", None, QtGui.QApplication.UnicodeUTF8))
        self.point1.setText(QtGui.QApplication.translate("Window", "Point 1", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_x.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p1_y.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.point2.setText(QtGui.QApplication.translate("Window", "Point 2", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_x.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p2_y.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.point3.setText(QtGui.QApplication.translate("Window", "Point 3", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_x.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.p3_y.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.tools.setTabText(self.tools.indexOf(self.affine_tool), QtGui.QApplication.translate("Window", "Affine", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_btn.setText(QtGui.QApplication.translate("Window", "Bulge", None, QtGui.QApplication.UnicodeUTF8))
        self.indent_btn.setText(QtGui.QApplication.translate("Window", "Indent", None, QtGui.QApplication.UnicodeUTF8))
        self.radius_lbl.setText(QtGui.QApplication.translate("Window", "Radius", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Window", "Position", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_x.setPlaceholderText(QtGui.QApplication.translate("Window", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.bulge_y.setPlaceholderText(QtGui.QApplication.translate("Window", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.tools.setTabText(self.tools.indexOf(self.bulge_tool), QtGui.QApplication.translate("Window", "Bulge", None, QtGui.QApplication.UnicodeUTF8))
        self.toggleRobot.setText(QtGui.QApplication.translate("Window", "Show Robot", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    Window = QtGui.QDialog()
    ui = Ui_Window()
    ui.setupUi(Window)
    Window.show()
    sys.exit(app.exec_())

