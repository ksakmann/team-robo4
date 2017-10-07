import sys
import time
import numpy as np

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg


class App(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)

        #### Create Gui Elements ###########
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)
        self.mainbox.setLayout(QtGui.QVBoxLayout())

        self.canvas = pg.GraphicsLayoutWidget()
        self.mainbox.layout().addWidget(self.canvas)

        self.label = QtGui.QLabel()
        self.mainbox.layout().addWidget(self.label)

        self.fig = self.canvas.addPlot()
        self.fig.setLabel('bottom', text='X', units='m')
        self.fig.setLabel('left', text='Y', units='m')
        self.fig.showGrid(x=True, y=True, alpha=0.5)

        self.hMap          = self.fig.plot(pen='w' , symbolBrush=None, symbolPen='y', symbol=None, symbolSize=10)
        self.hVehicle      = self.fig.plot(pen=None, symbolBrush='b' , symbolPen='b', symbol='o' , symbolSize=20)
        self.hTrafficLight = self.fig.plot(pen=None, symbolBrush='r' , symbolPen='r', symbol='o' , symbolSize=30)

    def plotMap(self, x, y):
        self.hMap.setData(x, y)

    def plotVehicle(self, x, y):
        self.hVehicle.setData([x], [y])

    def plotTrafficLight(self, x, y, color):
        self.hTrafficLight.setData([x], [y], symbolBrush=color, symbolPen=color)
    
    def plotTrafficLightClear(self):
        self.hTrafficLight.clear()


if __name__ == '__main__':

    print(sys.argv)
    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    sys.exit(app.exec_())