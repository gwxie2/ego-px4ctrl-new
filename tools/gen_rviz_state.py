import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget, QTextEdit
from PyQt5.QtCore import Qt

app = QApplication(sys.argv)
win = QMainWindow()
win.resize(1600, 1000)

dock_displays = QDockWidget("Displays", win)
win.addDockWidget(Qt.LeftDockWidgetArea, dock_displays)

dock_selection = QDockWidget("Selection", win)
win.addDockWidget(Qt.RightDockWidgetArea, dock_selection)

dock_tool = QDockWidget("Tool Properties", win)
win.addDockWidget(Qt.RightDockWidgetArea, dock_tool)

dock_views = QDockWidget("Views", win)
win.addDockWidget(Qt.RightDockWidgetArea, dock_views)

dock_time = QDockWidget("Time", win)
win.addDockWidget(Qt.BottomDockWidgetArea, dock_time)

dock_depth0 = QDockWidget("Depth uav0", win)
win.addDockWidget(Qt.RightDockWidgetArea, dock_depth0)

dock_depth1 = QDockWidget("Depth uav1", win)
win.addDockWidget(Qt.RightDockWidgetArea, dock_depth1)

# Resize to desired width/height. We want left sidebar (Displays) to be thin.
# And depth windows to be larger height-wise.
win.resizeDocks([dock_displays], [250], Qt.Horizontal)
win.resizeDocks([dock_depth0, dock_depth1], [400, 400], Qt.Vertical)

# We can also stack Selection, Tool Prop, Views
win.tabifyDockWidget(dock_selection, dock_tool)
win.tabifyDockWidget(dock_tool, dock_views)

state_hex = win.saveState().toHex().data().decode('ascii')
print(state_hex)
