import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QDockWidget
from PyQt5.QtCore import Qt

app = QApplication([""])  # True argument is needed by older versions sometimes, or just pass empty array
win = QMainWindow()
win.resize(1920, 1080)

def add_dock(name, area):
    d = QDockWidget(name, win)
    d.setObjectName(name)
    win.addDockWidget(area, d)
    return d

dock_displays = add_dock("Displays", Qt.LeftDockWidgetArea)
dock_selection = add_dock("Selection", Qt.RightDockWidgetArea)
dock_tool = add_dock("Tool Properties", Qt.RightDockWidgetArea)
dock_views = add_dock("Views", Qt.RightDockWidgetArea)
dock_time = add_dock("Time", Qt.BottomDockWidgetArea)
dock_depth0 = add_dock("Depth uav0", Qt.RightDockWidgetArea)
dock_depth1 = add_dock("Depth uav1", Qt.RightDockWidgetArea)
dock_depth2 = add_dock("Depth uav2", Qt.RightDockWidgetArea)

# Extremely small left menu
win.resizeDocks([dock_displays], [120], Qt.Horizontal)

win.splitDockWidget(dock_selection, dock_depth0, Qt.Vertical)
win.splitDockWidget(dock_depth0, dock_depth1, Qt.Vertical)
win.splitDockWidget(dock_depth1, dock_depth2, Qt.Vertical)

win.tabifyDockWidget(dock_selection, dock_tool)
win.tabifyDockWidget(dock_tool, dock_views)

# Set their heights
win.resizeDocks([dock_depth0, dock_depth1, dock_depth2], [300, 300, 300], Qt.Vertical)

state_hex = win.saveState().toHex().data().decode('ascii')
print(state_hex)
