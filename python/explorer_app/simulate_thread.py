from PySide6 import QtCore
import time


class SimulateThread(QtCore.QThread):
    def __init__(self, pathfinder, host_widget, delta_time: float = 0.1):
        self._host = host_widget
        self._pathfinder = pathfinder
        self._is_active = False
        self._delta_time = delta_time
        super(SimulateThread, self).__init__()

    def run(self):
        while True:
            if self._is_active:
                self._pathfinder.update()
            else:
                self._pathfinder.update_time()
            self._host.update_agents(self._pathfinder.get_all_agents_positions(), self._pathfinder.get_all_agents_paths(), self._pathfinder.get_all_agents_activities())
            self._is_active = not (self._pathfinder.get_active_agents_count() == 0)
            time.sleep(self._delta_time)
