import threading

import rospy
from typing_extensions import Callable


class MultiThreadedRobot:
    def __init__(self):
        self.processes = []

    def start_process(self, function: Callable, args):
        process = threading.Thread(target=function, args=args, name=function.__name__)
        process.start()
        self.processes.append(process)
        return process

    def end(self, process: threading.Thread):
        process.join()
        if process not in self.processes:
            raise ValueError('Process not found.')
        self.processes.remove(process)
        rospy.loginfo(f'Process {process.name} finished.')

    def wait_for_all_processes(self):
        all_processes = self.processes.copy()
        for process in all_processes:
            self.end(process)
        rospy.loginfo('All processes finished')
