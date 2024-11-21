import threading

from typing_extensions import Callable

class MultiThreadedRobot:
    def __init__(self):
        self.processes = []
    
    def start_process(self, function: Callable, args):
        process = threading.Thread(target=function, args=args)
        process.start()
        self.processes.append(process)
        return process
    
    def end(self, process):
        process.join()
        if process not in self.processes:
            raise ValueError('Process not found.')
        self.processes.remove(process)
        