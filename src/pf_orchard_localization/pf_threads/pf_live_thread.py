from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition
import numpy as np
import time
from ..utils.parameters import ParametersPf
from ..pf_engine import PfEngine
import inspect
from ..visual_odom import OpticalFlowOdometerThread
import os

class PfLiveThread(QThread):
    """
    Runs the particle filter algorithm in real-time using live data from the camera
    """
    
    pf_run_message = pyqtSignal(str)
    set_queue_size = pyqtSignal(int)    
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    signal_segmented_image = pyqtSignal(object, int)
    converged_signal = pyqtSignal(bool)

    def __init__(self, 
                 pf_engine: PfEngine, 
                 trunk_data_thread, 
                 stop_when_converged=False
                 ):
        """
        Args:
            pf_engine (PfEngine): The particle filter engine
            trunk_data_thread (TrunkDataConnectionRosSub): The thread that receives the trunk data
            stop_when_converged (bool, optional): If True, the thread will stop when the particle filter converges. Defaults to False.
        """
        
        super().__init__()
        
        self.pf_engine = pf_engine
        self.trunk_data_thread = trunk_data_thread
        self.stop_when_converged = stop_when_converged

        self.fps = int(os.environ.get("IMAGE_FPS"))
        
        self.dt = 1 / self.fps
        
        self.pf_active = False
        self.converged = False
        
        self.segmented_image_display_num = 1
        
        self.tree_image_data_queue = []
        self.odom_data_list = []
        
        self.odom_mutex = QMutex()
        self.odom_wait_condition = QWaitCondition()
        
        self.tree_image_data_mutex = QMutex()
        self.tree_image_data_condition = QWaitCondition()
        
        self.trunk_data_thread.trunk_data_signal.connect(self.trunk_data_reciever)
        self.trunk_data_thread.odom_data_signal.connect(self.odom_data_reciever)
        self.converged_signal.connect(self.trunk_data_thread.publish_converged)
        
    
    def run(self):
            """
            The main loop of the thread, stopped by calling stop_pf()
            """
                        
            self.pf_active = True
            
            # As long as the thread is active, keep processing the data
            while self.pf_active:
                
                # Wait for odom data
                self.odom_mutex.lock()
                if len(self.odom_data_list) == 0:
                    self.odom_wait_condition.wait(self.odom_mutex)
                if not self.pf_active:
                    self.odom_mutex.unlock()
                    break
                odom_data = self.odom_data_list.pop(0)
                self.odom_mutex.unlock()
                
                # Wait for trunk data
                self.tree_image_data_mutex.lock()
                if len(self.tree_image_data_queue) == 0:
                    self.tree_image_data_condition.wait(self.tree_image_data_mutex)
                if not self.pf_active:
                    self.tree_image_data_mutex.unlock()
                    break
                tree_image_data = self.tree_image_data_queue.pop(0)
                self.tree_image_data_mutex.unlock()
                
                # TODO check if the odom or trunk data is None and act accordingly
                
                # TODO I'm not sure this queue size is working right
                self.set_queue_size.emit(len(self.tree_image_data_queue))
                
                # Check if the timestamps are the same, they should be since they are from the same image
                timestamp_odom = odom_data["timestamp"]
                timestamp_tree_data = tree_image_data["timestamp"]
                timestamp_diff = timestamp_odom - timestamp_tree_data
                
                # Odom is newer, so ignore the trunk data and put the odom data back in the queue
                if timestamp_diff > 0.0001:
                    self.odom_mutex.lock()
                    self.odom_data_list.insert(0, odom_data)
                    self.odom_mutex.unlock()
                
                # Trunk data is newer, so handle the odom data and put the trunk data back in the queue
                elif timestamp_diff < -0.0001:
                    self.handle_odom_data(odom_data)
                    self.tree_image_data_mutex.lock()
                    self.tree_image_data_queue.insert(0, tree_image_data)
                    self.tree_image_data_mutex.unlock()
                    
                # The timestamps are the same so we can process the data
                else:
                    seg_img = tree_image_data["seg_img"]
                    
                    self.signal_segmented_image.emit(seg_img, self.segmented_image_display_num)
                    
                    self.handle_odom_data(odom_data)
                    self.handle_tree_image_data(tree_image_data)
                    
                    self.plot_best_guess.emit(self.pf_engine.best_particle)
                    self.plot_particles.emit(self.pf_engine.downsample_particles())
                
                
                self.converged_signal.emit(self.pf_engine.check_convergence())
                
    
    def handle_odom_data(self, odom_data):
        """
        Processes the odometry dat
        
        Args:
            odom_data (dict): The odometry data
        """
        x_odom = odom_data["x_odom"]
        u = np.array([[x_odom/self.dt], [0]])
        num_readings = int(self.dt * 60)
        self.pf_engine.motion_update(u, self.dt, num_readings=num_readings)
    
    def handle_tree_image_data(self, tree_image_data):
        """
        Processes the tree image data

        Args:
            tree_image_data (dict): The tree image data
        """
        trunk_data = tree_image_data["trunk_data"]
        self.pf_engine.scan_update(trunk_data)            
        
    def check_convergence(self):
        """
        Checks if the particle filter has converged, and sets the converged flag
        """
        self.converged = self.pf_engine.check_convergence()
        
        if self.converged and self.stop_when_converged:
            self.pf_active = False
            
    @pyqtSlot(dict)
    def trunk_data_reciever(self, tree_image_data):
        """
        Slot for receiving the trunk data from the ros thread
        """
        self.tree_image_data_mutex.lock()
        self.tree_image_data_queue.append(tree_image_data)
        self.tree_image_data_condition.wakeAll()
        self.tree_image_data_mutex.unlock()
    
    @pyqtSlot(dict)
    def odom_data_reciever(self, odom_data):
        """
        Slot for receiving the odometry data from the ros thread
        """
        self.odom_mutex.lock()
        self.odom_data_list.append(odom_data)
        self.odom_wait_condition.wakeAll()
        self.odom_mutex.unlock()
    
    @pyqtSlot()
    def stop_pf(self):
        """
        Stops the thread and stops any waiting
        """
        self.pf_active = False
    
        self.odom_wait_condition.wakeAll()
        self.tree_image_data_condition.wakeAll()
        