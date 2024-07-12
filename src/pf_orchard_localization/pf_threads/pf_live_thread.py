from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMutex, QWaitCondition
import numpy as np
import time
from ..utils.parameters import ParametersPf
from ..pf_engine import PFEngine
import inspect
from ..visual_odom import OpticalFlowOdometerThread

class PfLiveThread(QThread):
    
    pf_run_message = pyqtSignal(str)
    set_queue_size = pyqtSignal(int)    
    plot_best_guess = pyqtSignal(np.ndarray)
    plot_particles = pyqtSignal(np.ndarray)
    signal_segmented_image = pyqtSignal(object, int)
    converged_signal = pyqtSignal(bool)
    
    def __init__(self, 
                 pf_engine: PFEngine, 
                 trunk_data_thread, 
                 stop_when_converged=False, 
                 fps=10):
        
        super().__init__()
        
        self.pf_engine = pf_engine
        self.trunk_data_thread = trunk_data_thread
        self.stop_when_converged = stop_when_converged
        self.fps = fps
        
        self.dt = 1 / self.fps
        
        self.is_processing = False
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
        
    
    def run(self):
                        
            self.pf_active = True
            
            while self.pf_active:
                
                self.odom_mutex.lock()
                if len(self.odom_data_list) == 0:
                    self.odom_wait_condition.wait(self.odom_mutex)
                if not self.pf_active:
                    self.odom_mutex.unlock()
                    break
                odom_data = self.odom_data_list.pop(0)
                self.odom_mutex.unlock()
                
                self.tree_image_data_mutex.lock()
                if len(self.tree_image_data_queue) == 0:
                    self.tree_image_data_condition.wait(self.tree_image_data_mutex)
                if not self.pf_active:
                    self.tree_image_data_mutex.unlock()
                    break
                tree_image_data = self.tree_image_data_queue.pop(0)
                self.tree_image_data_mutex.unlock()
                
                # TODO check if the odom or trunk data is None and act accordingly
                
                self.set_queue_size.emit(len(self.tree_image_data_queue))
                
                timestamp_odom = odom_data["timestamp"]
                timestamp_tree_data = tree_image_data["timestamp"]
                timestamp_diff = timestamp_odom - timestamp_tree_data
                
                # Odom is newer
                if timestamp_diff > 0.0001:
                    self.handle_odom_data(odom_data)
                    # add the trunk data back to the top of the queue
                    self.tree_image_data_mutex.lock()
                    print("THIS IS WEIRD 12543") # delete this
                    self.tree_image_data_queue.insert(0, tree_image_data)
                    self.tree_image_data_mutex.unlock()
                
                # Trunk data is newer
                elif timestamp_diff < -0.0001:
                    print("THIS IS WEIRD 12544") # delete this
                    self.odom_mutex.lock()
                    self.odom_data_list.insert(0, odom_data)
                    self.odom_mutex.unlock()
                    
                else:
                    seg_img = tree_image_data["seg_img"]
                    
                    self.signal_segmented_image.emit(seg_img, self.segmented_image_display_num)
                    
                    self.handle_odom_data(odom_data)
                    self.handle_tree_image_data(tree_image_data)
                    
                    self.plot_best_guess.emit(self.pf_engine.best_particle)
                    self.plot_particles.emit(self.pf_engine.downsample_particles())
                
                
                self.converged_signal.emit(self.pf_engine.check_convergence())
                
    
    def handle_odom_data(self, odom_data):
        print(odom_data)
        x_odom = odom_data["x_odom"]
        u = np.array([[x_odom/self.dt], [0]])
        num_readings = int(self.dt * 60)
        self.pf_engine.motion_update(u, self.dt, num_readings=num_readings)
    
    def handle_tree_image_data(self, tree_image_data):
        trunk_data = tree_image_data["trunk_data"]
        
        self.pf_engine.scan_update(trunk_data)            
        
    def check_convergence(self):
        self.converged = self.pf_engine.check_convergence()
        
        if self.converged and self.stop_when_converged:
            self.pf_active = False
            
    @pyqtSlot(dict)
    def trunk_data_reciever(self, tree_image_data):
        self.tree_image_data_mutex.lock()
        self.tree_image_data_queue.append(tree_image_data)
        self.tree_image_data_condition.wakeAll()
        self.tree_image_data_mutex.unlock()
    
    @pyqtSlot(dict)
    def odom_data_reciever(self, odom_data):
        self.odom_mutex.lock()
        self.odom_data_list.append(odom_data)
        self.odom_wait_condition.wakeAll()
        self.odom_mutex.unlock()
    
    @pyqtSlot()
    def stop_pf(self):
        self.pf_active = False
    
        self.odom_wait_condition.wakeAll()
        self.tree_image_data_condition.wakeAll()
        
    