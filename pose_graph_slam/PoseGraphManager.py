import numpy as np
np.set_printoptions(precision=4)

import minisam
from UtilsMisc import *
    
class PoseGraphManager:
    def __init__(self):
        self.prior_cov = minisam.DiagonalLoss.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))  
        self.const_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])    
        self.odom_cov = minisam.DiagonalLoss.Sigmas(self.const_cov)  
        self.loop_cov = minisam.DiagonalLoss.Sigmas(self.const_cov)  
        self.graph_factors = minisam.FactorGraph()
        self.graph_initials = minisam.Variables()
        self.opt_param = minisam.LevenbergMarquardtOptimizerParams()
        self.opt = minisam.LevenbergMarquardtOptimizer(self.opt_param)

        self.init_pose = None        
        self.curr_se3 = None        
        self.prev_se3 = None        
        self.curr_node_idx = None 
        self.prev_node_idx = None 
        self.graph_optimized = None

    def addPriorFactor(self):
        self.curr_node_idx = 0
        self.prev_node_idx = 0
        self.curr_se3 = self.init_pose
        self.graph_initials.add(minisam.key('x', self.curr_node_idx), minisam.SE3(self.curr_se3))
        self.graph_factors.add(minisam.PriorFactor(                                                 
                               minisam.key('x', self.curr_node_idx),  
                               minisam.SE3(self.curr_se3), 
                               self.prior_cov))

    def addPoseFactor(self):
        self.graph_initials.add(minisam.key('x', self.curr_node_idx), minisam.SE3(self.curr_se3))                                                                                           
        
    def addOdometryFactor(self, odom_transform):
        self.graph_initials.add(minisam.key('x', self.curr_node_idx), minisam.SE3(self.curr_se3))                                                                                          
        self.graph_factors.add(minisam.BetweenFactor(                                               
                                                minisam.key('x', self.prev_node_idx), 
                                                minisam.key('x', self.curr_node_idx), 
                                                minisam.SE3(odom_transform), 
                                                self.odom_cov))

    def addLoopFactor(self, loop_transform, loop_idx):
        self.graph_factors.add(minisam.BetweenFactor(
                                            minisam.key('x', loop_idx), 
                                            minisam.key('x', self.curr_node_idx),  
                                            minisam.SE3(loop_transform), 
                                            self.loop_cov))
    def optimizePoseGraph(self):
        self.graph_optimized = minisam.Variables()
        status = self.opt.optimize(self.graph_factors, self.graph_initials, self.graph_optimized)
        pose_trans, pose_rot = getGraphNodePose(self.graph_optimized, self.curr_node_idx)
        self.curr_se3[:3, :3] = pose_rot
        self.curr_se3[:3, 3] = pose_trans