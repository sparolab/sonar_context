import numpy as np
import sys
np.set_printoptions(precision=4)
from scipy import spatial

def col_distance_sc(sc1, sc2):
    num_sector = sc1.shape[1]
    
    _one_step = 1
    sim_for_each_cols_r = np.zeros(num_sector) 
    sim_for_each_cols_l = np.zeros(num_sector) 

    bounded_factor = 0.5
    
    sc1_r = sc1
    sc1_l = sc1

    # 4.1 right column shifting
    for i in range(int(num_sector*bounded_factor)):
        sc1_r = np.roll(sc1_r, _one_step, axis=1)
        # 4.1.1 zero-padding       
        sc1_r[:, :i] = np.zeros(sc1_r[:, :i].shape)
        sum_cossim = 0
        num_col_engaged = 0    
            
        for j in range(i, num_sector):
            col_j1 = sc1_r[:, j]
            col_j2 = sc2[:, j]
            
            if (~np.any(col_j1) or ~np.any(col_j2)):
                continue 
            
            cossim = np.dot(col_j1,col_j2) / (np.linalg.norm(col_j1)*np.linalg.norm(col_j2))
            sum_cossim += cossim
            num_col_engaged += 1
        
        sim_for_each_cols_r[i] = sum_cossim/num_col_engaged

    # 4.2 left column shifting
    for i in range(int(num_sector*bounded_factor)):
        sc1_l = np.roll(sc1_l, -_one_step, axis=1)
        # 4.2.1 zero-padding       
        sc1_l[:, -i-1:] = np.zeros(sc1_l[:, -i-1:].shape)      
        sum_cossim = 0
        num_col_engaged = 0

        for j in range(0, num_sector-i):
            col_j1 = sc1_l[:, j]
            col_j2 = sc2[:, j]
            
            if (~np.any(col_j1) or ~np.any(col_j2)):
                continue
             
            cossim = np.dot(col_j1,col_j2) / (np.linalg.norm(col_j1)*np.linalg.norm(col_j2))
            sum_cossim += cossim
            num_col_engaged += 1

        sim_for_each_cols_l[i] = sum_cossim / num_col_engaged

    yaw_diff_r = np.argmax(sim_for_each_cols_r)+1
    yaw_diff_l = np.argmax(sim_for_each_cols_l)+1
    
    if(sim_for_each_cols_r[yaw_diff_r] > sim_for_each_cols_l[yaw_diff_l]):
        yaw_diff = yaw_diff_r
        sim = sim_for_each_cols_r[yaw_diff_r]
    else:
        yaw_diff = -yaw_diff_l
        sim = sim_for_each_cols_l[yaw_diff_l]
    
    dist = 1- sim

    return dist, -yaw_diff, None ,None

def row_distance_sc(sc1, sc2):
    num_sector = sc1.shape[0]
    
    _one_step = 1
    sim_for_each_rows_u = np.zeros(num_sector) 
    sim_for_each_rows_d = np.zeros(num_sector) 
    
    bounded_factor = 0.1

    sc1_u = sc1
    sc1_d = sc1

    # 5.1 up row shifting
    for i in range(int((num_sector)*bounded_factor)):
        sc1_u = np.roll(sc1_u, _one_step, axis=0)      
        # 5.1.1 zero-padding         
        sc1_u[:i, :] = np.zeros(sc1_u[:i, :].shape)
        sum_cossim = 0
        num_row_engaged = 0
        
        for j in range(i, num_sector):
            row_j1 = sc1[j, :]
            row_j2 = sc2[j, :]
            
            if (~np.any(row_j1) or ~np.any(row_j2)):
                continue
             
            cossim = np.dot(row_j1, row_j2) / (np.linalg.norm(row_j1)*np.linalg.norm(row_j2))
            sum_cossim += cossim
            num_row_engaged += 1
            
        sim_for_each_rows_u[i] = sum_cossim / num_row_engaged     
    
    # 5.2 down row shifting
    for i in range(int((num_sector)*bounded_factor)):
        sc1_d = np.roll(sc1_d, (-_one_step), axis=0)     
        # 5.2.1 zero-padding          
        sc1_d[-i-1:, :] = np.zeros(sc1_d[-i-1:, :].shape)
        sum_cossim = 0
        num_row_engaged = 0

        for j in range(0, num_sector-i):
            row_j1 = sc1[j, :]
            row_j2 = sc2[j, :]
            
            if (~np.any(row_j1) or ~np.any(row_j2)):
                continue 
            
            cossim = np.dot(row_j1, row_j2) / (np.linalg.norm(row_j1)*np.linalg.norm(row_j2))
            sum_cossim += cossim
            num_row_engaged += 1
            
        sim_for_each_rows_d[i] = sum_cossim / num_row_engaged   
    
    row_diff_u = np.argmax(sim_for_each_rows_u) + 1   
    row_diff_d = np.argmax(sim_for_each_rows_d) + 1   
    
    if(sim_for_each_rows_u[row_diff_u] > sim_for_each_rows_d[row_diff_d]):
        sim = sim_for_each_rows_u[row_diff_u]
        row_diff = row_diff_u
        
    else:
        sim = sim_for_each_rows_d[row_diff_d]
        row_diff = -row_diff_d
        
    row_dist = 1- sim
    
    return row_dist, row_diff                  

class SonarContextManager:
    def __init__(self, shape, col_threshold, row_threshold):
        self.shape = shape
        self.col_threshold = col_threshold
        self.row_threshold = row_threshold
        self.sonar_range   = 50
        self.sonar_angle   = 130
        self.ENOUGH_LARGE  = 15000                           
        self.sonarcontexts = [None] * self.ENOUGH_LARGE
        self.polarkeys     = [None] * self.ENOUGH_LARGE
        self.curr_node_idx = 0

    def addNode(self, node_idx, sc, pk):        
        self.curr_node_idx = node_idx
        self.sonarcontexts[node_idx] = sc
        self.polarkeys[node_idx] = pk
    
    def detectLoop(self, exclude_recent_nodes):
        valid_recent_node_idx = self.curr_node_idx - exclude_recent_nodes
        
        if(valid_recent_node_idx < 1):
            return None, None, None, None
        else:
            # 1. construct KD-tree
            polarkey_history = np.array(self.polarkeys[:valid_recent_node_idx])
            polarkey_tree = spatial.KDTree(polarkey_history)
            polarkey_query = self.polarkeys[self.curr_node_idx]
            _, nncandidates_idx = polarkey_tree.query(polarkey_query, k=1)

            # 2. Determine loop candidate
            query_sc = self.sonarcontexts[self.curr_node_idx]
            top_idx = nncandidates_idx
            top_sc = self.sonarcontexts[top_idx]
            
            # 3. Specify loop candidate    
            # 4. column shifting
            col_dist, col_diff, row_dist, row_diff = col_distance_sc(top_sc, query_sc)
                            
            if(col_dist < self.col_threshold):                
                return top_idx, col_dist, col_diff, 0
            else:
                # 5. row shifting
                row_dist, row_diff = row_distance_sc(top_sc, query_sc)
                if(row_dist < self.row_threshold):
                    return top_idx, row_dist, 0, row_diff
                else:
                    return None, None, None, None