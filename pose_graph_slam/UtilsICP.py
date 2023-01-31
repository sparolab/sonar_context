import cv2
import numpy as np
from sklearn.neighbors import NearestNeighbors
import transforms3d as tf3
    
def best_fit_transform(A, B):
    m = A.shape[1]
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    t = centroid_B.T - np.dot(R,centroid_A.T)

    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m,  m] = t

    return T, R, t

def nearest_neighbor(src, dst):
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def icp(A, B, T_mat, max_iterations=1000, tolerance=0.001):
    m = A.shape[1]

    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))

    src[:m,:] = np.copy(A.T[:m, :])
    dst[:m,:] = np.copy(B.T[:m, :])
    
    if T_mat is not None:
        src = np.dot(T_mat, src)
    
    prev_error = 0
    for i in range(max_iterations):
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)
        T, _, _ = best_fit_transform(src[:m,:].T, dst[:m,indices].T) 
        src = np.dot(T, src)
        mean_error = np.mean(distances)
        
        if np.abs(prev_error - mean_error) < tolerance:
            break
        
        prev_error = mean_error

    T,_,_ = best_fit_transform(A[:, :m], src[:m,:].T)

    return T, src

def point_filtering(path, image, context_col, col_diff):
    frame = cv2.imread(path + str(image)  +'.png', cv2.IMREAD_UNCHANGED)
    frame = cv2.medianBlur(frame, 5)
    _, img_result = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    img_result = cv2.rotate(img_result, cv2.ROTATE_180)
    
    for i in range((img_result.shape[1])):
        x = []
        for j in range((img_result.shape[0])):
            if j > context_col:
                x.append(img_result[j][i])
            else:
                x.append(0)
        try:
            l = x.index(255)
            x = [255 if n==l else 0 for n in range(len(x))] 
        except:
            x = [0 for i in range(len(x))]
        img_result[:,i] = x

    if(col_diff > 0):
        img_result = img_result[:,:-col_diff]
    else:
        img_result = img_result[:,-col_diff:]

    img_result = cv2.rotate(img_result,cv2.ROTATE_180)
    
    return img_result

def img2ptcloud(image, sonar_angle, sonar_range):
    x, y, z = [], [], []
    for i in range(image.shape[0]):
        
        for j in range(image.shape[1]):
            
            if image[i][j] > 1:
                height = sonar_range * ((image.shape[0]-i)     / image.shape[0])
                width  = sonar_angle * ((j-(image.shape[1]/2)) / image.shape[1])
                x.append(height * np.sin(np.deg2rad(width)))
                y.append(height * np.cos(np.deg2rad(width)))
                z.append(0)
                
    point = np.vstack((x, y, z))
    
    return point.T

def loop_constraint(path, current, previous, col_diff, row_diff, sonar_angle, sonar_range, context_col, context_row):
    cur_img = point_filtering(path, current, context_col, col_diff)
    pre_img = point_filtering(path, previous, context_col, col_diff)

    cur_point = img2ptcloud(cur_img, sonar_angle, sonar_range)
    pre_point = img2ptcloud(pre_img, sonar_angle, sonar_range)
    
    T_mat = np.eye(4)
    
    col_degree = col_diff * (sonar_angle / context_row)
    row_range  = row_diff * (sonar_range / context_col)
    col_angle  = (col_degree/180) * np.pi
    
    meaningful_threshold = 10
    
    if(abs(col_degree) > meaningful_threshold):
        T_mat[:3, :3] = tf3.euler.euler2mat(0, 0, col_angle)
        T_mat[:2,  3] = (T_mat[:2,:2].dot(np.array([[0],[row_range]]))).reshape(-1)
            
    T, _ = icp(cur_point, pre_point, T_mat)
      
    return T