import numpy as np
import graph.logic.params as params


def segmentIntersection(A:tuple[float, float], B:tuple[float, float], C:tuple[float, float], D:tuple[float, float]): #intersection entre 2 segments, les segments sont repérés par leur points
    #retourne un booléen
    if A[0]==B[0]:
        if A[1]==B[1]:
            return False
        if C[0]-D[0]==0:
            return False
        u = (A[0]-C[0])/(D[0]-C[0])
        if not 0<u<1:
            return False
        t = ((A[1]-C[1]) -u*(D[1]-C[1]))/(A[1]-B[1])
        return 0<t<1
    elif A[1]==B[1]:  # le coefficient de t est non nul
        if C[1]-D[1]==0:
            return False
        u = (A[1]-C[1])/(D[1]-C[1])
        if not 0<u<1:
            return False
        t = ((A[0]-C[0]) -u*(D[0]-C[0]))/(A[0]-B[0])
        return 0<t<1   
    a = (A[0] - B[0])*(C[1] - D[1])
    b =(A[1] - B[1])*(C[0]-D[0])
    denom = a-b
    if denom ==0:
        return False
    else:
        t = ((A[0]-C[0])*(C[1]-D[1]) - (A[1]-C[1])*(C[0] - D[0]))/denom
        u = -((A[0]-B[0])*(A[1]-C[1]) - (A[1]-B[1])*(A[0] - C[0]))/denom
        if u == 0 or t ==0 or u ==1 or t ==1:
            return False
        if not (0 < u < 1):
            return False
        t = ((A[0]-C[0])*(C[1]-D[1]) - (A[1]-C[1])*(C[0] - D[0]))/denom
        return 0<t<1
    

def voisinage(obstacle:list[tuple[float,float]], A:tuple[float, float], B:tuple[float, float]):
    for k in range(len(obstacle)):
        if (obstacle[k%len(obstacle)] == A and obstacle[(k+1)%len(obstacle)] == B) or (obstacle[k%len(obstacle)] == B and obstacle[(k+1)%len(obstacle)] == A):
            return True
    return False


def dist(A:tuple[float, float], B:tuple[float, float]):
    return np.sqrt((B[0]-A[0])**2 + (B[1] - A[1])**2)

def dist_way(L:list[tuple[float, float]]):
    res = 0
    for k in range(len(L)):
        res += dist(L[k], L[k+1])
    return res
    
def dot(p1:tuple[float, float], p2:tuple[float, float]):
    return p1[0]*p2[0] + p1[1]*p2[1]

def hitbox(kapla):
    return(dist(kapla[0], kapla[1]), dist(kapla[1], kapla[2]))

def center_pos(kapla):
    X1 = (kapla[0][0]+kapla[1][0])/2,(kapla[0][1]+kapla[1][1])/2 
    X2 = (kapla[2][0]+kapla[3][0])/2,(kapla[2][1]+kapla[3][1])/2
    return ((X1[0]+X2[0])/2, (X1[1]+X2[1])/2)

def aabb_collision(kapla1, kapla2):
    width1, height1 = hitbox(kapla1)
    width2, height2 = hitbox(kapla2)
    x_condition = (abs(center_pos(kapla1)[0] - center_pos(kapla2)[0]) < (height1 + height2) / 2)
    y_condition = (abs(center_pos(kapla1)[1] - center_pos(kapla2)[1]) < (width1 + width2) / 2)
    return x_condition and y_condition

def sat_collision(kapla1, kapla2):
    axis = [(kapla1[0][0] - kapla1[1][0], kapla1[0][1] - kapla1[1][1]), (kapla1[0][0] - kapla1[2][0], kapla1[0][1] - kapla1[2][1]), (kapla2[0][0] - kapla2[1][0], kapla2[0][1] - kapla2[1][1]), (kapla1[0][0] - kapla1[2][0], kapla1[0][1] - kapla1[2][1])]

    for a in axis:
        self_projected_points = [dot(points, a) for points in kapla1]
        self_min, self_max = min(self_projected_points), max(self_projected_points)
        other_projected_points = [dot(points, a) for points in kapla2]
        other_min, other_max = min(other_projected_points), max(other_projected_points)
        if (self_max < other_min) or (other_max < self_min):
            return False
    return True

def detect_collision(kapla1, kapla2):
    return aabb_collision(kapla1, kapla2) and sat_collision(kapla1, kapla2)

def merge_kaplas(kapla1, kapla2):
    min_x = kapla1[0][0]
    max_x = kapla1[0][0]
    min_y = kapla1[0][1]
    max_y = kapla1[0][1]
    for point in kapla1+kapla2:
        if point[0] > max_x:
            max_x = point[0]
        if point[0] < min_x:
            min_x = point[0]
        if point[1] > max_y:
            max_y = point[1]
        if point[1] < min_y:
            min_y = point[1]
    return [(min_x, max_y), (max_x, max_y), (max_x, min_y), (min_x, min_y)]
        
        
def is_instance_of(kapla1, kapla_class):
    for kapla in kapla_class:
        if detect_collision(kapla1, kapla):
            return True
    return False


def angle_seg(A:tuple, B:tuple):
    return np.arctan2((B[1]-A[1]), (B[0]-A[0]))

def angle_travel(path: list[tuple]):
    n = len(path)
    new_path = []
    for k in range(n -1):
        ang = angle_seg(path[k], path[k+1])
        new_path.append((path[k][0], path[k][1], ang))
    return new_path

#remplace les parcours le long des bords des obstacles par des déviations vers un point plus haut
def simplify_travel(path: list[tuple]):
    pass

def is_in_the_area(A: tuple):
    if not (0 < A[0] < params.GAME_AREA_WIDTH and 0 < A[1] < params.GAME_AREA_HEIGHT):
        return False
    return True