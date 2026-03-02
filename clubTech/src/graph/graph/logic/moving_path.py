
from graph.logic.utils import segmentIntersection, dist, voisinage, dist_way, is_instance_of, angle_travel, is_in_the_area
import graph.logic.params as params
import numpy as np


class Deplacement_table:
    def __init__(self, pos:tuple[float, float], pos_adv:tuple[float, float], kaplas:list[tuple[float, float,float,str]], nids:list):
        self.obstacles = [] #on represente les obstacles par des rectangles orientés
        self.pos = pos
        self.pos_adv = pos_adv
        self.kaplas = kaplas
        self.nids = nids
        self.kapla_dim = (0.15, 0.05) #en mètre 
        self.securityDistance = params.SECURITY_DISTANCE_BM #la distance ajouté dans les dimensions du kaplas pour une sécurité physique
        self.set_obstacles() #mets les groupes de kaplas sur lesquelles on travaillera à jour
        
        

    def set_kaplas(self): # avec les données des capteurs on renseigne les données sur les capteurs
        L, l = self.kapla_dim[0], self.kapla_dim[1]
        alpha = np.arctan(l/L)
        r = np.hypot(l, L) / 2
        kapla_representation_list: list[np.nd] = []
        
        c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
        security_offset_up_right = self.securityDistance * np.array([c, s])
        security_offset_down_right = self.securityDistance * np.array([c, -s])   
            
        for x,y,theta, _ in self.kaplas:
            A=np.array([x + r * np.cos(theta + alpha), y + r * np.sin(theta + alpha)])
            B=np.array([x + r * np.cos(theta - alpha), y - r * np.sin(theta - alpha)])
            C=np.array([x - r * np.cos(theta + alpha), y - r * np.sin(theta + alpha)])
            D=np.array([x - r * np.cos(theta - alpha), y + r * np.sin(theta - alpha)])
            
            rotation = np.array([[np.cos(theta), -np.sin(theta)], 
                                [np.sin(theta), np.cos(theta)]])
        
            kapla_representation_list.append([A + rotation @ security_offset_up_right, 
                                              B + rotation @ security_offset_down_right, 
                                              C - rotation @ security_offset_up_right, 
                                              D - rotation @ security_offset_down_right])
        return kapla_representation_list
    
    def set_obstacles(self): #maintenant qu'on a la position de tous les kaplas, on peut créer des obstacles globaux claires dans les déplacements du robot
        kapla_list = self.set_kaplas() #i assume the list isn't empty
        class_obstacles = {0:[kapla_list[0]]}
        class_obstacles_points = {0:[point for point in kapla_list[0]]} #on va regrouper les kaplas dont la zone de secu se rencontrent en un seul obstacle englobant
        for kapla in kapla_list[1:]:
            class_trouve = False
            for clé, group in class_obstacles.items():
                if is_instance_of(kapla, group):
                    class_trouve = True
                    class_obstacles[clé].append(kapla)
                    for points in kapla:
                        class_obstacles_points[clé].append(points)
                    break
            if not class_trouve:
                k = len(class_obstacles)
                class_obstacles[k] = [kapla]
                class_obstacles_points[k] = [points for points in kapla]

            
        #now that we have groups of kaplas, we can create the real obstacles BM
        #First of all, we add the scene area to the obstacle for the robots
        self.obstacles.append(params.SCENE_AREA)

        for group in class_obstacles_points.values():
            X = [point[0] for point in group]
            Y = [point[1] for point in group]
            xmax, ymax = (max(X),max(Y))
            xmin, ymin = ((min(X),min(Y)))
            self.obstacles.append([(xmin,ymin),(xmin,ymax), (xmax,ymax),(xmax,ymin)])


    def collision_obst(self, A:tuple[float, float], B:tuple[float, float]):  #on doit recupérer la liste des segments par lesquelles on coupe
        collisions = []
        for obstacle in self.obstacles:
            # obstacle = [A,B,C,D] ou plus de points pour un polygone plus grand
            for i in range(len(obstacle)):
                if segmentIntersection(A, B, obstacle[(i % len(obstacle))], obstacle[(i+1) % len(obstacle)]):
                    collisions.append((obstacle[int((i % len(obstacle)))], obstacle[int((i + 1) % len(obstacle))]))
        return collisions
    
    def deplacement_graph_building(self, A:tuple[float, float], B:tuple[float, float]):
        S ={
            0:A,
            1:B
        }

        Adj = {0:[],
             1:[]
             }
        G = Graph_valued(list(S.keys()),Adj)

        #une fonction pour ajouter recursivement les arrêtes du graphe
        def arrête_building(A_num):
            for s_num in G.S:
                if len(self.collision_obst(S[A_num], S[s_num]))==0:
                    if self.possible_road(S[A_num], S[s_num]):
                        G.add_arret(A_num, s_num, dist(S[A_num], S[s_num]))
                else:
                    collision = self.collision_obst(S[A_num], S[s_num])[0]
                    #for collision in self.collision_obst(S[A_num], S[s_num]):
                    for i in range(2):
                        if collision[i] not in list(S.values()):
                            C = collision[i]
                            k = len(S)
                            S[k] = C
                            G.add_sommet(k)
                            arrête_building(k)
                                

        arrête_building(0)
        return G, S

            
    def deplacement(self, A:tuple[float, float], B:tuple[float, float], theta:float):
        '''
        Docstring for deplacement
        reçoit les points de départ et d'arrivée et renvoit la liste des points à suivre pour se déplacer sur la table safely(vis à vis des obstacles)
        chaque point est donné avec l'angle que le robot doit prendre au cours du déplacement
        :param self: la table
        :param A: Point de départ
        :type A: tuple[float, float]
        :param B: point d'arrivé
        :type B: tuple[float, float]
        :param theta: angle d'arrivé
        :type theta: float
        '''
        G, S = self.deplacement_graph_building(A, B)
        way_graph = G.dijkstra(0, 1)
        path = [S[k] for k in way_graph]
        #on a pas besoin d'associer un angle au point de départ
        #l'angle d'arrivé est fixé par la requête du client, on ne le met pas aussi
        #angle_travel est une fonction qui calculera l'orientation des segments du trajet, et ces angles seront associés au point de départ du segment(trajet)
        new_path = [path[0]] + angle_travel(path[1:]) + [(path[-1][0],path[-1][1],theta)]
        return new_path

    def same_obst(self, A, B):
        for obstacle in self.obstacles:
            if ((A in obstacle) and (B in obstacle)):
                return (True, obstacle)
        return (False, None)
    
    def possible_road(self,A:tuple[float, float], B:tuple[float, float]): #vérifie si la route A, B directe passe en croix dans un obstacle(dans le cas où on ne coupe pas d'obstacle)
        
        #Let's verify that all the points are in the game area
        if not (is_in_the_area(A) and is_in_the_area(B)):
            return False

        verif, obst = self.same_obst(A, B)
        if verif:
            if voisinage(obst, A, B):
                return True
            return False
        return True
    
        
    
    
class Graph_valued:
    def __init__(self, S:list[int], A:dict[int,list[tuple[int,float]]]):
        self.S = S
        self.Arr  = A

    def add_sommet(self, sommet:int): #on identifie un sommet par la liste de points
        if sommet not in self.S:
            self.S.append(sommet)
            self.Arr[sommet] = []

    def add_arret(self, som1:int, som2:int, nu:float): #arrête de som1 vers som2
        if (som2,nu) not in self.Arr[som1]:
            self.Arr[som1].append((som2, nu))
            self.Arr[som2].append((som1, nu))

    def dijkstra(self, A:int, B:int): #dijkstra de A vers B
        cout, predec, djvu = {},{},{}
        for s in self.S: #initialisation
            cout[s] = np.inf   #dist infinie
            djvu[s] = False     #non visité
        cout[A] = 0
        for _ in self.S:
            smin , cmin =  A,  np.inf #plus court chemin pour aller 
            for s in self.S:
                if not djvu[s] and cout[s]<cmin:
                    smin = s 
                    cmin = cout[s]
                
            djvu[smin] = True
            for som,nu in self.Arr[smin]:
                if cout[smin] + nu < cout[som]:
                    cout[som] = cout[smin] + nu
                    predec[som] = smin

        way = [B]
        while A not in way:
            p = predec[way[-1]]
            way.append(p)
             
                
        return list(reversed(way))
    

            
