from graph.logic.moving_path import *
import matplotlib.pyplot as plt



class Test_way(Deplacement_table):
    def __init__(self, pos, pos_adv, kaplas, nids):
        super().__init__(pos, pos_adv, kaplas, nids, 0.210)
    
    def representation(self, sommets, Adj):
        #represente le graphe
        fig, ax = plt.subplots()
        X,Y = [],[]
        for point in sommets:
            X.append(point[0])
            Y.append(point[1])
        ax.scatter(X,Y, color='r')

        for u in Adj.keys():
            x1, y1 = sommets[u]
            for v in Adj[u]:
                x2, y2 = sommets[v[0]]
                plt.plot([x1, x2],[y1, y2], 'b')
        ax.set_xlim(-5, 20)
        ax.set_ylim(-10, 10)
        ax.set_title("ceci est la construction du graphe")
        plt.show()
        

    def shortway(self, way, sommets):
        fig, ax = plt.subplots()
        X,Y = [],[]
        for point in sommets:
            X.append(point[0])
            Y.append(point[1])
        ax.scatter(X,Y, color='r')
        x,y = [],[]
        for point in way:
            x.append(point[0])
            y.append(point[1])
        ax.plot(x,y,'b')
        ax.set_xlim(-5, 20)
        ax.set_ylim(-10, 10)
        ax.set_title("short way")
        plt.show()

        
        


    def test1(self): #un obstacle
        self.obstacles.append([(3.0,-1.0),(3.0,1.0),(5.0,1.0),(5.0,-1.0)])
        A = (0,0)
        B = (10,0)
        #G, S = self.deplacement_graph_building(A,B)
        path, G, S = self.deplacement(A, B)
        sommets = [S[k] for k in S.keys()]
        
        print(path)
        self.shortway(path, sommets)
        plt.show()


    def test2(self): #2 obstacles
        self.obstacles =[[(3,-1),(3,1),(5,1),(5,-1)], [(7,-1),(7,1),(9,1),(9,-1)]]
        A, B = (0,0), (10,0)
        #G, S = self.deplacement_graph_building(A,B)
        #self.representation(list(S.values()), G.Arr)
        path, G, S = self.deplacement(A,B)
        sommets = [S[k] for k in S.keys()]
        print(path)
        self.shortway(path, sommets)
        plt.show()


    def test3(self): #nouveaux obstacles dans l'algorithme
        self.obstacles =[[(5,-1),(5,1),(5.3,1),(5.3,-1)], [(7,-0.1),(7.5,-0.1),(7.5,-1.5),(7,-1.5)], [(3.5,-0.2),(4,-0.7),(3.5,-2.5),(3,-2)], [(3.6,0.2),(4.1,0.7),(3.6,2.5),(3.1,2)]]
        A, B = (0,0), (10,0)
        #G, S = self.deplacement_graph_building(A,B)
        #self.representation(list(S.values()), G.Arr)
        path, G, S = self.deplacement(A,B)
        sommets = [S[k] for k in S.keys()]
        print(path)
        self.shortway(path, sommets)
        plt.show()

    def test4(self): #un des obstacles obliques n'est pas traversé dans le parcours
        self.obstacles =[[(5,-1),(5,1),(5.3,1),(5.3,-1)], [(7,-0.1),(7.5,-0.1),(7.5,-1.5),(7,-1.5)], [(3.5,-3.5),(4,-4),(3.5,-6.5),(3,-7)], [(3.6,0.2),(4.1,0.7),(3.6,2.5),(3.1,2)]]
        A, B = (0,0), (10,0)
        #G, S = self.deplacement_graph_building(A,B)
        #self.representation(list(S.values()), G.Arr)
        path, G, S = self.deplacement(A,B)
        sommets = [S[k] for k in S.keys()]
        print(path)
        self.shortway(path, sommets)
        plt.show()


test = Test_way((200,200),(300,300),[],[])

test.test3()
    