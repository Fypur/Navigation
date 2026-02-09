from robust_serial import Order
import time as time
from encoder import display_encoder
from serial_communication import update_motors_order_thread
from rotation import *
import threading
from distance_angle import *


"This file contains all the algorithms that allow the command.py file to work."
"It is not possible to test these functions on the robot"
"without creating a command in command.py and running main.py and writing your command in the python terminal"

def quit_(L,motor_speed,angle_servo=None):
    print("Goodbye...")
    exit()

def forward(L,motor_speed,angle_servo=None):
    #Avance tout droit 
    print("Moving forward at " + str(motor_speed) + "%...")
    L.append(Order.MOTOR)
    L.append(motor_speed) #Front right
    L.append(motor_speed) #Front left
    L.append(motor_speed) #Rear right
    L.append(motor_speed) #Rear left

def backward(L,motor_speed,angle_servo=None):
    #Avance en arrière 
    print("Moving backward at " + str(motor_speed) + "%...")
    L.append(Order.MOTOR)
    L.append(-motor_speed)
    L.append(-motor_speed)       
    L.append(-motor_speed)
    L.append(-motor_speed)
    
def left(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(motor_speed)
    L.append(-motor_speed)
    L.append(motor_speed)
    L.append(-motor_speed)
    
def right(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(-motor_speed)
    L.append(+motor_speed)
    L.append(-motor_speed)
    L.append(+motor_speed)
    
def pause(L,motor_speed,angle_servo=None):
    L.append(Order.STOP)

def servo(L,motor_speed,angle_servo=90):
    L.append(Order.SERVO)
    L.append(angle_servo)
    print("Moving front servo...")

def move_right(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(-motor_speed)
    L.append(+motor_speed)
    L.append(+motor_speed)
    L.append(-motor_speed)

def move_left(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(+motor_speed)
    L.append(-motor_speed)
    L.append(-motor_speed)
    L.append(+motor_speed)
    
def move_diagonal_l_f(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(+motor_speed)
    L.append(0)
    L.append(0)
    L.append(+motor_speed)
    
def move_diagonal_r_f(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(0)
    L.append(+motor_speed)
    L.append(+motor_speed)
    L.append(0)
    
def move_diagonal_l_b(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(0)
    L.append(-motor_speed)
    L.append(-motor_speed)
    L.append(0)
    
def move_diagonal_r_b(L,motor_speed,angle_servo=None):
    L.append(Order.MOTOR)
    L.append(-motor_speed)
    L.append(0)
    L.append(0)
    L.append(-motor_speed)



# def tal(record1,L,angle):
#     try:
#         a,_,_,_=display_encoder()
#         last=a[1][-1]
#     except:
#         last=0
#     tick=angle*4200/360/3 #Calcul empirique à améliorer
#     orders=[]
#     left(orders,100)
#     update_motors_order_thread(orders)
#     time.sleep(0.1)
#     binary=True
#     while binary:
#         _,_,a,_=display_encoder()
#         binary=not abs(a[1][-1]-last)<tick
#     pause(orders,100)
#     update_motors_order_thread(orders)
#     return("Finished")
#       
# 
# 
# def tar(angle):        #Tourne de l'angle indiqué vers la droite
#     try:
#         a,_,_,_=display_encoder()
#         last=a[1][-1]
#     except:
#         last=0
#     tick=angle*4200/360/3 #Calcul empirique à améliorer
#     orders=[]
#     right(orders,100)
#     update_motors_order_thread(orders)
#     time.sleep(0.1)
#     binary=True
#     while binary:
#         _,_,a,_=display_encoder()
#         binary=not abs(a[1][-1]-last)<tick
#     pause(orders,100)
#     update_motors_order_thread(orders)
#     return("Finished")

def fd(L,motor_speed, distance_, angle_servo=None):  #cette fonction permet d'avancer d'une distance donnée. (voir présentation/soutenance finale S6 pour comprendre)
    "forward of a distance d in m"
    global recup_distance
    distance_initiale = distance_obstacle_angle(0)
    if distance_initiale <= distance_*1000:  # on regarde si on peut avancer de la distance sans heurter un obstacle
        recup_distance = 0
        print("obstacle trop proche")
    else :
        start=time.time()
        forward(L,motor_speed, angle_servo=None) #on avance d'une bonne partie
        update_motors_order_thread(L)
        speed=0.5 #arbitraire pour que dans tous les cas on soit en dessous de la distance demandée
        duree=distance_/speed
        while time.time()-start<duree:
            pass
        pause(L,motor_speed)
        update_motors_order_thread(L)
        distance_actuelle = distance_obstacle_angle(0) #on regarde de combien on s'est avancé
        distance_parcourue = distance_initiale-distance_actuelle
        while distance_parcourue < distance_*1000: 
            start=time.time()
            forward(L,50, angle_servo=None)
            update_motors_order_thread(L)
            while time.time()-start<0.4:    #on bouge ensuite un tout petit peu on regarde si on dépasse, le 0.4 s est arbitraire à modifier (plus c'est petit, plus ça va être précis mais plus ça sera long)
                pass
            pause(L,motor_speed)
            update_motors_order_thread(L)
            time.sleep(0.1)
            distance_actuelle_avant, distance_actuelle = distance_actuelle, distance_obstacle_angle(0)
            distance_parcourue += distance_actuelle_avant - distance_actuelle
        recup_distance = distance_parcourue/1000
        print("Avance de " + str(distance_parcourue/1000) + " m")
        
# def fd(L,motor_speed, distance_, angle_servo=None): # à améliorer ??? cette fonction a été faite pour essayer (en vain) de mesurer en avancant
#     global recup_distance
#     distance_initiale = distance_obstacle_angle(0)
#     if distance_initiale <= distance_*1000:
#         recup_distance = 0
#         print("obstacle trop proche")
#     else :
#         while distance_initiale-distance_obstacle_angle(0) < distance_*1000:
#             forward(L,motor_speed, angle_servo=None)
#             update_motors_order_thread(L)
#         pause(L,85)
#         update_motors_order_thread(L)
#         distance_parcourue = distance_initiale-distance_obstacle_angle(0)
#         recup_distance = distance_parcourue/1000
#         print("Avance de " + str(distance_parcourue/1000) + " m")   
        
def tor(L, motor_speed, angle, distance_filter = 1000, duration = 4, angle_servo=None): #même principe que fd (on tourne d'un angle important inférieur à l'angle voulu puis on tourne de epsilon jusqu'à dépasser
    "turn right of angle in °. You can change the distance_filter or duration according to your will if you want to be accurate"
    "This algorithm does not work because the rotation angle is not measured correctly because the robot's rotation axis is not constant. However, for the algorithm that"
    "measures rotation, the lidar must not translate between the two measurements of the two points."
    angle_intermediaire = 0
    angle_total=0
    point_A = recuperer_centroid(distance_filter,duration)
    start = time.time()
    right(L,85,angle_servo=None)
    update_motors_order_thread(L)
    speed=1
    duree=angle/180 #pour tourner d'un angle suffisant pour être quand même inférieur à l'angle demandé
    while time.time()-start<duree:
        pass
    pause(L,85)
    update_motors_order_thread(L)
    start = time.time()
    while time.time()-start<0.1:  #pour éviter de commencer à mesurer des points durant l'arrêt du robot
        pass
    point_B = recuperer_centroid(distance_filter,duration)
    angle_total = calculate_rotation_angle(point_A,point_B) #on mesure de combien on a tourner
    print(angle_total)
    while angle_total < angle : #on tourne de epsilon jusqu'à dépasser en mesurant l'angle de rotation A CCHAQUE FOIS
        start = time.time()
        right(L,85,angle_servo=None)
        update_motors_order_thread(L)
        while time.time()-start<0.3:
            pass
        pause(L,motor_speed)
        update_motors_order_thread(L)
        start = time.time()
        while time.time()-start<0.1:
            pass
        point_B = recuperer_centroid(distance_filter,duration)
        angle_total = calculate_rotation_angle(point_A,point_B)
        print(angle_total)
    print("Rotation de " + str(angle_total) + " ° à droite") 
    

def tol(L, motor_speed, angle, distance_filter = 1000, duration = 4, angle_servo=None): #voir tor
    "turn left of angle in ° for instance 90° to left"
    "Don't work"
    liste_points_debut=[]
    angle_intermediaire = 0
    angle_total=0
    point_A = recuperer_centroid(distance_filter,duration)
    start = time.time()
    left(L,100,angle_servo=None)
    update_motors_order_thread(L)
    speed=1
    duree=angle/180
    while time.time()-start<duree:
        pass
    pause(L,85)
    update_motors_order_thread(L)
    start = time.time()
    while time.time()-start<0.1:
        pass
    point_B = recuperer_centroid(distance_filter,duration)
    angle_total = abs(calculate_rotation_angle(point_A,point_B))  
    print(angle_total)
    while angle_total < angle :
        start = time.time()
        left(L,100,angle_servo=None)
        update_motors_order_thread(L)
        while time.time()-start<0.3:
            pass
        pause(L,motor_speed)
        update_motors_order_thread(L)
        start = time.time()
        while time.time()-start<0.1:
            pass
        point_B = recuperer_centroid(distance_filter,duration)
        angle_total = abs(calculate_rotation_angle(point_A,point_B))
        print(angle_total)
    print("Rotation de " + str(angle_total) + " ° à gauche")



def mrd(L, motor_speed, distance, recup_distance = None, angle_servo=None): #voir fd
    "translation to right of a distance d in m"
    distance_initiale = distance_obstacle_angle(90)
    if distance_initiale <= distance*1000:
        print("obstacle trop proche")
        recup_distance=0
    else :
        start=time.time()
        move_right(L,motor_speed, angle_servo=None)
        update_motors_order_thread(L)
        speed=0.25
        duree=distance/speed
        while time.time()-start<duree:
            pass
        pause(L,motor_speed)
        update_motors_order_thread(L)
        distance_actuelle = distance_obstacle_angle(90)
        distance_parcourue = distance_initiale-distance_actuelle
        while distance_parcourue < distance*1000:
            start=time.time()
            move_right(L,100, angle_servo=None)
            update_motors_order_thread(L)
            while time.time()-start<0.5:    #on bouge ensuite un tout petit peu on regarde si on dépasse
                pass
            pause(L,motor_speed)
            update_motors_order_thread(L)
            time.sleep(0.1)
            distance_actuelle_avant, distance_actuelle = distance_actuelle, distance_obstacle_angle(90)
            distance_parcourue += distance_actuelle_avant - distance_actuelle
        recup_distance = distance_parcourue
        print("Translation à droite de " + str(distance_parcourue/1000) + " m")

def mld(L, motor_speed, distance, recup_distance = None, angle_servo=None): #voir fd
    "translation to left of a distance d in m"
    distance_initiale = distance_obstacle_angle(270)
    if distance_initiale <= distance*1000:
        print("obstacle trop proche")
        recup_distance=0
    else :
        start=time.time()
        move_left(L,motor_speed, angle_servo=None)
        update_motors_order_thread(L)
        speed=0.25
        duree=distance/speed
        while time.time()-start<duree:
            pass
        pause(L,motor_speed)
        update_motors_order_thread(L)
        distance_actuelle = distance_obstacle_angle(270)
        distance_parcourue = distance_initiale-distance_actuelle
        while distance_parcourue < distance*1000:
            start=time.time()
            move_left(L,100, angle_servo=None)
            update_motors_order_thread(L)
            while time.time()-start<0.5:    #on bouge ensuite un tout petit peu on regarde si on dépasse
                pass
            pause(L,motor_speed)
            update_motors_order_thread(L)
            time.sleep(0.1)
            distance_actuelle_avant, distance_actuelle = distance_actuelle, distance_obstacle_angle(270)
            distance_parcourue += distance_actuelle_avant - distance_actuelle
        recup_distance = distance_parcourue
        print("Translation à gauche de " + str(distance_parcourue/1000) + " m")  
        
def translation_gauche(L, motor_speed, d=0.5, angle_servo=None):
    "translation to left of d m using tor and tol"
    tol(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)
    fd(L,motor_speed, d, recup_distance=None, angle_servo=None)
    tor(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)
    
def translation_droite(L, motor_speed, d=0.5, angle_servo=None):
    tor(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)
    fd(L,motor_speed, d, recup_distance=None, angle_servo=None)
    tol(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)

            
def deplacement(L, motor_speed, x, y, angle_servo=None):
    "Allows you to move from the current point to a new point, avoiding obstacles."
    "You can use this algorithm to move forward a given distance by taking x = 0 and y = the desired distance in meters."
    "This algorithm is very very naive and needs to be improved but provides some basic ideas."
    # deplacement y en premier !!!
    recup_distance = 0
    fd(L,motor_speed, y,angle_servo=None)
    if recup_distance ==0 : #obstacle devant
        d_g = distance_obstacle_angle(270)#distance de l'obstacle -90
        d_d = distance_obstacle_angle(90) # distance de l'obstacle +90
        if d_g>d_d:
            mld(L, motor_speed, 0.5)
            deplacement(L, motor_speed, x+0.5,y) #on se déplace de 50cm à droite ou gauche puis on regarde si on peut avancer des y m sinon on recommence
        else:
            mrd(L, motor_speed, 0.5) 
            deplacement(L, motor_speed, x-0.5,y)
    elif x>-0.05 and x <0.05 and y>-0.05 and y<0.05:
        print('fin du déplacement')
        return None
    else : 
        # deplacement x en second !!!!
        if x>0.05:
            mrd(L, motor_speed, x)
            
        elif x<-0.05:
            mld(L, motor_speed, -x)
        else :
            print('fin du déplacement')
    #A AMELIORER
            

# def deplacement(L, motor_speed, x, y, angle_servo=None): #version améliorée mais fonctionnelle car tor et tol ne sont pas encore précis
#     "Allows you to move from the current point to a new point, avoiding obstacles."
#     "You can use this algorithm to move forward a given distance by taking x = 0 and y = the desired distance in meters."
#     "This algorithm is a better version using tor and tol avoid mrd and mld which decrease the possibilities"
#     # deplacement y en premier !!!
#     recup_distance = 0
#     fd(L,motor_speed, y,angle_servo=None)
#     if recup_distance ==0 : #obstacle devant
#         d_g = distance_obstacle_angle(270)#distance de l'obstacle -90
#         d_d = distance_obstacle_angle(90) # distance de l'obstacle +90
#         if d_g>d_d:
#             translation_gauche(L, motor_speed, d=0.5, angle_servo=None)
#             deplacement(L, motor_speed, x+0.5,y) #on se déplace de 50cm à droite ou gauche puis on regarde si on peut avancer des y m sinon on recommence
#         else:
#             translation_droite(L, motor_speed, d=0.5, angle_servo=None)
#             deplacement(L, motor_speed, x-0.5,y)
#     elif x>-0.05 and x <0.05 and y>-0.05 and y<0.05:
#         print('fin du déplacement')
#         return None
#     else : 
#         # deplacement x en second !!!!
#         if x>0.05:
#             #tor(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)
#             #deplacement(L, motor_speed, 0,x)
#       
#         elif x<-0.05:
#             #tol(L, motor_speed, 90, distance_filter = 1000, duration = 3, angle_servo=None)
#             #deplacement(L, motor_speed, 0,-x)
#         else :
#             print('fin du déplacement')
#     #A AMELIORER