from serial_communication import read_sensor, update_motors_order_thread
from encoder import display_encoder
from movement_type import *
from robust_serial import Order
import cv2 
from measurement import increment_per_second
from entrainement import test1
from robust_serial import Order
import time as time
from encoder import display_encoder
from serial_communication import update_motors_order_thread


"This file contains the robot s commands. To create a command,"
"you must create your function in movement_type.py, then put its keyboard shortcut in a new key in cmd_fct"
"and then either call the function in the value of the created key or call it below the code."


cmd_fct = { 'q': quit_, 
            'e': 'encoder_values',
            'z': 'zero_setting_encoders',
            '%': 'set_speed_pourcentage',
            'ff': forward,
            'l': left, 
            'r': right, 
            'bb': backward, 
            'lb': '[lb] left step back', 'rb': '[rb] right step back', 'f': '[f]orward step', 'b': '[b]ackward step', 
            'p': pause,    
            'fd':'fd',  
            's':servo,
            'c': '[c]amera',
            'ass' : 'ass',
            #'tar': tar,
            #'tal': tal,
            'tor' : 'tor',
            'tol' : 'tol',
            'mrd':'mrd',
            'mld':'mld',
            'mr': move_right,
            'ml':move_left,
            'mdrf': move_diagonal_r_f,
            'mdlf': move_diagonal_l_f,
            'mdrb': move_diagonal_r_b,
            'mdlb': move_diagonal_l_b
            }


cmd_help = {"[q]uit":'q',
          "[h]elp":'h',
          "[e]ncoder values":'e',
          "[z]ero setting encoders":'z',
          "(%) set motor speed percentage":'%',
          "[f]orward step": 'f',
          "[l] left step": 'l',
          "[r] right step": 'r',
          "[b]ackward step": 'b',
          "[lb] left step back":'lb',
          "[rb] right step back":'rb',
          "[ff]orward":'ff',
          "[bb]ackward":'bb',
          "[tl] turn left":'tl',
          "[tr] turn right":'tr',
          "[p]ause motors": 'p',
          "[s]ervo move": 's',
          "[d]istance measure":'d',
          "[fd]orward distance":'fd',
          "[rs] rotation speed" :'rs',
          "[mrd] translate right distance":'mrd',
          "[mld] translate left distance":'mld',
          "[sp]eed":'sp',
          "[c]amera" : 'c',
          "[tor] turn angle right]" : 'tor',
          "[tol] turn angle left]" : 'tol',
          "[ass] go to x y point" : 'ass',
          "move right":'mr',
          "move left":'ml',
          "move diagonal right forward":'mdrf',
          "move diagonal left forward":'mdlf',
          "move diagonal right backward":'mdrb',
          "move diagonal left backward":'mdlb',
            "[en]trainement" : 'en'
            }


def read_cmd():
    global motor_speed
    motor_speed = 100
    while True:
        cmd=input('Enter your command')
        print(cmd)
        #try :
        process_cmd(cmd)
        #except :
        #    pass

        
def process_cmd(cmd):
    global cmd_fct
    global cmd_help
    global motor_speed
    angle_servo = 90
    command_list=[]
    if cmd==cmd_help["[h]elp"]:
        for key in cmd_help.keys():
            print(key)
            
    elif cmd==cmd_help["[e]ncoder values"]:
        print(display_encoder())
   
    elif cmd==cmd_help["(%) set motor speed percentage"]:
        #Change en % la puissance du moteur
        motor_speed = int(cmd)
        print("Speed set to " + cmd + "%")

    elif cmd==cmd_help["[d]istance measure"]:
        print("distance measured: " + str(read_sensor()))
        
    elif cmd=='rs':
        print('looking for rotation speed...')
        print(increment_per_second(),' (increment per second)')
    
    elif cmd==cmd_help["[c]amera"]:
        #Permet d'afficher la vision caméra
        camera_port = 0
        vid = cv2.VideoCapture(camera_port)
        while vid.isOpened():
            print("La caméra est active")
            ret, frame = vid.read()
            cv2.imshow("test", frame)
            if cv2.waitKey(1) == ord("x"):
                vid.release()
                cv2.destroyAllWindows()
                break
            
    elif cmd==cmd_help["[en]trainement"]:
        test1()
        
    elif cmd[:2]=="fd":
        try : 
            distance = float(cmd.split()[1])
            L=[]
            fd(L,motor_speed, distance)
        except :
            print("Il faut saisir fd + distance (ex : fd 5.2)")
            
    elif cmd[:3]=="tor":
        try : 
            angle = float(cmd.split()[1])
            L=[]
            tor(L,motor_speed, angle)
        except :
            print("Il faut saisir tor + angle_en_° (ex : tor 90)")
            
    elif cmd[:3]=="mrd":
        try : 
            distance = float(cmd.split()[1])
            L=[]
            mrd(L,motor_speed, distance)
        except :
            print("Il faut saisir mrd + distance_en_m (ex : mrd 0.5)")
    
    elif cmd[:3]=="mld":
        try : 
            distance = float(cmd.split()[1])
            L=[]
            mld(L,motor_speed, distance)
        except :
            print("Il faut saisir mld + distance_en_m (ex : mld 0.5)")
            
    elif cmd[:3]=="ass":
        try : 
            x = float(cmd.split()[1])
            y = float(cmd.split()[2])
            L=[]
            deplacement(L,motor_speed, x , y)
        except :
            print("Il faut saisir ass + x_en_m + y_en_m (ex : ass 2 1)")
            
    elif cmd[:3]=="tol":
        try : 
            angle = float(cmd.split()[1])
            L=[]
            tol(L,motor_speed, angle)
        except :
            print("Il faut saisir tol + angle_en_° (ex : tor 90)")
    
    
    elif cmd[:1]=="%":
        try : 
            motor_speed = int(cmd.split()[1])
            print("Speed set to " + str(motor_speed) + "%")
        except :
            print("Il faut saisir % + pourcentage (ex : % 55)")
        

    elif cmd not in cmd_help.values():
        print('Wrong command')
        
    else:
        orders=[]
        cmd_fct[cmd](orders,motor_speed,angle_servo)
        print("hello")
        update_motors_order_thread(orders)


