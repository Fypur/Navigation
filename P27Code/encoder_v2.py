import time as time
from RPi import GPIO
import numpy as np
import time

w1=((0,1,0),(0,1,0))
w2=((0,1,0),(0,1,0))
w3=((0,1,0),(0,1,0))
w4=((0,1,0),(0,1,0))

def encoder():
    global w1
    global w2
    global w3
    global w4
    t0 = time.time()
    
    clk1 = 17
    dt1 = 18
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter1 = 0
    clkLastState1 = GPIO.input(clk1)

    clk2 = 22
    dt2 = 23
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter2 = 0
    clkLastState2 = GPIO.input(clk2)
    
    clk3 = 20
    dt3 = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter3 = 0
    clkLastState3 = GPIO.input(clk3)
    
    clk4 = 4
    dt4 = 14
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter4 = 0
    clkLastState4 = GPIO.input(clk4)
    
    while True:
        clkState1 = GPIO.input(clk1)               
        dtState1 = GPIO.input(dt1)
        if clkState1 != clkLastState1:
            dir = 0
            if dtState1 != clkState1:
                counter1 += 1
                dir = 1
                t=time.time()
            else:
                counter1 -= 1
                dir = -1
                t = time.time()
            if abs(counter1-w1[1][2])>=21:
                w1=(w1[1],(time.time()-t0, dir, counter1))
        clkLastState1 = clkState1
        
        clkState2 = GPIO.input(clk2)               
        dtState2 = GPIO.input(dt2)
        if clkState2 != clkLastState2:
            dir = 0
            if dtState2 != clkState2:
                counter2 += 1
                dir = 1
                t=time.time()
            else:
                counter2 -= 1
                dir = -1
                t = time.time()
            if abs(counter2-w2[1][2])>=21:
                w2=(w2[1],(time.time()-t0, dir, counter2))
        clkLastState2 = clkState2
        
        clkState3 = GPIO.input(clk3)               
        dtState3 = GPIO.input(dt3)
        if clkState3 != clkLastState3:
            dir = 0
            if dtState3 != clkState3:
                counter3 += 1
                dir = 1
                t=time.time()
            else:
                counter3 -= 1
                dir = -1
                t = time.time()
            if abs(counter3-w3[1][2])>=21:
                w3=(w3[1],(time.time()-t0, dir, counter3))

        clkLastState3 = clkState3
        
        clkState4 = GPIO.input(clk4)               
        dtState4 = GPIO.input(dt4)
        if clkState4 != clkLastState4:
            dir = 0
            if dtState4 != clkState4:
                counter4 += 1
                dir = 1
                t=time.time()
            else:
                counter4 -= 1
                dir = -1
                t = time.time()
            if abs(counter4-w4[1][2])>=21:
                w4=(w4[1],(time.time()-t0, dir, counter4))
        clkLastState4 = clkState4

"""w1,w2,w3,w4=21*[(0,1,0)],21*[(0,1,0)],21*[(0,1,0)],21*[(0,1,0)]

def encoder():
    global w1
    global w2
    global w3
    global w4
    global liste_t
    dir1,dir2,dir3,dir4=1,1,1,1
    
    clk1 = 17
    dt1 = 18
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter1 = 0
    clkLastState1 = GPIO.input(clk1)

    clk2 = 22
    dt2 = 23
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter2 = 0
    clkLastState2 = GPIO.input(clk2)
    
    clk3 = 20
    dt3 = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter3 = 0
    clkLastState3 = GPIO.input(clk3)
    
    clk4 = 4
    dt4 = 14
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter4 = 0
    clkLastState4 = GPIO.input(clk4)
    
    t0 = time.time()
    liste_t=[0]
    while True:
        clkState1 = GPIO.input(clk1)               
        dtState1 = GPIO.input(dt1)
        t=time.time()
        if clkState1 != clkLastState1:
            if dtState1 != clkState1:
                counter1 += 1
                dir1 = 1
            else:
                counter1 -= 1
                dir1 = -1
            w1=[(t-t0, dir1, counter1)]+w1[:20]
        if t-w1[0][0]-t0>0.001 :
            w1=[(t-t0, dir1, counter1)]+w1[:20]
        clkLastState1 = clkState1
        
        clkState2 = GPIO.input(clk2)               
        dtState2 = GPIO.input(dt2)
        t=time.time()
        if clkState2 != clkLastState2:
            if dtState2 != clkState2:
                counter2 += 1
                dir2 = 1
            else:
                counter2 -= 1
                dir2 = -1
            w2=[(t-t0, dir2, counter2)]+w2[:20]
        if t-w2[0][0]-t0>0.001 :
            w2=[(t-t0, dir2, counter2)]+w2[:20]
        clkLastState2 = clkState2
        
        clkState3 = GPIO.input(clk3)
        dtState3 = GPIO.input(dt3)
        t=time.time()
        if clkState3 != clkLastState3:
            if dtState3 != clkState3:
                counter3 += 1
                dir3 = 1
            else:
                counter3 -= 1
                dir3 = -1
            w3=[(t-t0, dir3, counter3)]+w3[:20]
            "liste_t.append(t-t0)"
        if t-w3[0][0]-t0>0.001 :
            w3=[(t-t0, dir3, counter3)]+w3[:20]
        clkLastState3 = clkState3
        
        clkState4 = GPIO.input(clk4)               
        dtState4 = GPIO.input(dt4)
        t=time.time()
        if clkState4 != clkLastState4:
            if dtState4 != clkState4:
                counter4 += 1
                dir4 = 1
            else:
                counter4 -= 1
                dir4 = -1
            w4=[(t-t0, dir4, counter4)]+w4[:20]
        if t-w4[0][0]-t0>0.001 :
            w4=[(t-t0, dir4, counter4)]+w4[:20]
        clkLastState4 = clkState4"""



    
def display_encoder():
    return (w1[-1],w1[0]),(w2[-1],w2[0]),(w3[-1],w3[0]),(w4[-1],w4[0])
    
def colect_liste_t():
    return(liste_t)

def reset_liste_t():
    liste_t=[]

def collect_encoder():
    global w1
    global w2
    global w3
    global w4
    last_encoder = [0,0,0,0]
    
    try:
        with open('encoders_info_turnleft.txt', 'w') as f:
            f.write(f'{last_encoder}\n')
            while True:
                encoder = [w1[1][2],w2[1][2],w3[1][2],w4[1][2]]
                #f.write(f'{encoder}\n')
                if True:#if encoder[0]!=0 and last_encoder[0]!= encoder[0]:
                    f.write(f'{encoder}\n')
                    last_encoder = encoder
                

    except Exception as e:
        print(f"Failed to write to file: {e}")
    

