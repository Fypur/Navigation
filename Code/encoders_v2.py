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
    prev_time_enc = t0
    
    clk1 = 17
    dt1 = 18
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter1 = 0
    clkLastState1 = GPIO.input(clk1)
    dtLastState1 = GPIO.input(dt1)

    clk2 = 22
    dt2 = 23
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter2 = 0
    clkLastState2 = GPIO.input(clk2)
    dtLastState2 = GPIO.input(dt2)
    
    clk3 = 12
    dt3 = 6
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter3 = 0
    clkLastState3 = GPIO.input(clk3)
    dtLastState3 = GPIO.input(dt3)
    
    clk4 = 4
    dt4 = 14
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(clk4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(dt4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    counter4 = 0
    clkLastState4 = GPIO.input(clk4)
    dtLastState4 = GPIO.input(dt4)
    
    
    dir1 = 0
    dir2 = 0
    dir3 = 0
    dir4 = 0
    t1 = 0
    t2 = 0
    t3 = 0
    t4 = 0
    
    while True:
        
        clkState1 = GPIO.input(clk1)               
        dtState1 = GPIO.input(dt1)
        if clkState1 != clkLastState1:
            dir1 = 0
            if dtState1 != clkState1:
                counter1 += 1
                dir1 = 1
                t1=time.time()
            else:
                counter1 -= 1
                dir1 = -1
                t1 = time.time()
        if dtState1 != dtLastState1:
            dir1 = 0
            if dtState1 == clkState1:
                counter1 += 1
                dir1 = 1
                t1=time.time()
            else:
                counter1 -= 1
                dir1 = -1
                t1 = time.time()
        
        dtLastState1 = dtState1    
        clkLastState1 = clkState1
        
        clkState2 = GPIO.input(clk2)               
        dtState2 = GPIO.input(dt2)
        if clkState2 != clkLastState2:
            dir2 = 0
            if dtState2 != clkState2:
                counter2 += 1
                dir2 = 1
                t2=time.time()
            else:
                counter2 -= 1
                dir2 = -1
                t2 = time.time()
        if dtState2 != dtLastState2:
            dir2 = 0
            if dtState2 == clkState2:
                counter2 += 1
                dir2 = 1
                t2=time.time()
            else:
                counter2 -= 1
                dir2 = -1
                t2 = time.time()
        
        dtLastState1 = dtState1
        clkLastState2 = clkState2
        
        clkState3 = GPIO.input(clk3)               
        dtState3 = GPIO.input(dt3)
        if clkState3 != clkLastState3:
            dir3 = 0
            if dtState3 != clkState3:
                counter3 += 1
                dir3 = 1
                t3=time.time()
            else:
                counter3 -= 1
                dir3 = -1
                t3 = time.time()
        if dtState3 != dtLastState3:
            dir3 = 0
            if dtState3 == clkState3:
                counter3 += 1
                dir3 = 1
                t3=time.time()
            else:
                counter3 -= 1
                dir3 = -1
                t3 = time.time()
        
        dtLastState3 = dtState3
        clkLastState3 = clkState3
        
        clkState4 = GPIO.input(clk4)               
        dtState4 = GPIO.input(dt4)
        if clkState4 != clkLastState4:
            dir4 = 0
            if dtState4 != clkState4:
                counter4 += 1
                dir4 = 1
                t4=time.time()
            else:
                counter4 -= 1
                dir4 = -1
                t4 = time.time()
        if dtState1 != dtLastState1:
            dir4 = 0
            if dtState4 == clkState4:
                counter4 += 1
                dir4 = 1
                t4=time.time()
            else:
                counter4 -= 1
                dir4 = -1
                t4 = time.time()
        
        dtLastState4 = dtState4
        clkLastState4 = clkState4
               
        if (time.time() - prev_time_enc)>=0.002: # we update all values temporally
            
            w1=(w1[1],(t1-t0, dir1, counter1))
            w2=(w2[1],(t2-t0, dir2, counter2))
            w3=(w3[1],(t3-t0, dir3, counter3))
            w4=(w4[1],(t4-t0, dir4, counter4))
            
            prev_time_enc = time.time()
        


    
def display_encoder():
    return w1,w2,w3,w4
    

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

