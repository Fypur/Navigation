from measurement import increment_per_second
from robust_serial import Order
import time as time
from encoder import display_encoder,return_speed
from serial_communication import update_motors_order_thread, read_ADC
import numpy as np
import csv
from serial_communication import read_ADC
import matplotlib.pyplot as plt

def test1():
    L=[Order.MOTOR,0,70,0,0]
    update_motors_order_thread(L)
    L1,L2=[],[]
    time.sleep(2)
    for _ in range (60):
        time.sleep(0.5)
        a=return_speed()
        L1.append(a[1])
        L2.append(-a[-1])


    L=[Order.STOP]
    update_motors_order_thread(L)
    
    time.sleep(1)
    print(return_speed())
    plt.plot(L1)
    plt.plot(L2)
    plt.show()

"""def test1():
    a=read_ADC()
    print(a)"""


"""def test0():
    print(increment_per_second())

def test1():
    global L
    print('go1')
    L=[Order.MOTOR,0,50,0,0]
    update_motors_order_thread(L)
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 2:
        t=time.time()
        if t - t1 >0.5:
            i = read_ADC()
            print(i)
    
    L=[Order.STOP]
    update_motors_order_thread(L)"""
    
"""liste_w =[]
    t0 = time.time()
    t1=t0
    L=[Order.MOTOR,0,0,0,0]
    update_motors_order_thread(L)
    while time.time()-t0<5:
        if time.time()-t1>0.001:
            liste_w.append(increment_per_second()[1])
            t1 = time.time()
    L=[Order.STOP]
    update_motors_order_thread(L)
    w_avg=sum(liste_w)/len(liste_w)
    print(w_avg)
    with open("mesure_40.csv","w",newline="") as file :
        writer=csv.writer(file)
        for item in liste_w:
            writer.writerow([item])"""
    

'''L=[Order.MOTOR,100,0,0,0]
    update_motors_order_thread(L)
    time.sleep(2)
    print(increment_per_second())
    L=[Order.MOTOR,0,100,0,0]
    update_motors_order_thread(L)
    time.sleep(2)
    print(increment_per_second())
    L=[Order.MOTOR,0,0,100,0]
    update_motors_order_thread(L)
    time.sleep(2)
    print(increment_per_second())
    L=[Order.MOTOR,0,0,0,100]
    update_motors_order_thread(L)
    time.sleep(2)
    print(increment_per_second())
    L=[Order.STOP]
    update_motors_order_thread(L)'''