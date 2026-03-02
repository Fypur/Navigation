from moving_path import *

def test_set_kapla():
    kaplas = [(1.77645, 0.80060, 1.57, "YELLOW")]
    table = Deplacement_table((0,0),(0,0),kaplas,[], 0.15)
    print(table.set_kaplas())

test_set_kapla()
