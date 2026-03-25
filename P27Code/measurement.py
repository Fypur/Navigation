from encoder import display_encoder

def increment_per_second():
    while True:
        try:
            w1,w2,w3,w4 = display_encoder()
            #print(w1,w2,w3,w4)
            break
        except:
            a=1
    try:
        v1 = (w1[1][2] - w1[0][2]) / (w1[1][0] - w1[0][0])
    except:
        v1=0
    try:
        v2 = (w2[1][2] - w2[0][2]) / (w2[1][0] - w2[0][0])
    except:
        v2=0
    try:
        
        v3 = (w3[1][2] - w3[0][2]) / (w3[1][0] - w3[0][0])
    except:
        v3=0
    try:
        v4 = (w4[1][2] - w4[0][2]) / (w4[1][0] - w4[0][0])
    except:
        v4=0
    return v1,v2,v3,v4

