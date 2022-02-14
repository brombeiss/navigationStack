import math
import numpy as np

from cv2 import sqrt


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def calcMovement(dest, pos, yaw):
    x_dest = dest[0]
    y_dest = dest[1]
    x_pos = pos[0]
    y_pos = pos[1]
    dx = x_dest - x_pos
    dy = y_dest -y_pos
    forward_by_m = math.sqrt(dx*dx+dy*dy)


    # yaw (Drehung um z-Achse) im Uhrzeigersinn im Lidar negativ behaftet
    # Umrechnung in Grad --> 90°(Uhrzeigersinn)=-0,5 --> 1°=0,005555
    # 
    print (math.degrees(math.atan(dest[1]/dest[0])) )
    angel = math.degrees(math.atan(dest[1]/dest[0]))
    turn_by_angel =  angel - yaw
    
    return(forward_by_m,turn_by_angel)
    

#d = calcMovement((-2.0,1.0),(0.0,0.0),0.0)
#print(d)
(rho,phi) = cart2pol(5,2)
print(rho,90.0-math.degrees(phi))