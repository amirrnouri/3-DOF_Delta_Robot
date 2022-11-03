from cmath import cos
import math
from math import sin,cos,tan
from operator import invert
from numpy import arange
import scipy.io as sio


e = 0.2563435195
f = 0.3464101615
r_e = 0.46
r_f = 0.20
offset_zero = 47
gearbox_ratio = 50


def delta_calcAngleYZ(x0,y0,z0):
    y1 = -f/2 * math.tan(math.radians(30))
    y0 -= 0.5 * math.tan(math.radians(30)) * e
    a = (x0**2 + y0**2 + z0**2 + r_f**2 - r_e**2 - y1**2)/(2*z0)

    b = (y1 - y0) / z0
    d = -1 * (a + b*y1)**2 + r_f*(b*b*r_f + r_f)
    if(d < 0):
        print("not in range")
        return
    yj = (y1 - a*b - d**0.5) / (b*b + 1)
    zj = a + b*yj
    theta = 180 * math.atan(-1 * zj / (y1 - yj)) / math.pi + (180 if yj > y1 else 0)
    return theta

def delta_inverse(x1,y1,z0):
    x0=y1
    y0=-x1
    cos120 = math.cos(math.radians(120))
    sin120 = math.sin(math.radians(120))
    t1 = delta_calcAngleYZ(x0,y0,z0) 
    t2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120 , -1*x0*sin120 + y0*cos120 , z0) 
    t3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120 , x0*sin120 + y0*cos120 , z0)  
    return t1/180*math.pi,t2/180*math.pi,t3/180*math.pi

if __name__ == "__main__":
    #center of EE position (m):
    x = 0.2
    y = 0.2
    z = -0.4

    t1,t2,t3 = delta_inverse(x,y,z)
print("t1,t2,t3,motors is :",t1*180/math.pi,t2*180/math.pi,t3*180/math.pi)



