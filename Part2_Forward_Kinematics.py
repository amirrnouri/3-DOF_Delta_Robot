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


def delta_forward(theta1 , theta2, theta3):
    


    sin30 = sin(math.radians(30))
    cos30 = cos(math.radians(30))
    tan30 = tan(math.radians(30))
    tan60 = tan(math.radians(60))

    t = (f - e) * tan30 / 2    
    x1 = 0
    y1 = -1 * (t + r_f*cos(theta1))
    z1 = -1 * r_f*sin(theta1)

    y2 = (t + r_f*cos(theta2)) * sin30
    x2 = y2 * cos30 / sin30
    z2 = -1 * r_f * sin(theta2)


    y3 = (t + r_f*cos(theta3))*sin30
    x3 = -1 * y3* cos30 / sin30 
    z3 = -1 * r_f*sin(theta3)

    d = (y2-y1)*x3-(y3-y1)*x2

    w1 = x1*x1 + y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3

    a1=(1/d)*((z2-z1)*(y3-y1)-(z3-z1)*(y2-y1))
    b1=(-1/(2*d))*((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))
    a2=(-1/d)*((z2-z1)*x3-(z3-z1)*x2)
    b2=(1/(2*d))*((w2-w1)*x3-(w3-w1)*x2)

    a=(a1**2+a2**2+1)
    b=2*(a1*b1+a2*(b2-y1)-z1)
    c=(b1**2+(b2-y1)**2 +z1**2-r_e**2)

    dd = b*b - 4.0*a*c
    if (dd < 0):
        print("non existing point")
        return 'not in range'

    z0 = -0.5*(b+dd**0.5)/a
    x0 = (a1*z0 + b1)
    y0 = (a2*z0 + b2)
    #due to rotation of coorinates:
    return -y0,x0,z0


if __name__ == "__main__":
    #theta_1 degrees in radians:

    t1=-math.pi/4
    t2=-math.pi/4
    t3=-math.pi/4
    x,y,z = delta_forward(t1,t2,t3)

    print("x,y,z,EE is :", x, y, z)



