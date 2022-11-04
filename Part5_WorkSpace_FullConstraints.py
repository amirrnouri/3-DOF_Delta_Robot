from cmath import cos
import math
from math import sin, cos, tan
from operator import invert
from numpy import arange
import scipy.io as sio

e = 0.2563435195
f = 0.3464101615
r_e = 0.46
r_f = 0.20
offset_zero = 47
gearbox_ratio = 50


def delta_forward(theta1, theta2, theta3):
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)

    sin30 = sin(math.radians(30))
    cos30 = cos(math.radians(30))
    tan30 = tan(math.radians(30))
    tan60 = tan(math.radians(60))

    t = (f - e) * tan30 / 2
    x1 = 0
    y1 = -1 * (t + r_f * cos(theta1))
    z1 = -1 * r_f * sin(theta1)

    y2 = (t + r_f * cos(theta2)) * sin30
    x2 = y2 * cos30 / sin30
    z2 = -1 * r_f * sin(theta2)

    y3 = (t + r_f * cos(theta3)) * sin30
    x3 = -1 * y3 * cos30 / sin30
    z3 = -1 * r_f * sin(theta3)

    d = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = x1 * x1 + y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1 = (1 / d) * ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1))
    b1 = (-1 / (2 * d)) * ((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1))
    a2 = (-1 / d) * ((z2 - z1) * x3 - (z3 - z1) * x2)
    b2 = (1 / (2 * d)) * ((w2 - w1) * x3 - (w3 - w1) * x2)

    a = (a1 ** 2 + a2 ** 2 + 1)
    b = 2 * (a1 * b1 + a2 * (b2 - y1) - z1)
    c = (b1 ** 2 + (b2 - y1) ** 2 + z1 ** 2 - r_e ** 2)

    dd = b * b - 4.0 * a * c
    if (dd < 0):
        print("non existing point")
        return

    z0 = -0.5 * (b + dd ** 0.5) / a
    x0 = (a1 * z0 + b1)
    y0 = (a2 * z0 + b2)
    # due to rotation of coordinate systems:
    return -y0, x0, z0

def delta_calcAngleYZ(x0, y0, z0):
    y1 = -f / 2 * math.tan(math.radians(30))
    y0 -= 0.5 * math.tan(math.radians(30)) * e
    a = (x0 ** 2 + y0 ** 2 + z0 ** 2 + r_f ** 2 - r_e ** 2 - y1 ** 2) / (2 * z0)

    b = (y1 - y0) / z0
    d = -1 * (a + b * y1) ** 2 + r_f * (b * b * r_f + r_f)
    if (d < 0):
        return 'not in range'
    yj = (y1 - a * b - d ** 0.5) / (b * b + 1)
    zj = a + b * yj
    theta = 180 * math.atan(-1 * zj / (y1 - yj)) / math.pi + (180 if yj > y1 else 0)
    return theta*math.pi/180


def delta_inverse(x1, y1, z0):
    x0 = y1
    y0 = -x1
    cos120 = math.cos(math.radians(120))
    sin120 = math.sin(math.radians(120))
    t1 = delta_calcAngleYZ(x0, y0, z0)
    t2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, -1 * x0 * sin120 + y0 * cos120, z0)
    t3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, x0 * sin120 + y0 * cos120, z0)
    return t1, t2, t3


def delta_passiveJoints_angle(x,y):
    cos120 = math.cos(math.radians(120))
    sin120 = math.sin(math.radians(120))

    p1=abs(math.acos(y/r_e))
    p2=abs(math.acos((-sin120*(x)+cos120*y)/r_e))
    p3=abs(math.acos((sin120*(x)+cos120*y)/r_e))
    return math.pi/2-p1,math.pi/2-p2,math.pi/2-p3

if __name__ == "__main__":
    # for a cylinder with radius=r and height=h:
    r=0.328
    h=0.19
    #when ti is -45 degres:
    h0=0.28702944545996756
    xwork=[]
    ywork = []
    zwork = []

    for z in arange(-h0,-h0-h,-0.01):

        for t in arange(0,2*math.pi,0.1):
            for rrange in arange(r, 0.002, -0.01):
                x=rrange*math.cos(t)
                y=rrange*math.sin(t)
                t1, t2, t3 = delta_inverse(x, y, z)


                if  t1!='not in range' and t2!='not in range' and t3!='not in range':
                    if t1>-40/180*math.pi and t2>-40/180*math.pi and t3>-40/180*math.pi and t1<math.pi/2 and t2<math.pi/2 and t3<math.pi/2:
                        p1, p2, p3 = delta_passiveJoints_angle(x, y)
                        if p1<35/180*math.pi and p2<35/180*math.pi and p3<35/180*math.pi:
                            print('ok')
                            xwork.append(x)
                            ywork.append(y)
                            zwork.append(z)



    sio.savemat('allconst_xwork.mat', {'allconst_xwork':xwork})
    sio.savemat('allconst_ywork.mat', {'allconst_ywork':ywork})
    sio.savemat('allconst_zwork.mat', {'allconst_zwork':zwork})

