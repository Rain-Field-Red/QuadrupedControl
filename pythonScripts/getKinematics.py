
from sympy import symbols
from sympy import sin, cos, asin, acos
from sympy import simplify, diff
from sympy import solve, Eq
from sympy import linsolve
from sympy import pi

from math import sqrt
#from math import pi

H0, R3, R2, R1, R0 = symbols('H0 R3 R2 R1 R0')
theta00, theta01, theta02, theta03 = symbols('theta00 theta01 theta02 theta03')

# 足端相对于髋关节位置
x0 = -(R3*cos(theta02)-R2*cos(theta01-theta02)+R1*cos(theta00-theta01+theta02))
tmp = R3*sin(theta02)-R2*sin(theta01-theta02)+R1*sin(theta00-theta01+theta02)
y0 = -(H0+tmp)*cos(theta03)
z0 = (H0+tmp)*sin(theta03)

# 雅克比矩阵（仅与足端位置相关的部分）
j11 = diff(x0, theta00)
j12 = diff(x0, theta01)
j13 = diff(x0, theta02)
j14 = diff(x0, theta03)

j21 = diff(y0, theta00)
j22 = diff(y0, theta01)
j23 = diff(y0, theta02)
j24 = diff(y0, theta03)

j31 = diff(z0, theta00)
j32 = diff(z0, theta01)
j33 = diff(z0, theta02)
j34 = diff(z0, theta03)

print('1 row of jacobian')
print(j11)
print(j12)
print(j13)
print(j14)
print('2 row of jacobian')
print(j21)
print(j22)
print(j23)
print(j24)
print('3 row of jacobian')
print(j31)
print(j32)
print(j33)
print(j34)






