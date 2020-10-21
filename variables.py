import numpy as np

m1 = 0.6
m2 = 0.2
g = 9.8
L = 5
l = L/2             # CoM of uniform rod                 
I = (1./12) * m2 * (L**2)
b = .2           # dampending coefficient

divisor = m1*m2*(l**2) + I*(m1 + m2)
A21 = (m2**2)*g*(l**2) / divisor
A31 = m2*g*l*(m1+m2) / divisor
A22 = -b*(m2*(l**2)+I) / divisor
A32 = -m2*l*b / divisor
B2 = (m2*(l**2)+I) / divisor
B3 = m2*l /divisor

A = np.matrix([[0, 0, 1, 0],
               [0, 0, 0, 1],
               [0, A21, A22, 0],
               [0, A31, A32, 0]])

B = np.matrix([[0], [0], [B2], [B3]])