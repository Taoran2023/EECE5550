%problem1
clear all
clc
num = [1]
denom = [2]

Gp = tf(num, denom)
H = [1]

Kp = 1
Ki = 0
Kd = 0
Gc = pid( Kp, Ki, Kd)

Mc = feedback( Gc*Gp, H)
step(Mc)
