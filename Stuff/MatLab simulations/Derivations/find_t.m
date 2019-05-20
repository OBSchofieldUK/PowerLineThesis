clear all
close all
clc

syms x0 z0 dx dz point t xs d

eqn = point == (x0+dx*t)*d*xs/(z0+dz*t)

sol = solve(eqn,t)

syms t(x0,z0,dx,dz,point,t,xs,d)

t(x0,z0,dx,dz,point,xs,d) = sol
