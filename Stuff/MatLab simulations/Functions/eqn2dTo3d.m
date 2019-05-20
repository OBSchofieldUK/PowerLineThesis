function solution = eqn2dTo3d()
%EQN2DTO3D Summary of this function goes here
%   Detailed explanation goes here
%% Image Variables
syms a1 a2 b1 b2
syms d

%% Line Variables
syms x0 y0 z0 xv yv zv

%% Equations 1
eq1 = x0 == 0;
disp("Equation 1")
pretty(eq1)

eq2 = y0 == b1*z0/d;
disp("Equation 2")
pretty(eq2)

dy = d*(yv*z0-y0*zv)/(z0*(z0+zv));
dx = d*(xv*z0-x0*zv)/(z0*(z0+zv));

eq3 = a1 == dy/dx; 
disp("Equation 3")
pretty(eq3)

eq4 = 1 == (xv^2+yv^2+zv^2)^(1/2);
disp("Equation 4")
pretty(eq4)

%%  transform matrix 1
syms t00 t01 t02 t03
syms t10 t11 t12 t13
syms t20 t21 t22 t23


t1 = [t00 t01 t02 t03; t10 t11 t12 t13; t20 t21 t22 t23; 0 0 0 1];
r1 = [t00 t01 t02; t10 t11 t12; t20 t21 t22];

res = t1*[x0; y0; z0; 1];

x01_t = res(1);
y01_t = res(2);
z01_t = res(3);

res1 = r1*[xv; yv; zv];

xv1 = res1(1);
yv1 = res1(2);
zv1 = res1(3);

%%  normalize line
t = -x01_t/xv1;

x01 = x01_t + xv1*t;
y01 = y01_t + yv1*t;
z01 = z01_t + zv1*t;

%% equations 2
eq5 = y01 == b2*z01/d;
disp("Equation 5")
pretty(eq5)

dy1 = d*(yv1*z01-y01*zv1)/(z01*(z01+zv1));
dx1 = d*(xv1*z01-x01*zv1)/(z01*(z01+zv1));

eq6 = a2 == dy1/dx1; 
disp("Equation 6")
pretty(eq6)

%% Solv equation
sol = solve(eq1,eq2,eq3,eq4,eq5,eq6);

syms solution(d,a1,b1,a2,b2,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23)

solution(d,a1,b1,a2,b2,t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23) = [sol.x0(1);sol.y0(1);sol.z0(1);sol.xv(1);sol.yv(1);sol.zv(1)];

end

