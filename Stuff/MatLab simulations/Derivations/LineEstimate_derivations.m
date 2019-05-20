clear all;
close all;
clc;

syms px0 py0 px py

syms x0 y0 z0 x y z

syms d Xs Ys

f1 = x0*d/z0*Xs;
disp('f1 = ')
pretty(f1)

f2 = y0*d/z0*Ys;
disp('f2 = ')
pretty(f2)

f3 = d*(x*z0-x0*z)/(z0*(z0+z))*Xs;
disp('f3 = ')
pretty(f3)

f4 = d*(y*z0-y0*z)/(z0*(z0+z))*Ys;
disp('f4 = ')
pretty(f4)

line3d = [x0;y0;z0;x;y;z];
line2d = [px0;py0;px;py];
f_array = [f1 ; f2 ; f3 ; f4];

H1=sym(zeros([4,6]));

for row = 1:size(H1,1)
    for col = 1:size(H1,2)
        
        H1(row,col)=diff(f_array(row),line3d(col));
        
    end
end

syms H(x0,y0,z0,x,y,z,d)

H(x0,y0,z0,x,y,z,d) = H1

%H(1,2,3,4,5,6,7)