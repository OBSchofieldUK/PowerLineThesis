function H = H_Matrix(camera)
%H_MATRIX construct a H matrix for extended kalman filter
%   Detailed explanation goes here
syms x0 y0 z0 x y z

d = camera.d;
X_scale = camera.pixel.x/(2*camera.size.x);
Y_scale = camera.pixel.y/(2*camera.size.y);

f1 = x0*d/z0*X_scale;
%disp('f1 = ')
%pretty(f1)

f2 = y0*d/z0*Y_scale;
%disp('f2 = ')
%pretty(f2)

f3 = d*(x*z0-x0*z)/(z0*(z0+z))*X_scale;
%disp('f3 = ')
%pretty(f3)

f4 = d*(y*z0-y0*z)/(z0*(z0+z))*Y_scale;
%disp('f4 = ')
%pretty(f4)
syms r
line3d = [x0;y0;z0;r;x;y;z];

f_array = [f2-(f1/f3)*f4 ; f4/f3];

H1=sym(zeros([size(f_array,1),size(line3d,1)]));

for row = 1:size(H1,1)
    for col = 1:size(H1,2)
        
        H1(row,col)=diff(f_array(row),line3d(col));
        
    end
end

syms H(x0,y0,z0,x,y,z)
H(x0,y0,z0,x,y,z) = H1;




end

