function PlotImageError(World,X_hat,camera,line2d,n)
%PLOTIMAGEERROR Summary of this function goes here
%   Detailed explanation goes here
line3d = LineTo3D(World(1));
X_hat2di = LineTo2D(X_hat,camera,true);
X_hat2d = LineTo2D(X_hat,camera,false);
X_hat2di(1) = X_hat2di(1)-1;

plot2Dline(X_hat2di,3,'m')
plot2Dline(line2d,3,'g')

%plot2Dline(X_hat2d,4,'m')
%plot2Dline(LineTo2D(LineTo3D(World(1)),camera,false),4,'g')

printVector(X_hat,'Estimat')
printVector(line3d,'Real   ')
a_error = norm(line3d(4:6)-X_hat(4:6));
if a_error > norm(line3d(4:6)+X_hat(4:6))
    a_error = norm(line3d(4:6)+X_hat(4:6));
end
b_error = norm(line3d(1:3)-X_hat(1:3));

figure(5)
subplot(2,1,1)
title('Position error')
plot(n,b_error,'r*')
ylabel('meters')
hold on

subplot(2,1,2)
title('Direction error')
plot(n,a_error,'r*')
xlabel('iteration')
hold on
end

