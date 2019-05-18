clear all;
close all;
clc;

mm = 0.001;
cm = 0.01;
m  = 1;

Line.start = FRU2coord(5,-10,0);
Line.end   = FRU2coord(30,10,0);
Line.points = constuctLine(Line);

World = [Line,];


camera.d = 22*mm;
camera.size.x = 15*mm / 2;
camera.size.y = 9/16*camera.size.x  /2;
camera.pixel.x = 1920;
camera.pixel.y = 1080;

l = LineTo3D(World(1));
[World,IMU] = moveCamera(World,randomMove(0.4,[0,0,l(2)*0.2]),randomMove(0.004,[0,0,0]));

line3d = LineTo3D(World(1));
image = takePicture(World,camera);
line2d = Line2Dregression(image.pixel)


P = image.pixel(:,30)
Xs = camera.pixel.x/(camera.size.x*2);

syms x0 z0 dx dz point t xs d

eqn = point == (x0+dx*t)*d*xs/(z0+dz*t)

sol = solve(eqn,t)

syms t(x0,z0,dx,dz,point,xs,d)

t(x0,z0,dx,dz,point,xs,d) = sol

t1 = double(t(line3d(1),line3d(3),line3d(4),line3d(6),P(1),Xs,camera.d))

P3d = line3d(1:3)+t1*line3d(4:6)
P2d1 = P3d(1)*camera.d*Xs/P3d(3)

%P2d = (line3d(1)+line3d(4)*t1)*camera.d*Xs/(line3d(3)+line3d(6)*t1)

function res = randomMove(sig,mu)
    res = zeros(size(mu));
    for n = 1:length(mu)
        r = rand(1);
        r = r*sig*2;
        r = r-sig;
        res(n) = mu(n)+r;
    end
end