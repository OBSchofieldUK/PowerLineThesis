function line2Dreal = PixelToRealLine2D(line2D,camera)
%PIXELTOREALLINE2D Summary of this function goes here
%   Detailed explanation goes here
line2Dreal = line2D;
Xs = camera.pixel.x/(camera.size.x*2);
Ys = camera.pixel.y/(camera.size.y*2);

line2Dreal(1) = line2D(1)/Ys;
line2Dreal(2) = line2D(2)*Xs/Ys;

end

