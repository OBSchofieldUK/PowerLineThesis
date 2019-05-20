function plotLidar(Lidar,camera)
%PLOTLIDAR Summary of this function goes here
%   Detailed explanation goes here
points = LidarPointTo3D(Lidar);
figure(3)
hold on;
for u = 1:size(points,2)
    pix = [points(1,u)*camera.Xs;points(2,u)*camera.Ys].*(camera.d/points(3,u));
    plot(pix(1),pix(2),'r*');
end


end

