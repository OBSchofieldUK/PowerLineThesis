function points = LidarPointTo3D(lidar)
%LIDARPOINTTO3D Summary of this function goes here
%   Detailed explanation goes here
points = [];
for seg = 1:lidar.seg
   if lidar.data(seg) > 0
       angle = -lidar.spread(1)/2 + (seg-1)*lidar.segSize+lidar.segSize/2;
       angle = deg2rad(angle);
       point = [0;sin(angle)*lidar.data(seg); cos(angle)*lidar.data(seg);1];
       p = lidar.transform*point;
       points = [points, p(1:3)];
   end
end
end

