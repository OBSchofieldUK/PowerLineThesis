function out = lidarScan(World,Lidar)
%LIDARSCAN Summary of this function goes here
%   Detailed explanation goes here
out = ones([Lidar.seg,1])*9999;
rotate = zeros([length(out),1]);
segSize = Lidar.spread(1)/Lidar.seg;
t1 = Lidar.transform^-1;
for i = 1:length(out)
    X_rot = -Lidar.spread(1)/2 + (i-1)*segSize+segSize/2;
    rotate(i) = deg2rad(X_rot);
end


for n = 1:length(rotate)
    t2 = makehgtform('xrotate',rotate(n));
    t = t1*t2;
    for w = 1:length(World)
        Line = World(w);
        for i = 1:size(Line.points,2)
            point = [Line.points(:,i);1];
            p = t*point;
            dx_max = p(3)*tan(deg2rad(Lidar.spread(2)));
            dy_max = p(3)*tan(deg2rad(segSize*(1+Lidar.overlap)/2));
            if abs(p(1)) < dx_max && abs(p(2)) < dy_max
               if p(3) < out(n)
                   out(n) = p(3);
               end
            end    
        end
    end
    if out(n) == 9999
        out(n) = -1;
    end
end
end