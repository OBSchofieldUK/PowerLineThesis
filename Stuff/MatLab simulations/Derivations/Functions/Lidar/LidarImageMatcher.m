function pixPoints = LidarImageMatcher(lidar,image)
%LIDARIMAGEMATCHER Summary of this function goes here
%   Detailed explanation goes here
points = LidarPointTo3D(lidar);
pixPoints = zeros([size(points,2),2]);
for u = 1:size(points,2)
    pix = [points(1,u)*camera.Xs;points(2,u)*camera.Ys].*(camera.d/points(3,u));
    dist = 9999;
    close = -1;
    for j = 1:length(image.pixel)
        d = norm(pix-image.pixel(:,j));
        if d < dist
           close = j;
           dist = d;
        end
    end
    pixPoints(u,:) = image.pixel(:,close);
end

end

