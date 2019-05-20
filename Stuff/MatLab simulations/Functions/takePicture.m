function image = takePicture(lineArray,camera)

%% 3D figure to 2D Image plane
image.cameraPlane = {};
for i1 = 1:size(lineArray)
    Line = lineArray(i1);
    for i = 1:size(Line.points,2)
        point = [Line.points(1,i)*camera.d/Line.points(3,i);
                 Line.points(2,i)*camera.d/Line.points(3,i)];
        if camera.size.x > abs(point(1))
            if camera.size.y > abs(point(2))
                image.cameraPlane = [image.cameraPlane, point];
            end
        end   
    end
end

image.cameraPlane = cell2mat(image.cameraPlane);
image.pixel = 0;
image.segments=lineArray;
image.camera=camera;

%% Image Plane to pixel conversion
if size(image.cameraPlane,1) == 2
    %figure(4)
    %clf
    %plot(image.cameraPlane(1,:),image.cameraPlane(2,:),'*b')
    
    image.pixel = zeros(size(image.cameraPlane));
    Xs = camera.pixel.x/(camera.size.x*2);
    Ys = camera.pixel.y/(camera.size.y*2);
    for i = 1:size(image.cameraPlane,2)
        point = image.cameraPlane(:,i);
        image.pixel(1,i) = round(point(1)*Xs,0);
        image.pixel(2,i) = round(point(2)*Ys,0);      
    end

end



image.x = camera.pixel.x/2;
image.y = camera.pixel.y/2;
image.ymin = -image.y;
image.xmin = -image.x;
end

