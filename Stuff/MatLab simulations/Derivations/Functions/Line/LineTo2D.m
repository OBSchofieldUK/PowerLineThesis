function Line2D = LineTo2D(Line,camera,is4Image)
%LINETO2D Summary of this function goes here
%   Detailed explanation goes here

skipImageScale = false;

if exist('is4Image','var')
    skipImageScale = ~is4Image;
end

Line2D = zeros([4,1]);
i_cor = 0;
if size(Line,1) == 6
    i_cor = 0;
elseif size(Line,1) == 7
    i_cor = 1;
else
    error('Dimension of Line is wrong, must be either 6 or 7 row vector')
end

for i = 1:2
    Line2D(i) = Line(i)*camera.d/Line(3);
    Line2D(2+i) = camera.d*(Line(i+3+i_cor)*Line(3)-Line(i)*Line(6+i_cor)) / (Line(3)*(Line(3)+Line(6+i_cor)));
end




if ~skipImageScale
    Xs = camera.pixel.x/(camera.size.x*2);
    Ys = camera.pixel.y/(camera.size.y*2);
    for i = 1:2:3
        Line2D(i) = Line2D(i) * Xs;
        Line2D(i+1) = Line2D(i+1) * Ys;
    end
end


h = Line2D;
b = h(2)-(h(1)/h(3))*h(4);
a = h(4)/h(3);
Line2D = [b,a];

end

