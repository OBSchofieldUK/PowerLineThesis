function F = F_Matrix(input)
%F_MATRIX Summary of this function goes here
%   Detailed explanation goes here
F = zeros(7);
F(1:4,1:4) = makehgtform('translate',input(1:3),'xrotate',input(4),'yrotate',input(5),'zrotate',input(6));
F(5:7,5:7) = F(1:3,1:3);

end

