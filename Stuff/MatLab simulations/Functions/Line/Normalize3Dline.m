function newline = Normalize3Dline(line,setDim6)
%NORMALIZE3DLINE Summary of this function goes here
%   Detailed explanation goes here
if size(line,1) == 6
    i_cor = 0;
elseif size(line,1) == 7
    i_cor = 1;
else
    error('Dimension of Line is wrong, must be either 6 or 7 row vector')
end
if ~exist('setDim6','var')
     setDim6 = false;
end

vec=line(i_cor+4:6+i_cor);
vec = vec/norm(vec);
if vec(3) < 0
    vec= -vec;
end
t=-line(1)/vec(1);

line(1) = line(1)+t*vec(1);
line(2) = line(2)+t*vec(2);
line(3) = line(3)+t*vec(3);

if i_cor == 0
    newline = [line(1:3);vec(1:3)];
else
    newline = [line(1:3);1;vec(1:3)];
end

if setDim6 && i_cor == 1
    newline = [newline(1:3);newline(5:7)];
    
end

end

