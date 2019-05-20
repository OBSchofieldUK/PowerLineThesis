function line2d = Line2Dregression(LinePoints)
%LINE2DREGRESSION Summary of this function goes here
%   Detailed explanation goes here

if LinePoints == 0
    line2d = NaN
    return
end
X = [];
Y = [];
N = 0;
if size(LinePoints,1) < size(LinePoints,2)
    N = size(LinePoints,2);
    X = LinePoints(1,:);
    Y = LinePoints(2,:);
else
    N = size(LinePoints,1);
    X = LinePoints(:,1);
    Y = LinePoints(:,2);
end




XX = X.*X;
XY = X.*Y;

Xsum = sum(X);
Ysum = sum(Y);
XXsum = sum(XX);
XYsum = sum(XY);

a = (N*XYsum-Xsum*Ysum)/(N*XXsum-Xsum^2);
b = (Ysum-a*Xsum)/N;

line2d = [b,a];
end

