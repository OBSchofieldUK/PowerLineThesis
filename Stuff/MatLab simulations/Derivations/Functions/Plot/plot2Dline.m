function plot2Dline(line,figNum,color)
%PLOT2DLINEONIMAGE Summary of this function goes here
%   Detailed explanation goes here
a = line(2);
b = line(1);
fig = 3;
if exist('figNum','var')
    fig = figNum;
end
c = 'g';
if exist('color','var')
    c = color;
end

figure(fig)
hold on
x = xlim;
y = a*x+b;

lim = ylim;
if y(1)< lim(1)
    y(1) = lim(1);
elseif y(1) > lim(2)
    y(1) = lim(2);
end

x(1) = (y(1)-b)/a;

if y(2)< lim(1)
    y(2) = lim(1);
elseif y(2) > lim(2)
    y(2) = lim(2);
end

x(2) = (y(2)-b)/a;
plot(x,y,c)
end