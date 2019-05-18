function [WorldRet,IMU] = moveCamera(World,moveFRU,RPY)
%MOVECAMERA Summary of this function goes here
%   Detailed explanation goes here
move = -FRU2coord(moveFRU(1),moveFRU(2),moveFRU(3));
rpy = -FRU2coord(RPY(1),RPY(2),RPY(3));
t = makehgtform('translate', move,'xrotate',rpy(1),'yrotate',rpy(2),'zrotate',rpy(3));

WorldRet = {};
for i = 1:size(World)
    Line = World(i);
    res = t*[Line.start;1];
    Line.start = res(1:3)*(1/res(4));
    
    res = t*[Line.end;1];
    Line.end = res(1:3)*(1/res(4));

    Line.points = constuctLine(Line);
    
    
    WorldRet = [WorldRet, Line];
end

WorldRet = cell2mat(WorldRet);
IMU_old = [move;rpy];
move = move + noise(0.001,[0,0,0])';
rpy = rpy + noise(0.01,[0,0,0])';
IMU = [move;rpy];
IMU_signal2Noise_ratio =(IMU_old-IMU)./IMU_old*100;
TheNoise = IMU-IMU_old;


%printVector(IMU_signal2Noise_ratio,'IMU-dif')
%printVector(TheNoise,'Noise')
end

function res = noise(sig,mu)
    res = zeros(size(mu));
    for n = 1:length(mu)
        r = rand(1);
        r = r*sig*2;
        r = r-sig;
        res(n) = mu(n)+r;
    end
end

function printVector(vector,name)
    text = string(name) + ' [' + string(vector(1));
    for i = 2:length(vector)
       text = text +', '+ string(vector(i)); 
    end
    text = text +']';
    disp(text)
end