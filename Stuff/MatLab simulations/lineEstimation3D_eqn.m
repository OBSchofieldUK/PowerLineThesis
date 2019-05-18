clear all;
%close all;
clc;

mm = 0.001;
cm = 0.01;
m  = 1;

solver = eqn2dTo3d();

camera.d = 22*mm;
camera.size.x = 15*mm / 2;
camera.size.y = 9/16*camera.size.x  /2;
camera.pixel.x = 1920;
camera.pixel.y = 1080;
camera.Xs = camera.pixel.x/(camera.size.x*2);
camera.Ys = camera.pixel.y/(camera.size.y*2);

lidar.seg = 9;
lidar.position = [5*cm;5*cm;0;0;0;0];
lidar.spread = [10;3];
lidar.overlap = 0;%
lidar.segSize = lidar.spread(1)/lidar.seg;
move = lidar.position(1:3);
rpy = lidar.position(4:6);
lidar.transform = makehgtform('translate', move,'xrotate',rpy(1),'yrotate',rpy(2),'zrotate',rpy(3));


%% Run Simulation
clc
figure(5)
clf;
subplot(2,1,1)
title('b error')
subplot(2,1,2)
title('a error')


% ######## INIT WORLD #############
Line.start = FRU2coord(5,-10,0);
Line.end   = FRU2coord(30,10,0);
Line.points = constuctLine(Line);

World = [Line,];


% ######### INITIAL CONDITIONS #####################
[World, IMU] = moveCamera(World,[13;0;0],[0;0;0]);
image = takePicture(World,camera);
last_line2d = Line2Dregression(image.pixel);

% ######### INIT KALMAN FILTER #######################
X_hat = [World(1).start;1;(World(1).end-World(1).start)];
X_hat = Normalize3Dline(X_hat);
X_hat = X_hat+randomMove(22,zeros([7,1]));
X_hat(4) = 1;
X_hat = Normalize3Dline(X_hat);

P = eye(7)*40;
P(4,4) = 0;
P(1,1) = 0;

H_dir = [   0 0 0 0 1 0 0;
            0 0 0 0 0 1 0;
            0 0 0 0 0 0 1];
            

H_pos =  [ 1 0 0 0 0 0 0;
           0 1 0 0 0 0 0;
           0 0 1 0 0 0 0];

Sigma_u = [ [1;1;1]*0.2 ; 0 ; [1;1;1]*0.05];
G = diag([1;1;1;0;1;1;1]);
%Q = (G*G'*Sigma_u(1:4)*Sigma_u(1:4)');
Q = diag(Sigma_u.^2);

Sigma_r_picImu_pos = ones([size(H_pos,1),1])*3; 
%R_picImu_pos = (Sigma_r_picImu_pos*Sigma_r_picImu_pos')*(H_pos*H_pos');
R_picImu_pos = diag(Sigma_r_picImu_pos.^2);

Sigma_r_picImu_dir = ones([size(H_dir,1),1])*0.2;
Sigma_r_picImu_dir(2) = 0.01; 
R_picImu_dir = diag(Sigma_r_picImu_dir.^2);


Sigma_r_lidar = ones([size(H_pos,1),1])*1;
% R_lidar = (Sigma_r_lidar*Sigma_r_lidar')*(H_pos*H_pos');
R_lidar = diag(Sigma_r_lidar.^2);
IMU_last = zeros([6,1]);
for n= 1:200
    % ################ 3D world Simulation #############################
    disp('it:'+string(n))
    l = LineTo3D(World(1));
    [World,IMU_n] = moveCamera(World,randomMove(0.1,[0,0,l(2)*0.1]),randomMove(0.004,[0,0,0]));
    IMU = IMU_n + IMU_last;
    image = takePicture(World,camera);
    showPicture(image)
% ################ DATA AQUISITION ######################################
    % ##### Line EST 2xPic + IMU #####
    ready_picImu = false;
    if length(image.pixel)>2
        line2d = Line2Dregression(image.pixel);
        if length(line2d)>1 & ~isnan(line2d) 
            
            lineIMUPic = solve2dTo3d(line2d,last_line2d,IMU,camera,solver);
            lineIMUPic(3) = abs(lineIMUPic(3));
            
            estLine = LineTo2D(lineIMUPic,camera,true);
            error = [abs(estLine(1)- line2d(1)); abs(atan(estLine(2))-atan(line2d(2)))];
            
            
            if error(1) < 10 && error(2)< 5
                PlotImageError(World,lineIMUPic,camera,line2d,n)
                ready_picImu = true;
            else
                disp('Line 2xPic + IMU Estimate FAILED')
            end
            IMU_last = zeros([6,1]);
            last_line2d = line2d;
        else
            figure(4)
            clf
            IMU_last = IMU_last + IMU;
        end
    end

    % #### Line EST Pic + Lidar ####
    lidar.data = lidarScan(World,lidar);
    printVector(lidar.data,'lidar data')
    Lidar3dPoints = LidarPointTo3D(lidar);
    ready_lidar = false;
    if size(Lidar3dPoints) == [3,1]
        ready_lidar = true;
    end
    plotLidar(lidar,camera)
% ################### KALMAN FILTER ESTIMATION ########################    

    % #### Prediction step ####
    F = F_Matrix(IMU);

    u = [IMU(1:3);0;IMU(4:6)];

    X_hat = F*X_hat;% + G.*u;
    X_hat = Normalize3Dline(X_hat); %Normalise line and make it a 6x1 vector

    P = F*P*F' + Q;

    % #### Update with PicIMU position data ####
    if ready_picImu
        S = H_pos*P*H_pos'+R_picImu_pos;
        K = (P*H_pos')/S;
        P_u = (eye(7)-K*H_pos)*P;

        e = lineIMUPic(1:3) - H_pos*X_hat;
        if e < 20 | n < 4
            X_hat_u = X_hat +K*e;
            X_hat = Normalize3Dline(X_hat_u);        
            P = P_u;
        end
    end
    % #### Update With PicIMU direction data ####
    if ready_picImu
        S = H_dir*P*H_dir'+R_picImu_dir;
        K = (P*H_dir')/S;
        P_u = (eye(7)-K*H_dir)*P;
        
        e = lineIMUPic(4:6) - H_dir*X_hat;
        %if norm(e) > norm(lineIMUPic(4:6) + H_dir*X_hat)
        %    e = lineIMUPic(4:6) + H_dir*X_hat;
        %end
        X_hat_u = X_hat +K*e;
        X_hat = Normalize3Dline(X_hat_u);        
        P = P_u;
    end
    % ###Update With Lidar data ###############
    if ready_lidar && false
        lid_line = [Lidar3dPoints; X_hat(5:7)];
        lidar_data = Normalize3Dline(lid_line);
        
        S = H_pos*P*H_pos'+R_picImu_pos;
        K = (P*H_pos')/S;
        P_u = (eye(7)-K*H_pos)*P;

        e = lidar_data(1:3) - H_pos*X_hat;

        X_hat_u = X_hat +K*e;
        X_hat = Normalize3Dline(X_hat_u);        
        P = P_u;
    end
    
% ################### EVALUATION ######################################
    line3d = LineTo3D(World(1));
    a_error = norm(line3d(4:6)-X_hat(5:7));
    if a_error > norm(line3d(4:6)+X_hat(5:7))
        a_error = norm(line3d(4:6)+X_hat(5:7));
    end
    b_error = norm(line3d(1:3)-X_hat(1:3));

    figure(5)
    subplot(2,1,1)
    plot(n,b_error,'b*')
    hold on

    subplot(2,1,2)
    plot(n,a_error,'b*')
    hold on


    %pause(2)       
end


function res = randomMove(sig,mu)
    res = zeros(size(mu));
    for n = 1:length(mu)
        r = rand(1);
        r = r*sig*2;
        r = r-sig;
        res(n) = mu(n)+r;
    end
end
function line3d = solve2dTo3d(line2d_cur, line2d_last,IMU,camera,solution)
    line2d_cur = PixelToRealLine2D(line2d_cur,camera);
    line2d_last = PixelToRealLine2D(line2d_last,camera);

    a1 = line2d_cur(2);
    b1 = line2d_cur(1);
    a2 = line2d_last(2);
    b2 = line2d_last(1);

    TM = makehgtform('translate', IMU(1:3),'xrotate',IMU(4),'yrotate',IMU(5),'zrotate',IMU(6))^(-1);

    line3d = double(solution(camera.d,a1,b1,a2,b2,TM(1,1),TM(1,2),TM(1,3),TM(1,4),TM(2,1),TM(2,2),TM(2,3),TM(2,4),TM(3,1),TM(3,2),TM(3,3),TM(3,4)));
    line3d = Normalize3Dline(line3d);
end