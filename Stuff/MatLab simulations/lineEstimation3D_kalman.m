clear all;
close all;
clc;

figure(5)
clf;
subplot(2,1,1)
title('b error')
subplot(2,1,2)
title('a error')

mm = 0.001;
cm = 0.01;
m  = 1;

Line.start = FRU2coord(5,-10,0);
Line.end   = FRU2coord(30,10,0);
Line.points = constuctLine(Line);

World = [Line,];


camera.d = 22*mm;
camera.size.x = 15*mm / 2;
camera.size.y = 9/16*camera.size.x  /2;
camera.pixel.x = 1920;
camera.pixel.y = 1080;

H = H_Matrix(camera);
H_ = double(H(1,2,3,4,5,6))

return
simLength = 300;

%X_hat = [Line.start;1;(Line.end-Line.start)];
X_hat = [Line.start;1;(Line.end-Line.start)];
X_hat = Normalize3Dline(X_hat);
X_hat = X_hat+randomMove(22,zeros([7,1]));

X_hat = [0;2;3;1;1;1;1];
X_hat = Normalize3Dline(X_hat);
X_hat_start = X_hat;

P = eye(7)*22;
P(4,4) = 0;
Sigma_r = ones([size(H_,1),1])*0.9;
Sigma_u = ones(size(X_hat))*0.2;
Sigma_u(4) = 0;
for r = 1:10
    Sigma_r = ones([size(H_,1),1])*r/10;
    for u = 1:10
        Sigma_u = ones(size(X_hat))*u/10;
        Sigma_u(4) = 0;
        
        data_mean = zeros([100,3]);
        for reTry = 1:100
            
            Line.start = FRU2coord(5,-10,0);
            Line.end   = FRU2coord(30,10,0);
            Line.points = constuctLine(Line);
            World = [Line,];
            
            P = eye(7)*22;
            P(4,4) = 0;

            X_hat = X_hat_start;

            figure(5)
            clf;
            subplot(2,1,1)
            title('b error')
            subplot(2,1,2)
            title('a error')
            
            data = zeros([simLength,2]);
            underOne = false;
            disp(char(['it: U(',num2str(u),'), R(',num2str(r),'), reTry(',num2str(reTry),')']))
            for n = 1:simLength
                %% 3D WORLD 
                l = LineTo3D(World(1));
                [World,IMU] = moveCamera(World,randomMove(0.4,[0,0,l(2)*0.2]),randomMove(0.004,[0,0,0]));
                image = takePicture(World,camera);
                %showPicture(image,X_hat)

                %% Kalman Filter
                F = F_Matrix(IMU);
                G = [1;1;1;0;1;1;1];
                u1 = [IMU(1:3);0;IMU(4:6)];

                X_hat = F*X_hat;% + G.*u;
                X_hat = Normalize3Dline(X_hat);
                Q = diag(Sigma_u); %(G.*Sigma_u)*(Sigma_u'.*G');

                P = F*P*F' + Q;

                H_ = H(X_hat(1),X_hat(2),X_hat(3),X_hat(5),X_hat(6),X_hat(7));
                H_ = double(H_);
                %R = diag(Sigma_r); %(Sigma_r*Sigma_r')*(H_*H_');
                R = diag(Sigma_r)*H_*H_'*diag(Sigma_r)';

                S = H_*P*H_'+R;
                K = (P*H_')/S;
                P_u = P-K*H_*P;
                if length(image.pixel)>2
                    line2d = Line2Dregression(image.pixel);
                    if ~isnan(line2d)
                        e = Line2Dregression(image.pixel) - LineTo2D(X_hat,camera);
                        %e = e*0.01;
                        X_hat_u = X_hat +K*e';

                        P = P_u;
                        X_hat = Normalize3Dline(X_hat_u);
                    else
                        data(n,:) = [NaN,NaN];
                    end
                else
                    data(n,:) = [NaN,NaN];
                end
                
                %% Plotting
                line3d = LineTo3D(World(1));
                a_error = norm(line3d(4:6)-X_hat(5:7));
                b_error = norm(line3d(1:3)-X_hat(1:3));

                if b_error < 1 | underOne
                    underOne = true;
                    data(n,:) = [a_error, b_error];
                else
                   data(n,:) = [NaN,NaN]; 
                end

                figure(5)
                subplot(2,1,1)
                plot(n,b_error,'r*')
                hold on

                subplot(2,1,2)
                plot(n,a_error,'r*')
                hold on
                
                %pause(0.5)
            end
            
            data_mean(reTry,2) = mean(data(:,1),'omitnan');
            data_mean(reTry,2) = mean(data(:,2),'omitnan');
            
            if max(data(:,2)) > 100
                data_mean(reTry,2) = NaN;
                data_mean(reTry,3) = 1;
            end
        end
        figure(6)
        l = (r-1)*10+u;
        plot(l,mean(data_mean,'omitnan'),'r*');
        plot(l,sum(data_mean(:,3)),'b*');
        hold on;
    end
end


return

figure(4)
hold on
Line = World(1);
aLine = zeros([6,1]);
aLine(1:3) = Line.start;
aLine(4:6) = Line.end - Line.start;
line2d= LineTo2D(aLine,camera);
plot(line2d(1),line2d(2),'rx')
plot(line2d(1)+line2d(3),line2d(2)+line2d(4),'rx')
hold off




