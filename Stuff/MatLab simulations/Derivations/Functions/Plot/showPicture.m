function showPicture(image,X_hat)
if size(image.segments) > 0
%     figure(1)
%     clf
%     hold on
%     title('Top Down view')
%     xlabel('Right')
%     ylabel('Forward')
%     legendData = {};
%     for i = 1:size(image.segments)
%         Line = image.segments(i);
%         plot(Line.points(1,:),Line.points(3,:),'b*')
%         legendData = [legendData,['LinePoints ',num2str(i)]];
%     end
%     vec = [image.camera.size.x,image.camera.d] - [0,0];
%     vec = vec*1000;
%     X = [vec(1), 0, -vec(1)];
%     Y = [vec(2), 0, vec(2)];
%     plot(X,Y,'r')
%     
%     if exist('X_hat','var')
%         line([X_hat(1)-X_hat(5)*20, X_hat(1)+X_hat(5)*20],[X_hat(3)-X_hat(7)*20,X_hat(3)+X_hat(7)*20])
%     end
%     legendData = [legendData,'Camera Viewing angle'];
%     legend(legendData)
%     hold off
%     
%     figure(2)
%     clf
%     hold on
%     title('Side View')
%     xlabel('Forward')
%     ylabel('UP')
%     for i = 1:size(image.segments)
%         Line = image.segments(i);
%         plot(Line.points(3,:),Line.points(2,:),'b*')
%         
%     end
%     vec = [image.camera.d,image.camera.size.y] - [0,0];
%     vec = vec*1000;
%     
%     X = [vec(1), 0, vec(1)];
%     Y = [vec(2), 0, -vec(2)];
%     plot(X,Y,'r');
%     if(exist('X_hat','var')==1)
%         line([X_hat(3)-X_hat(7)*20, X_hat(3)+X_hat(7)*20],[X_hat(2)-X_hat(6)*20,X_hat(2)+X_hat(6)*20])
%     end
%     
%     legend(legendData)
%     hold off
end
if size(image.pixel,1) == 2
    figure(3)
    clf
    hold on
    title('Image')
    
    plot(image.pixel(1,:),image.pixel(2,:),'b*')
    ylim([image.ymin,image.y])
    xlim([image.xmin,image.x])
    %legend('pixel','prediction')
    hold off
else
    figure(3)
    clf
    hold on
    title('Image')
    ylim([image.ymin,image.y])
    xlim([image.xmin,image.x])
    hold off
end
end

