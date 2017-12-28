model_params;
filename = ['scene3.gif'];
wheelWidth = 0.07;
wd = 0.5;
figure;
% global steering_Angle; 

xmax = max(states(:,1));
ymax = max(states(:,2));
xmin = min(states(:,1));
ymin = min(states(:,2));
minLim = min(xmin,ymin);
maxLim = max(xmax,ymax);

for i = 1:length(time)
    
     x = states(i,1);
     y = states(i,2);
    th = states(i,3);
%     dx = states(i,4);
%     dy = states(i,5);
%     dth = states(i,6);
%     phi_d = steering_Angle(i);
    phi_d = states(i,9);
%     
    thrust = states(i,10)/500*(maxLim-minLim);
    
% %     if(time(i) > 0.7 && time(i) < 1.6)
% %         phi_d = 1;
% %         phi_d = 1;
% %     else
% %         phi_d = 0;
% %         phi_d = 0;    
% %     end
% %     
% %     if(t < 2)
% %         thrust = 100*time(i)^2;
% %     else
% %         thrust = 0;    
% %     end
    
    
    corner1 = [x - a*1.2*sin(th) - c*1.2*(cos(th)); y + a*1.2*cos(th) - c*1.2*(sin(th))];
    corner2 = [x - a*1.2*sin(th) + c*1.2*(cos(th)); y + a*1.2*cos(th) + c*1.2*(sin(th))];
    corner3 = [x + b*1.2*sin(th) + c*1.2*(cos(th)); y - a*1.2*cos(th) + c*1.2*(sin(th))];
    corner4 = [x + b*1.2*sin(th) - c*1.2*(cos(th)); y - a*1.2*cos(th) - c*1.2*(sin(th))];
    
    wheel1cen = [x - a*sin(th) - c*(cos(th)); y + a*cos(th) - c*(sin(th))];
    wheel2cen = [x - a*sin(th) + c*(cos(th)); y + a*cos(th) + c*(sin(th))];
    wheel3cen = [x + b*sin(th) + c*(cos(th)); y - a*cos(th) + c*(sin(th))];
    wheel4cen = [x + b*sin(th) - c*(cos(th)); y - a*cos(th) - c*(sin(th))];
    
    wh1p1 = wheel1cen + wd*[sin(phi_d+th);-cos(phi_d+th)];
    wh1p2 = wheel1cen + wd*[-sin(phi_d+th);cos(phi_d+th)];
    wh2p1 = wheel2cen + wd*[sin(phi_d+th);-cos(phi_d+th)];
    wh2p2 = wheel2cen + wd*[-sin(phi_d+th);cos(phi_d+th)];
    wh3p1 = wheel3cen + wd*[sin(th);-cos(th)];
    wh3p2 = wheel3cen + wd*[-sin(th);cos(th)];
    wh4p1 = wheel4cen + wd*[sin(th);-cos(th)];
    wh4p2 = wheel4cen + wd*[-sin(th);cos(th)];
    arrow1 = thrust*[-sin(th);cos(th)];
    
% alpha1 = getslipAnlge((dy - dth*c),(dx - dth*a),phi_d,th);
% alpha2 = getslipAnlge((dy + dth*c),(dx - dth*a),phi_d,th);
% alpha3 = getslipAnlge((dy + dth*c),(dx + dth*b),0,th);
% alpha4 = getslipAnlge((dy - dth*c),(dx + dth*b),0,th);

% if (states(i,8) <= -100)
%     k = -100;
% else
%     k = 0;
% end
% F1 = magicFormula(alpha1*180/pi,k)*u*m*g;
% F2 = magicFormula(alpha2*180/pi,k)*u*m*g;
% F3 = magicFormula(alpha3*180/pi,k)*u*m*g;
% F4 = magicFormula(alpha4*180/pi,k)*u*m*g;
% 
%     f1p = F1*[-cos(phi_d+th);-sin(phi_d+th)]/2;
%     f2p = F2*[-cos(phi_d+th);-sin(phi_d+th)]/2;
%     f3p = F3*[-cos(th);-sin(th)]/2;
%     f4p = F4*[-cos(th);-sin(th)]/2;

    plot(x,y,'ko')
    hold on
    patch([corner1(1),corner2(1),corner3(1),corner4(1)],...
        [corner1(2),corner2(2),corner3(2),corner4(2)],...
        [0.8 0.5 0.5])
     line([wh1p1(1),wh1p2(1)],...
        [wh1p1(2),wh1p2(2)],'LineWidth',5)
    line([wh2p1(1),wh2p2(1)],...
        [wh2p1(2),wh2p2(2)],'LineWidth',5)
    line([wh3p1(1),wh3p2(1)],...
        [wh3p1(2),wh3p2(2)],'LineWidth',5)
    line([wh4p1(1),wh4p2(1)],...
        [wh4p1(2),wh4p2(2)],'LineWidth',5)
    
    quiver(wh3p1(1),wh3p1(2),arrow1(1),arrow1(2),0,'r','LineWidth',2)
    quiver(wh4p1(1),wh4p1(2),arrow1(1),arrow1(2),0,'r','LineWidth',2)
%      quiver(wheel1cen(1),wheel1cen(2),f1p(1),f1p(2),0,'g','LineWidth',2)
%     quiver(wheel2cen(1),wheel2cen(2),f2p(1),f2p(2),0,'c','LineWidth',2)
%      quiver(wheel3cen(1),wheel3cen(2),f3p(1),f3p(2),0,'g','LineWidth',2)
%     quiver(wheel4cen(1),wheel4cen(2),f4p(1),f4p(2),0,'c','LineWidth',2)
    
    axis([minLim-1 maxLim+1 minLim-1 maxLim+1])
    xlabel('X')
    ylabel('Y')
    text(.5,2.5,['t = ' num2str(time(i))] )
  
    
    hold off
    
    

    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i  == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.01);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.01);
    end
    
    pause(.1)
end
