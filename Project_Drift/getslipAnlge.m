function ang = getslipAnlge(vy,vx,phi,th)
% if(vy == 0 && vx == 0)
%     ang = phi;
% else
% temp = atan2(vy,vx);
% if(temp < 0)
%     temp = 2*pi + temp;
% end
% th = rem(th,2*pi);
% if(th < 0)
%     th = 2*pi + th;
% end
% temp = temp - th;
% ang = -(temp - pi/2) + phi ;
% end

if(vy == 0 && vx == 0)
    ang = phi;
else
slip = atan2(vy,vx);
if(slip < 0)
    slip = 2*pi + slip;
end
th = th + pi/2;
th = rem(th,2*pi);
total = th + phi;
ang = total - slip;
if(abs(ang) > pi)
    angabs = abs(ang) - 2*pi;
    ang = angabs*sign(ang);
end
end
end