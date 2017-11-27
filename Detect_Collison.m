function [Collison_val] = Detect_Collison(C,R,P1,P2)

% Plotting the circle in the form of polyline
I = [0:0.05:2*pi]';
x = R*cos(I)+C(1);
x = [x;x(1,:)];
y = R*sin(I)+C(2);
y = [y;y(1,:)];
% plot(x,y);
% xlim([-100,100]);
% ylim([-100,100]);
% hold on;
% plot([P1(1),P2(1)],[P1(2),P2(2)])


% algorithm for collison detection
% perpendicular projection of point from center of the circle to line 
xi = (C(2)-P1(2)-(C(1)*((P1(1)-P2(1))/(P2(2)-P1(2))))+(P1(1)*((P2(2)-P1(2))/(P2(1)-P1(1)))))/(((P2(2)-P1(2))/(P2(1)-P1(1)))+((P2(1)-P1(1))/(P2(2)-P1(2))));
yi = C(2)+(((P1(1)-P2(1))/(P2(2)-P1(2)))*(xi-C(1)));
d = sqrt(((C(1)-xi)^2)+((C(2)-yi)^2));
% hold on;
% scatter(xi,yi,'k')
if R<d
    Collison_val = 0;
else
    dp1 = sqrt(((C(1)-P1(1))^2)+((C(2)-P1(2))^2));
    if R>dp1
        Collison_val = 1;
    else
        dp2 = sqrt(((C(1)-P2(1))^2)+((C(2)-P2(2))^2));
    if R>dp2
        Collison_val = 1;
    else
        mp1p2 = sqrt(((P2(1)-P1(1))^2)+((P2(2)-P1(2))^2));
        mp1 = sqrt(((P1(1)-xi)^2)+((P1(2)-yi))^2);
        mp2 = sqrt(((P2(1)-xi)^2)+((P2(2)-yi))^2);
        if abs(mp1p2 - (mp1+mp2))<0.01;
            Collison_val = 1;
        else
            Collison_val = 0;
        end
    end
    end   
end

end