function [pl_l1] = Arm_Plotter(Theta1,Theta2,Theta3)

%lengths of arms
L1 = 500;
L2 = 300;
L3 = 100;
L4 = 50;
%theta conversion from degree to radians
Theta1 = Theta1*pi/180;
Theta2 = Theta2*pi/180;
Theta3 = Theta3*pi/180;

%finding line vertices 
Pa = [0,0];
Pb = [L1*cos(Theta1),L1*sin(Theta1)];
Pc = [L1*cos(Theta1)+L2*cos(Theta1+Theta2),L1*sin(Theta1)+L2*sin(Theta1+Theta2)];
Pd = [L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3),L1*sin(Theta1)+L2*sin(Theta1+Theta2)+L3*sin(Theta1+Theta2+Theta3)];
Pe = [L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3)-L4*0.5*sin(Theta1+Theta2+Theta3),L1*sin(Theta1)+L2*sin(Theta1+Theta2)+L3*sin(Theta1+Theta2+Theta3)+L4*0.5*cos(Theta1+Theta2+Theta3)];
Pf = [L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3)+L4*0.5*sin(Theta1+Theta2+Theta3),L1*sin(Theta1)+L2*sin(Theta1+Theta2)+L3*sin(Theta1+Theta2+Theta3)-L4*0.5*cos(Theta1+Theta2+Theta3)];

%plotting line
plot([Pa(1),Pb(1)],[Pa(2),Pb(2)],'k');
hold on;
plot([Pc(1),Pb(1)],[Pc(2),Pb(2)],'k');
hold on;
plot([Pc(1),Pd(1)],[Pc(2),Pd(2)],'k');
hold on;
pl_l1 = plot([Pe(1),Pf(1)],[Pe(2),Pf(2)],'k');
end