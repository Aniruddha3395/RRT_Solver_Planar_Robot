function [ptsnew,thetanew] = Interpolator(thetai,thetaf,C,R)
L1 = 500;
L2 = 300;
L3 = 100;
L4 = 50;

Theta1i = thetai(1);
Theta2i = thetai(2);
Theta3i = thetai(3);
Pai = [0,0];
Pbi = [L1*cos(Theta1i),L1*sin(Theta1i)];
Pci = [L1*cos(Theta1i)+L2*cos(Theta1i+Theta2i),L1*sin(Theta1i)+L2*sin(Theta1i+Theta2i)];
Pdi = [L1*cos(Theta1i)+L2*cos(Theta1i+Theta2i)+L3*cos(Theta1i+Theta2i+Theta3i),L1*sin(Theta1i)+L2*sin(Theta1i+Theta2i)+L3*sin(Theta1i+Theta2i+Theta3i)];
Pei = [L1*cos(Theta1i)+L2*cos(Theta1i+Theta2i)+L3*cos(Theta1i+Theta2i+Theta3i)-L4*0.5*sin(Theta1i+Theta2i+Theta3i),L1*sin(Theta1i)+L2*sin(Theta1i+Theta2i)+L3*sin(Theta1i+Theta2i+Theta3i)+L4*0.5*cos(Theta1i+Theta2i+Theta3i)];
Pfi = [L1*cos(Theta1i)+L2*cos(Theta1i+Theta2i)+L3*cos(Theta1i+Theta2i+Theta3i)+L4*0.5*sin(Theta1i+Theta2i+Theta3i),L1*sin(Theta1i)+L2*sin(Theta1i+Theta2i)+L3*sin(Theta1i+Theta2i+Theta3i)-L4*0.5*cos(Theta1i+Theta2i+Theta3i)];

ptsnew = [Pai;Pbi;Pci;Pdi;Pei;Pfi];
thetanew = [thetai];
dTheta = [thetaf-thetai]/10;

%interpolation
for i = 1:10

Theta1 = (Theta1i+i*dTheta(1));
Theta2 = (Theta2i+i*dTheta(2));
Theta3 = (Theta3i+i*dTheta(3));

Theta= [Theta1,Theta2,Theta3];
Pa = [0,0];
Pb = [L1*cos(Theta(1)),L1*sin(Theta(1))];
Pc = [L1*cos(Theta(1))+L2*cos(Theta(1)+Theta(2)),L1*sin(Theta(1))+L2*sin(Theta(1)+Theta(2))];
Pd = [L1*cos(Theta(1))+L2*cos(Theta(1)+Theta(2))+L3*cos(Theta(1)+Theta(2)+Theta(3)),L1*sin(Theta(1))+L2*sin(Theta(1)+Theta(2))+L3*sin(Theta(1)+Theta(2)+Theta(3))];
Pe = [L1*cos(Theta(1))+L2*cos(Theta(1)+Theta(2))+L3*cos(Theta(1)+Theta(2)+Theta(3))-L4*0.5*sin(Theta(1)+Theta(2)+Theta(3)),L1*sin(Theta(1))+L2*sin(Theta(1)+Theta(2))+L3*sin(Theta(1)+Theta(2)+Theta(3))+L4*0.5*cos(Theta(1)+Theta(2)+Theta(3))];
Pf = [L1*cos(Theta(1))+L2*cos(Theta(1)+Theta(2))+L3*cos(Theta(1)+Theta(2)+Theta(3))+L4*0.5*sin(Theta(1)+Theta(2)+Theta(3)),L1*sin(Theta(1))+L2*sin(Theta(1)+Theta2())+L3*sin(Theta1+Theta2+Theta3)-L4*0.5*cos(Theta1+Theta2+Theta3)];

chkval = Detect_Collison(C,R,Pa,Pb);
if chkval == 1
    break;
else 
   chkval = Detect_Collison(C,R,Pb,Pc);
   if chkval==1
        break;
   else 
        chkval = Detect_Collison(C,R,Pc,Pd);
        if chkval==1
            break;
        else 
            chkval = Detect_Collison(C,R,Pe,Pf);
            if chkval==1
                break;
            end
        end
   end
end
ptsnew = [Pa;Pb;Pc;Pd;Pe;Pf];
thetanew = [Theta(1),Theta(2),Theta(3)];
end
end






   



    

