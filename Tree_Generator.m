function [Tree,Parent,pt_store] = Tree_Generator(growth_max,N_samples,Theta1I,Theta2I,Theta3I,Theta1F,Theta2F,Theta3F,Center,Radius)

L1 = 500;
L2 = 300;
L3 = 100;
L4 = 50;
stop_code = 0;
Theta1F = Theta1F*pi/180;
Theta2F = Theta2F*pi/180;
Theta3F = Theta3F*pi/180;
PdF = [L1*cos(Theta1F)+L2*cos(Theta1F+Theta2F)+L3*cos(Theta1F+Theta2F+Theta3F),L1*sin(Theta1F)+L2*sin(Theta1F+Theta2F)+L3*sin(Theta1F+Theta2F+Theta3F)];

% getting initial configuration
Theta1I = Theta1I*pi/180;
Theta2I = Theta2I*pi/180;
Theta3I = Theta3I*pi/180;
PaI = [0,0];
PbI = [L1*cos(Theta1I),L1*sin(Theta1I)];
PcI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)];
PdI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)];
PeI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I)-L4*0.5*sin(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)+L4*0.5*cos(Theta1I+Theta2I+Theta3I)];
PfI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I)+L4*0.5*sin(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)-L4*0.5*cos(Theta1I+Theta2I+Theta3I)];
pt_store = [];
pt_store = [pt_store;PaI,PbI,PcI,PdI,PeI,PfI];

scatter(PdI(1),PdI(2),'.','r');
hold on;
scatter(PdF(1),PdF(2),'.','r');

Tree = [];
Tree = [Tree;PdI];
Parent = [];
Parent = [Parent;PdI];
theta_store = [];
theta_store = [theta_store;Theta1I,Theta2I,Theta3I];
pt_store = [];
pt_store = [pt_store;PaI,PbI,PcI,PdI,PeI,PfI];
for i = 1:N_samples
    %toss between random sample and destination point
    luck = randi([0,100],1);
    if luck ==100
        Theta_rand = [Theta1F,Theta2F,Theta3F];
    else
        Theta_rand = [(rand(1)*pi),(rand(1)*pi-pi/2),(rand(1)*pi-pi/2)];
    end
%     if (Theta_rand(1)- theta_store(size(theta_store,1),1)<(pi/10)) && (Theta_rand(2)- theta_store(size(theta_store,1),2)<(pi/10)) && (Theta_rand(3)- theta_store(size(theta_store,1),3)<(pi/10))
    Pa = [0,0];
    Pb = [L1*cos(Theta_rand(1)),L1*sin(Theta_rand(1))];
    Pc = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))];
    Pd = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    Pe = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))-L4*0.5*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))+L4*0.5*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    Pf = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))+L4*0.5*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))-L4*0.5*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    
    % finding near neighbour
    
    d1 = growth_max;
    if Pb(2)>0 && Pc(2)>0 && Pd(2)>0 && Pe(2)>0 && Pf(2)>0
        for j = 1:size(Tree,1)
            d = sqrt(((Pd(1)-Tree(j,1))^2)+((Pd(2)-Tree(j,2))^2));
            if d<=d1
                d1 = d;
                jnew = j;
            end
        end
        
        % checking collison detection
        if d1<growth_max
            for k = 1:size(Center,1)
                Collison_val1 = Detect_Collison(Center(k,:),Radius(k),Pa,Pb);
                Collison_val2 = Detect_Collison(Center(k,:),Radius(k),Pb,Pc);
                Collison_val3 = Detect_Collison(Center(k,:),Radius(k),Pc,Pd);
                Collison_val4 = Detect_Collison(Center(k,:),Radius(k),Pe,Pf);
                if Collison_val1 == 1 || Collison_val2 == 1 || Collison_val3 == 1 || Collison_val4 == 1
                    [ptsnew,thetanew] = Interpolator(theta_store(jnew,:),Theta_rand,Center(k,:),Radius(k));
                    Pa = ptsnew(1,:);Pb = ptsnew(2,:);Pc = ptsnew(3,:);Pd = ptsnew(4,:);Pe = ptsnew(5,:);Pf = ptsnew(6,:);
                    Theta_rand = [thetanew(1),thetanew(2),thetanew(3)];
                end
            end
            
            if abs(Tree(jnew,:)- Pd) <=growth_max
            Tree = [Tree;Pd];
            Parent = [Parent;Tree(jnew,:)];
            pt_store = [pt_store;Pa,Pb,Pc,Pd,Pe,Pf];
            theta_store = [theta_store;Theta_rand(1),Theta_rand(2),Theta_rand(3)];
            end
            if Pd ==PdF
                stop_code = 1;
            end
            hold on;
            plot([Tree(size(Tree,1),1),Parent(size(Parent,1),1)],[Tree(size(Tree,1),2),Parent(size(Parent,1),2)],'b');
            pause(0.0000001);
            hold off;
        end
        
    end
    
    
    
    if stop_code==1
        break;
    end
    
    end
end
% end

