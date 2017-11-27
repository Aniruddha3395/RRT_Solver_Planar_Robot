
clc;
clear;
close all;

% Center = [0,700;575,575;-600,600;800,350];
% Radius = [50;40;65;10];

Center=[-200,700;200,620;-200,400;300,500];
Radius=[100;50;100;50];



growth_max = 100;
for i = 1:size(Center,1)
    hold on;
    plotcirc = Obstacle(Center(i,:),Radius(i,:));
    hold off;
end
Theta1I = 80;
Theta2I = 0;
Theta3I = 0;

Theta1F = 100;
Theta2F = 50;
Theta3F = 0;
hold on;
Arm_Plotter(Theta1I,Theta2I,Theta3I);
hold on;
Arm_Plotter(Theta1F,Theta2F,Theta3F);


hold on;
L1 = 500;
L2 = 300;
L3 = 100;
L4 = 50;

Theta1F = Theta1F*pi/180;
Theta2F = Theta2F*pi/180;
Theta3F = Theta3F*pi/180;

PdF = [L1*cos(Theta1F)+L2*cos(Theta1F+Theta2F)+L3*cos(Theta1F+Theta2F+Theta3F),L1*sin(Theta1F)+L2*sin(Theta1F+Theta2F)+L3*sin(Theta1F+Theta2F+Theta3F)];
new_flag = 0;
%tree generator
Theta1I = Theta1I*pi/180;
Theta2I = Theta2I*pi/180;
Theta3I = Theta3I*pi/180;
ThetaI= [Theta1I,Theta2I,Theta3I];
PaI = [0,0];
PbI = [L1*cos(Theta1I),L1*sin(Theta1I)];
PcI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)];
PdI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)];
PeI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I)-L4*0.5*sin(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)+L4*0.5*cos(Theta1I+Theta2I+Theta3I)];
PfI = [L1*cos(Theta1I)+L2*cos(Theta1I+Theta2I)+L3*cos(Theta1I+Theta2I+Theta3I)+L4*0.5*sin(Theta1I+Theta2I+Theta3I),L1*sin(Theta1I)+L2*sin(Theta1I+Theta2I)+L3*sin(Theta1I+Theta2I+Theta3I)-L4*0.5*cos(Theta1I+Theta2I+Theta3I)];
scatter(PdI(1),PdI(2),'.','r');

N_samples = 1e7;
pt_store = [];
pt_store = [pt_store;PaI,PbI,PcI,PdI,PeI,PfI];
Tree = [];
Tree = [Tree;PdI];
Parent = [];
Parent = [Parent;PdI];
theta_store = [];
theta_store = [theta_store;Theta1I,Theta2I,Theta3I];
stop_code = 0;
for i = 1:N_samples
    %toss between random sample and destination point
    luck = randi([0,100],1);
    if luck ==100
        Theta_rand = [Theta1F,Theta2F,Theta3F];
    else
%         Theta_rand = [theta_store(1)+abs(rand(1)*30*pi/180),theta_store(2)+(rand(1)*30*pi/180)-15*pi/180,theta_store(3)+(rand(1)*30*pi/180)-15*pi/180];
%         while Theta_rand(1)<0 || Theta_rand(1)>pi || Theta_rand(2)<-pi/2 || Theta_rand(2)>pi/2 || Theta_rand(3)<-pi/2 || Theta_rand(3)>pi/2
%             %
%             Theta_rand = [theta_store(1)+abs(rand(1)*30*pi/180),theta_store(2)+(rand(1)*30*pi/180)-15*pi/180,theta_store(3)+(rand(1)*30*pi/180)-15*pi/180];
                    Theta_rand = [(rand(1)*pi),(rand(1)*pi-pi/2),(rand(1)*pi-pi/2)];
            %         Theta_rand = [(randi([0,180])*pi/180),(randi([0,180])*pi/180)-(pi/180),(randi([0,180])*pi/180)-(pi/180)];
            
            %         Theta_rand = [theta_store(1)+abs(rand(1)*10*pi/180),theta_store(2)+(rand(1)*10*pi/180)-5*pi/180,theta_store(3)+(rand(1)*10*pi/180)-5*pi/180];
            
            %             if Theta_rand(1)<=0 || Theta_rand(1)>=pi || Theta_rand(2)<=-pi/2 || Theta_rand(2)>=pi/2 || Theta_rand(2)<=-pi/2 || Theta_rand(2)>=pi/2
            %             continue;
%         end
    end
    
    Pa = [0,0];
    Pb = [L1*cos(Theta_rand(1)),L1*sin(Theta_rand(1))];
    Pc = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))];
    Pd = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    Pe = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))-L4*0.5*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))+L4*0.5*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    Pf = [L1*cos(Theta_rand(1))+L2*cos(Theta_rand(1)+Theta_rand(2))+L3*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))+L4*0.5*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3)),L1*sin(Theta_rand(1))+L2*sin(Theta_rand(1)+Theta_rand(2))+L3*sin(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))-L4*0.5*cos(Theta_rand(1)+Theta_rand(2)+Theta_rand(3))];
    
    if sqrt((Pd(2)^2))<1200
        
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
                
                for k = 1:size(Center,1)
                    [pt_temp,theta_temp] = Interpolator(theta_store(jnew,:),Theta_rand,Center(k,:),Radius(k));
                
                if theta_temp~=Theta_rand
                 new_flag = 1;
                    break;
                end
                end
%             
            if new_flag==1;
                new_flag=0;
                continue;
            end
            
            if abs(Tree(jnew,:)- Pd) <=growth_max
                Tree = [Tree;Pd];
                Parent = [Parent;Tree(jnew,:)];
                pt_store = [pt_store;Pa,Pb,Pc,Pd,Pe,Pf];
                theta_store = [theta_store;Theta_rand(1),Theta_rand(2),Theta_rand(3)];
                
            end
            if Pd ==PdF
                Child_Parent = [Tree, Parent];
                store = [];
                store = [store;Tree(size(Tree,1),:)];
                pts = [];
                thetaval = [];
                while store(size(store,1),:)~=Tree(1,:)
                    Parent_temp = store(size(store,1),:);
                    for i = 1:size(Tree,1)
                        if Parent_temp == Tree(i,:)
                            store = [store;Child_Parent(i,3:4)];
                            pts = [pts;pt_store(i,:)];
                            thetaval = [thetaval;theta_store(i,:)];
                            break;
                        end
                    end
                end
                
                for i = 1:size(thetaval,1)-1
                    if (abs(thetaval(i,1)-thetaval(i+1,1))>(pi/2))||(abs(thetaval(i,2)-thetaval(i+1,2)>(pi/2)))||(abs(thetaval(i,3)-thetaval(i+1,3))>(pi/2))
                        break;
                    else
                        stop_code = 1;
                    end
                end
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
hold on;
scatter(Pd(1),Pd(2),'.','r');

% backtracing path
Child_Parent = [Tree, Parent];
store = [];
store = [store;Tree(size(Tree,1),:)];
pts = [];
thetaval = [];
while store(size(store,1),:)~=Tree(1,:)
    Parent_temp = store(size(store,1),:);
    for i = 1:size(Tree,1)
        if Parent_temp == Tree(i,:)
            store = [store;Child_Parent(i,3:4)];
            pts = [pts;pt_store(i,:)];
            thetaval = [thetaval;theta_store(i,:)];
            break;
        end
    end
end

hold on;
plot(store(:,1),store(:,2),'g','LineWidth',2);
pts = [pts;pt_store(size(pt_store,1),:)];
pts = flipud(pts);
thetaval = [thetaval;theta_store(size(theta_store,1),:)];
thetaval = flipud(thetaval);
figure;
for i = 1:size(Center,1)
    hold on;
    plotcirc = Obstacle(Center(i,:),Radius(i,:));
    hold off;
end
r = [1:-(1/size(pts,1)):0];
b = [0:(1/size(pts,1)):1];
for i = 1:size(pts,1)
    hold on;
    plot([pts(i,1),pts(i,3)],[pts(i,2),pts(i,4)],'Color',[r(i) 0 b(i)]);
    hold on;
    plot([pts(i,3),pts(i,5)],[pts(i,4),pts(i,6)],'Color',[r(i) 0 b(i)]);
    hold on;
    plot([pts(i,5),pts(i,7)],[pts(i,6),pts(i,8)],'Color',[r(i) 0 b(i)]);
    hold on;
    plot([pts(i,9),pts(i,11)],[pts(i,10),pts(i,12)],'Color',[r(i) 0 b(i)]);
    pause(0.1);
     
%    colormap winter
end

