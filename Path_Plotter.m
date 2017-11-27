function [pl] = Path_Plotter(Tree,Parent,pt_store)


% path line

Child_Parent = [Tree, Parent];
store = [];
store = [store;Tree(size(Tree,1),:)];
pts = [];

while store(size(store,1),:)~=Tree(1,:)
    Parent_temp = store(size(store,1),:);
    for i = 1:size(Tree,1)
        if Parent_temp == Tree(i,:)
            store = [store;Child_Parent(i,3:4)];
            pts = [pts;pt_store(i,:)];
            break;
        end
    end
end
hold on;
plot(store(:,1),store(:,2),'g','LineWidth',4);

% robotic arm movement

pts = [pts;pt_store(size(pt_store,1),:)];
pts = flipud(pts);

for i = 1:size(pts,1)

hold on;    
plot([pts(i,1),pts(i,3)],[pts(i,2),pts(i,4)],'k','linewidth',1);
hold on;
plot([pts(i,3),pts(i,5)],[pts(i,4),pts(i,6)],'k');
hold on;
plot([pts(i,5),pts(i,7)],[pts(i,6),pts(i,8)],'k');
hold on;
plot([pts(i,9),pts(i,11)],[pts(i,10),pts(i,12)],'k');
pause(0.05);

end


end