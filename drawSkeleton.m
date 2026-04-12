function drawSkeleton(ax, theta)
    cla(ax);
    hold(ax,'on');
    axis(ax,'equal'); grid(ax,'on');
    view(ax,35,25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');

    [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta);
    Tlist = {eye(4),T01,T02,T03,T04,T05,T06};

    origins = zeros(7,3);
    for i = 1:7
        origins(i,:) = Tlist{i}(1:3,4)';
    end

    plot3(ax, origins(:,1),origins(:,2),origins(:,3), ...
        'o-','Color',[0.20 0.65 1.0],'LineWidth',3, ...
        'MarkerFaceColor',[1 0.5 0.1],'MarkerSize',8);

    sc = 0.5;
    quiver3(ax,0,0,0,sc,0,0,'r','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(ax,0,0,0,0,sc,0,'g','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(ax,0,0,0,0,0,sc,'b','LineWidth',2,'MaxHeadSize',0.5);
    text(ax,sc,0,0,'X','Color','r','FontWeight','bold');
    text(ax,0,sc,0,'Y','Color','g','FontWeight','bold');
    text(ax,0,0,sc,'Z','Color','b','FontWeight','bold');
end