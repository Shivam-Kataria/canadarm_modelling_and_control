function LinkPatches = drawRobot(ax, V0, F, numLinks)
    cla(ax); hold(ax,'on');
    axis(ax,'equal'); grid(ax,'on');
    view(ax,35,25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    ax.Color     = [0.10 0.10 0.13];
    ax.GridColor = [0.30 0.30 0.30];
    ax.XColor    = [0.70 0.70 0.70];
    ax.YColor    = [0.70 0.70 0.70];
    ax.ZColor    = [0.70 0.70 0.70];
    title(ax,'Canadarm Simulator','Color',[0.9 0.9 0.9],'FontSize',11,'FontWeight','bold');

    colors = [0.45 0.45 0.50;
              0.20 0.45 0.72;
              0.20 0.45 0.72;
              0.85 0.33 0.10;
              0.85 0.33 0.10;
              0.15 0.68 0.38;
              0.80 0.20 0.20];

    LinkPatches = cell(1, numLinks);
    for i = 1:numLinks
        LinkPatches{i} = patch(ax, ...
            'Faces',    F{i}, ...
            'Vertices', V0{i}(:,1:3), ...
            'FaceColor', colors(i,:), ...
            'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', ...
            'AmbientStrength', 0.4, ...
            'DiffuseStrength', 0.7);
    end
    light(ax,'Position',[1 1 2],'Style','infinite');
    light(ax,'Position',[-1 -1 1],'Style','infinite');
end