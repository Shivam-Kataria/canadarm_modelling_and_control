function [V0, F, C, numLinks] = loadRobotData(filepath)
    data   = load(filepath);
    fields = fieldnames(data);

    linkFields = {};
    for i = 1:length(fields)
        if startsWith(fields{i},'s') && isstruct(data.(fields{i}))
            linkFields{end+1} = fields{i};
        end
    end
    linkFields = sort(linkFields);
    numLinks   = min(length(linkFields), 7);  % changed 6 to 7

    V0 = cell(1, numLinks);
    F  = cell(1, numLinks);
    C  = cell(1, numLinks);

    for i = 1:numLinks
        s  = data.(linkFields{i});
        sf = fieldnames(s);

        vf = sf(startsWith(sf,'V')); V0{i} = s.(vf{1});
        ff = sf(startsWith(sf,'F')); F{i}  = s.(ff{1});
        cf = sf(startsWith(sf,'C')); C{i}  = s.(cf{1});

        if size(V0{i}, 2) == 3
            V0{i} = [V0{i}, ones(size(V0{i},1), 1)];
        end

        % Convert from decimeters to meters
        V0{i}(:, 1:3) = V0{i}(:, 1:3) / 10;
    end
end