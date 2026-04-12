function Robot_cad2mat
% 
% To get PUMA560 CAD data into Matlab, the process is:
%
% 1) Export the 3D CAD data as an ASCII STL (soliworks save as stl) file.
% 2) This Matlab routine reads the CAD data
% 3) Once read, the CAD data for each link is stored into a struct.
%

file1 = 'arm1 - Part 1.STL'; % link 1
file2 = 'arm1 - Part 2.STL'; % link 2
file3 = 'arm1 - Part 3.STL'; % link 3
file4 = 'arm1 - Part 4.STL'; % link 4
file5 = 'arm1 - Part 5.STL'; % link 5
file6 = 'arm1 - Part 6.STL'; % link 6
file7 = 'arm1 - Part 7.STL'; % link 7

%
% Read the CAD data file:
% F is face color, 
% V is the xyz coordinates of all vertices
% C is the face vertex color (color per vertex), 
% somehow this value is zero from Soliworks output

[F1, V1, C1] = rndread(file1);
[F2, V2, C2] = rndread(file2);
[F3, V3, C3] = rndread(file3);
[F4, V4, C4] = rndread(file4);
[F5, V5, C5] = rndread(file5);
[F6, V6, C6] = rndread(file6);
[F7, V7, C7] = rndread(file7);

% vertices data is nx3 matrix
% we make it nx4 for later usage with homogeneous transformation matrix
%V1 = V1';
%V1 = [V1(1,:); V1(2,:); V1(3,:); ones(1,length(V1))]';

[m1 n1]=size(V1);%m is the number row
%V1 = [V1(:, 1) V1(:,2) V1(:,3) ones(m1,1)];
V1 = [V1 ones(m1,1)];

V2 = V2';
V2 = [V2(1,:); V2(2,:); V2(3,:); ones(1,length(V2))]';

V3 = V3';
V3 = [V3(1,:); V3(2,:); V3(3,:); ones(1,length(V3))]';

V4 = V4';
V4 = [V4(1,:); V4(2,:); V4(3,:); ones(1,length(V4))]';

V5 = V5';
V5 = [V5(1,:); V5(2,:); V5(3,:); ones(1,length(V5))]';

V6 = V6';
V6 = [V6(1,:); V6(2,:); V6(3,:); ones(1,length(V6))]';

V7 = V7';
V7 = [V7(1,:); V7(2,:); V7(3,:); ones(1,length(V7))]';

clf;
%create the struct variable for each link
s1 = struct('V1', V1, 'F1', F1, 'C1', C1);
s2 = struct('V2', V2, 'F2', F2, 'C2', C2);
s3 = struct('V3', V3, 'F3', F3, 'C3', C3);
s4 = struct('V4', V4, 'F4', F4, 'C4', C4);
s5 = struct('V5', V5, 'F5', F5, 'C5', C5);
s6 = struct('V6', V6, 'F6', F6, 'C6', C6);
s7 = struct('V7', V7, 'F7', F7, 'C7', C7);

% save all the struct s1 to s7 into a mat file
save('RobotName_LinksData.mat', 's*');


function [fout, vout, cout] = rndread(filename)
% Reads CAD STL ASCII files, which most CAD programs can export.
% Used to create Matlab patches of CAD 3D data.
% Returns a vertex list and face list, for Matlab patch command.
% 
% filename = 'hook.stl';  % Example file.
%
fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
if fid == -1 
    error('File could not be opened, check name or path.')
end
%
% Render files take the form:
%   
%solid BLOCK
%  color 1.000 1.000 1.000
%  facet
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%    outer loop
%      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
%    endloop
% endfacet
%
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.
%
CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
%                                                %Some STLs have it, some don't.   
vnum=0;       %Vertex number counter.
report_num=0; %Report the status as we go.
VColor = 0;
%
while feof(fid) == 0                    % test for end of file, if not then do stuff
    tline = fgetl(fid);                 % reads a line of data from file.
    fword = sscanf(tline, '%s ');       % make the line a character string
% Check for color
    if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
       VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
    end                                % Keep this color, until the next color is used.
    if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
       vnum = vnum + 1;                % If a V we count the # of V's
       report_num = report_num + 1;    % Report a counter, so long files show status
       if report_num > 249;
           disp(sprintf('Reading vertix num: %d.',vnum));
           report_num = 0;
       end
       v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
       c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
    end                                 % we "*s" skip the name "color" and get the data.                                          
end
%   Build face list; The vertices are in order, so just number them.
%
fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
flist = 1:vnum;     %Face list of vertices, all in order.
F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
%
%   Return the faces and vertexs.
%
fout = F';  %Orients the array for direct use in patch.
vout = v';  % "
cout = c';
%
fclose(fid);