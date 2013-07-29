%% Como leer los stl en el sistema de la base y pasarlos a su propio 
% sistema de referencia
clc;clear;close
disp('NECESARIA LA CLASE VR2SchunkRobot en el path')
disp('OJO con el mex en el constructor')

name='SchunkLink7Low';
link=7;

%% Primero crear el robot
schunk=VR2SchunkRobot('stl');
close

%% Leyendo link1
fv = stlread([name '.stl']);
%Transformacion de los vertices
tr=schunk.dKineN(zeros(7,1),link);

V=fv.vertices;
V=[V ones(length(V),1)]';
V=(tr^-1*V)';
fv.vertices=V(:,1:3);

%% Guardando el stl
stlwrite([name 'transform.stl'],fv) 



patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);