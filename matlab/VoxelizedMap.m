% simple file for visualizing the voxelized mesh map
function y = VoxelizedMap(filename)

fv = stlread(filename);

%p = patch('faces',F,'vertices',V,'FaceColor',[0.8 0.8 1.0],'FaceLighting','gouraud');
a = patch(fv, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.16);

camlight('right');
camlight('headlight');
material('dull');

grid on;
