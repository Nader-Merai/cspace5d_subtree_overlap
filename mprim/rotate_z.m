% rotate matrix. Rotating around z-axis by 'angle'


function[Rz] = rotate_z(angle)


Rz = [ cos(angle) sin(angle)  0  0;
      -sin(angle) cos(angle)  0  0;
         0           0        1  0;
         0           0        0  1];


