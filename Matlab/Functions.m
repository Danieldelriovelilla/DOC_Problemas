% Cosine matrix
function [C] = C_from_to(from, to)
C = zeros(3);
for i = 1:size(from, 1)
    for j = 1:size(to, 1)
    C(i,j) = dot(from(:,i),to(:,j));
    end
end
disp("C = ")
disp(C)
end

function [C] = C1(theta)
C = [1, 0, 0
    0, cos(theta), sin(theta);
    0, -sin(theta), cos(theta)];
end

function [C] = C2(theta)
C = [cos(theta), 0, -sin(theta);
    0, 1, 0
    sin(theta), 0, cos(theta)];
end

function [C] = C3(theta)
C = [cos(theta), sin(theta), 0;
    -sin(theta), cos(theta), 0;
    0, 0, 1];
end

function [C] = C323(t1, t2, t3)
C = C3(t1)*C2(t2)*C3(t3);
end

function [C] = C123(t1, t2, t3)
C = C1(t1)*C2(t2)*C3(t3);
end

function [theta1] = theta1_321(C)
theta1 = atan2(C(2,3),C(3,3));
end

function [theta2] = theta2_321(C)
theta2 = -asin(C(1,3));
end

function [theta3] = theta3_321(C)
theta3 = atan2(C(1,2),C(1,1));
end

function [C] = C_from_e(phi, e)
ex = [0, -e(3), e(2);
      e(3), 0, -e(1);
      -e(2), e(1), 0];
C = cos(phi)*eye(3) + (1-cos(phi))*e'*e - sin(phi)*ex;
end


% Eigenaxis
function [phi, e] = Eigenaxis(C21)
phi = acos( ( C21(1,1) + C21(2,2) + C21(3,3) - 1)/2 );
e = [C21(2,3) - C21(3,2);
     C21(3,1) - C21(1,3);
     C21(1,2) - C21(2,1)]/(2*sin(phi));
end

% Quaternions
function [q] = Quaternions_from_C(C)
phi = acos( ( C(1,1) + C(2,2) + C(3,3) - 1)/2 );
e = [C(2,3) - C(3,2);
     C(3,1) - C(1,3);
     C(1,2) - C(2,1)]/(2*sin(phi));

q = [e(1)*sin(phi/2);
     e(2)*sin(phi/2);
     e(3)*sin(phi/2)];
q(4) = cos(phi/2);
end

function [q] = Quaternions_from_e(phi, e)
q = [e(1)*sin(phi/2);
     e(2)*sin(phi/2);
     e(3)*sin(phi/2)];
q(4) = cos(phi/2);
end


% Attitude determination
function [Cb, Ci, Cbi] = Triad_Method(ub, vb, ui, vi)
% Body
t1b = ub;
t2b = cross(ub, vb)/norm(cross(ub, vb));
t3b = cross(t1b, t2b);
% Inertial
t1i = ui;
t2i = cross(ui, vi)/norm(cross(ui, vi));
t3i = cross(t1i, t2i);
% Matrix
Cb = [t1b, t2b, t3b];
Ci = [t1i, t2i, t3i];
Cbi = Cb*Ci';
end