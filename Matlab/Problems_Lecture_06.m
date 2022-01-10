clc
clear
close all

%% EJERCICIO 1
% Datos
a = [1/2, sqrt(3)/2, 0;
     0, 0, 1;
     sqrt(3)/2, -1/2, 0]';
b = [0, 1, 0;
     1, 0, 0;
     0, 0, -1]';
in = [1, 0, 0;
      0, 1, 0;
      0, 0, 1];

% a)
for i = 1:size(a, 1)
    for j = 1:size(a, 2)
    prod = dot(a(:,i),a(:,j));
    disp(strcat("a_", num2str(i), ".a_", num2str(j), " = ",num2str(prod)))
    end
end

% b)
Cab = C_from_to(a, b);

% c) Find the directional cosine matrix Cai that describes the orientation of frame A relative to frame I.
Cai = C_from_to(a, in);

% d) Find the directional cosine matrix Cbi that describes the orientation of frame B relative to frame I.
Cbi = C_from_to(b, in);

% e) Check if  holds
disp("Si es igual a 0, lo cumple")
disp(Cab - Cai*Cbi')

% Check if , where 1 is 3x3 unit matrix.
disp("Si es identidad, lo cumple")
disp(Cab*Cab')

% g) For given arbitrary matrix  and matrix  check if they do not commute (AB ≠ BA).
A = Cai;
B = Cbi';
disp("Si es igual a 0, lo cumple")
disp(A*B - B*A)

% h) Is the following matrix a rotation matrix?
C = [1/2, 0, 0;
    0, 1, 0;
    0, 0, 2];

disp("Si es identidad, lo cumple")
disp(C*C')


%% EJERCICIO 3

% a) Find the Euler rotation matrix C21 in terms of 3-2-3 Euler angles rotation sequence, with angles ϴ1, ϴ2 and ϴ3.
% Specifically, frame 2 is obtained from frame 1 by:
% - A rotation ϴ1 about the z-axis (3-axis) of frame 1,
% - a rotation ϴ2 about the y-axis (2-axis) of intermediate frame,
% - a rotation ϴ3 about the z-axis (3-axis) of the transformed frame.
syms t1(t) t2(t) t3(t)
C21 = C3(t3)*C2(t2)*C3(t1);
dC21 = diff(C21, t);

% b)
syms syms t1 t2 t3 dt1 dt2 dt3 w1 w2 w3

% Descomposicion de velocidades angulares
w21 = dt3*[0, 0, 1]' +...
    dt2*C3(t3)*[0, 1, 0]' +...
    dt1*C3(t3)*C2(t2)*[0, 0, 1]';
eqn = w21 == [w1, w2, w3]';

% Despejar como variable independiente la derivada de los angulos
vars = [dt1; dt2; dt3];
[A,b] = equationsToMatrix(eqn,vars);
disp("dteta = omega/A, siendo A:")
disp(A)

% d) What are the points of singularity for this Euler rotation?
disp("Determinante = 0")
condicion = det(A);
disp(condicion)
disp("Curva donde se hace 0")
fimplicit(condicion)


%% EJERCICIO 4

% The orientation of an object is given in terms of the 3-2-1 Euler angles (-15, 25, 10).
% a) Write the direction cosine Euler rotation matrix C21.
t1 = deg2rad(-15);
t2 = deg2rad(25);
t3 = deg2rad(10);
disp("Cosine matrix")
C21 =C1(t1)*C2(t2)*C3(t3);

% b) Find the principle Euler eigenaxis rotation angle φ.
[phi, e] = Eigenaxis(C21);
disp(strcat("Phi =", num2str(phi)))
disp("e ="); disp(e);

% c) Find the corresponding principal Euler rotation eigenaxis e.
% Verify that C21e =e.
disp("Si es igual a 0, lo cumple")
disp(e-C21*e)

% d) Find the corresponding Euler parameters = Quaternions
disp("Quaternion")
q = Quaternions_from_C(C21)

% e) Is the last expression an unit quaternion? Has it magnitude one?
disp(strcat("Magnitud del quaternion: ", num2str(norm(q))))


%% EJERCICIO 5
C10 = C123(deg2rad(60), deg2rad(-45), deg2rad(30));
C20 = C123(deg2rad(-15), deg2rad(25), deg2rad(10));

C12 = C10*C20'

thata1 = rad2deg(theta1_321(C12))
theta2 = rad2deg(theta2_321(C12))
theta3 = rad2deg(theta3_321(C12))


%% EJERCICIO 6
phi = deg2rad(45);
e = [1, 1, 1]/sqrt(3);
[C] = C_from_e(phi, e)
thata1 = rad2deg(theta1_321(C))
theta2 = rad2deg(theta2_321(C))
theta3 = rad2deg(theta3_321(C))


%% FUNCTIONS

function dot_theta = kinematics_321(t,y)
w = [sin(0.1*t), 0, cos(0.1*t)]'*deg2rad(5);
dot_theta = [1, (sin(y(1))*sin(y(2)))/(cos(y(2))*cos(y(1))^2 + cos(y(2))*sin(y(1))^2), (cos(y(1))*sin(y(2)))/(cos(y(2))*cos(y(1))^2 + cos(y(2))*sin(y(1))^2);
        0,                                    cos(y(1))/(cos(y(1))^2 + sin(y(1))^2),                                   -sin(y(1))/(cos(y(1))^2 + sin(y(1))^2);
        0,              sin(y(1))/(cos(y(2))*cos(y(1))^2 + cos(y(2))*sin(y(1))^2),              cos(y(1))/(cos(y(2))*cos(y(1))^2 + cos(y(2))*sin(y(1))^2)]*...
        w;
end

function dot_q = kinematics_q(t,q)
w = [sin(0.1*t); 0.01; cos(0.1*t); 0.0]*deg2rad(50);
w1 = w(1);
w2 = w(2);
w3 = w(3);
w4 = w(4);
dot_q = 0.5 * [ 0 w3 -w2 w1 
 -w3 0 w1 w2 
 w2 -w1 0 w3 
 -w1 -w2 -w3 0 ]*q;
end


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