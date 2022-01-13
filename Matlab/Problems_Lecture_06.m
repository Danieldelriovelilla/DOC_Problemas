clc
clear
close all


%%

doc = doc_functions();


%% EJERCICIO 1
disp("EJERCICIO 1")
% Rotar cosas

a = [1/2, sqrt(3)/2, 0;
     0, 0, 1;
     sqrt(3)/2, -1/2, 0]';
b = [0, 1, 0;
     1, 0, 0;
     0, 0, -1]';
in = [1, 0, 0;
      0, 1, 0;
      0, 0, 1];

% a) Comporobar que a y b son ortonormales
for i = 1:size(a, 1)
    for j = 1:size(a, 2)
    prod = dot(a(:,i),a(:,j));
    disp(strcat("a_", num2str(i), ".a_", num2str(j), " = ",num2str(prod)))
    end
end

% b)
Cab = doc.C_from_to(a, b);

% c) Find the directional cosine matrix Cai that describes the orientation of frame A relative to frame I.
Cai = doc.C_from_to(a, in);

% d) Find the directional cosine matrix Cbi that describes the orientation of frame B relative to frame I.
Cbi = doc.C_from_to(b, in);

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

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 3
disp("EJERCICIO 3")
% Cosas con rotaciones

% a) Find the Euler rotation matrix C21 in terms of 3-2-3 Euler angles rotation sequence, with angles ϴ1, ϴ2 and ϴ3.
% Specifically, frame 2 is obtained from frame 1 by:
% - A rotation ϴ1 about the z-axis (3-axis) of frame 1,
% - a rotation ϴ2 about the y-axis (2-axis) of intermediate frame,
% - a rotation ϴ3 about the z-axis (3-axis) of the transformed frame.
syms t1(t) t2(t) t3(t)
C21 = doc.C3(t3)*doc.C2(t2)*doc.C3(t1);
dC21 = diff(C21, t);

% b)
syms syms t1 t2 t3 dt1 dt2 dt3 w1 w2 w3

% Descomposicion de velocidades angulares
w21 = dt3*[0, 0, 1]' +...
    dt2*doc.C3(t3)*[0, 1, 0]' +...
    dt1*doc.C3(t3)*doc.C2(t2)*[0, 0, 1]';
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

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 4
disp("EJERCICIO 4")
% 

% The orientation of an object is given in terms of the 3-2-1 Euler angles (-15, 25, 10).
% a) Write the direction cosine Euler rotation matrix C21.
t1 = deg2rad(-15);
t2 = deg2rad(25);
t3 = deg2rad(10);
disp("Cosine matrix")
C21 =doc.C1(t1)*doc.C2(t2)*doc.C3(t3);

% b) Find the principle Euler eigenaxis rotation angle φ.
[phi, e] = doc.Eigenaxis(C21);
disp(strcat("Phi =", num2str(phi)))
disp("e ="); disp(e);

% c) Find the corresponding principal Euler rotation eigenaxis e.
% Verify that C21e =e.
disp("Si es igual a 0, lo cumple")
disp(e-C21*e)

% d) Find the corresponding Euler parameters = Quaternions
disp("Quaternion")
q = doc.Quaternions_from_C(C21)

% e) Is the last expression an unit quaternion? Has it magnitude one?
disp(strcat("Magnitud del quaternion: ", num2str(norm(q))))

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 5
disp("EJERCICIO 5")
% Angulos entre dos sistemas de referencia

C10 = doc.C123(deg2rad(60), deg2rad(-45), deg2rad(30));
C20 = doc.C123(deg2rad(-15), deg2rad(25), deg2rad(10));

C12 = C10*C20'

thata1 = rad2deg(doc.theta1_321(C12))
theta2 = rad2deg(doc.theta2_321(C12))
theta3 = rad2deg(doc.theta3_321(C12))

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 6
disp("EJERCICIO 6")
% Eigenaxis a C321

phi = deg2rad(45);
e = [1, 1, 1]/sqrt(3);
[C] = doc.C_from_e(phi, e)
thata1 = rad2deg(doc.theta1_321(C))
theta2 = rad2deg(doc.theta2_321(C))
theta3 = rad2deg(doc.theta3_321(C))