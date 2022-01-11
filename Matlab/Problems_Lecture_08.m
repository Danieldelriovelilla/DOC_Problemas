clc
clear
close all

%% OBJETO CON TODAS LAS FUNCIONES

doc = doc_functions();



%%  EJERCICIO 1

J = [1500, 0, -1000;
    0, 2700, 0;
    -1000, 0, 3000];

[I1, C1] = doc.Principal_Inertia(J);


%% EJERCICIO 2

J = [20, -10,  -15;
     -10, 30, 0;
     -15, 0, 15];
 
[I2, C2] = doc.Principal_Inertia(J);




%% EJERCICIO 3



%% EJERCICIO 4



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