clc
clear
close all

%% OBJETO CON TODAS LAS FUNCIONES

doc = doc_functions();


%%  EJERCICIO 1
disp("EJERCICIO 1")
% Eigenvalues, eigenvectors and rotation matrix

J = [1500, 0, -1000;
    0, 2700, 0;
    -1000, 0, 3000];

[I1, C1] = doc.Principal_Inertia(J)

disp(" "); disp("%   ---   ---   %"); disp(" ")



%% EJERCICIO 2
disp("EJERCICIO 2")
% Resolver w(t) con condiciones iniciales de un cilindro

J = [20, -10,  -15;
     -10, 30, 0;
     -15, 0, 15];
 
[I2, C2] = doc.Principal_Inertia(J)


disp(" "); disp("%   ---   ---   %"); disp(" ")