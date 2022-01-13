clc
clear
close all

%%

doc = doc_functions();


%% EJERCICIO 1
disp("EJERCICIO 1")

% Estabilidad cilindro
syms m h r

I = doc.Inertia_Cylinder(m, r, h);
stab = I(1,1) == I(3,3);
eq = isolate(stab, h);
pretty( eq )

disp(" "); disp("%   ---   ---   %"); disp(" ")


%% EJERCICIO 2
disp("EJERCICIO 2")

% Gravity gradient torque
Ro = 7000;
mu = 3.986e5;

I = doc.Inertia_Matrix(100, 120, 80);
C = doc.C123(pi/4, pi/4, pi/4);
RoG = [0, 0, Ro]';

% R en ejes cuerpo
Rob = C*RoG;

Tgg = doc.Gravitational_Torque(mu, Rob, I)


disp(" "); disp("%   ---   ---   %"); disp(" ")


%% EJERCICIO 3
disp("EJERCICIO 3")

Is = [8, 12, 10]';
w = [0, 0, 0.1]';

spin_ax = 3;
Ispin = Is(spin_ax);
Is(spin_ax) = [];
ws = w(spin_ax);

cond1 = [...
    (Is(2) - Ispin)*ws;
    (Is(1) - Ispin)*ws;
    ];
disp("Condicion 1")
h_cond1 = max(cond1)
cond2 = [...
    (Is(2) - Ispin)*ws;
    (Is(1) - Ispin)*ws;
    ];
disp("Condicion 2")
h_cond2 = min(cond2)

disp(" "); disp("%   ---   ---   %"); disp(" ")


%% EJERCICIO 4
disp("EJERCICIO 4")

% Variables independientes
Is = sym('Is',[3 1]);
I = doc.Inertia_Matrix(Is(1), Is(2), Is(3));
th = sym('th',[3 1]);
dth = sym('dth',[3 1]);
hs = sym('hs');
dw = sym('dw',[3 1]);
% Eje de rotacion wheel
a = [0, 0, 1]';


% a.
T = [0, 0, 0]';
w = [0, 0, 0]';

[eq] = doc.Euler_Equation_Stabilized(I, w, dw, T, hs, a);

% b.
% Calcular dw con la rueda
w = sym('w',[3 1]);
[eq] = expand(doc.Euler_Equation_Stabilized(I, w, dw, T, hs, a));

% Imponer que w es pequeño, por lo tanto w1*w2 es 0
eq = subs(eq, {w(2)*w(3), w(1)*w(3), w(1)*w(2)}, {0, 0, 0});

% Sustituir w y dw por variables dependientes del tiempo
syms w1(t) w2(t) w3(t)
dw1 = diff(w1, t);
dw2 = diff(w2, t);
dw3 = diff(w3, t);
eq = subs(eq,...
    {w(1) w(2) w(3) dw(1) dw(2) dw(3)}, ...
    {w1, w2, w3, dw1, dw2, dw3});

% Integrar las ecuaciones para obtener la frecuencia de oscilacion
disp("La fecuencia es el parámetro que multiplica a t dentro de la exponencial ")
[w1, w2, w3] = dsolve(eq);
disp("w1"); pretty(w1)
disp("w2"); pretty(w2)
disp("w3"); pretty(w3)
disp("HAY QUE QUITAR EL i DENTRO DE LA EXPONENCIAL")

disp(" "); disp("%   ---   ---   %"); disp(" ")




