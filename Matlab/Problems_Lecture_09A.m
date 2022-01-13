clc
clear
close all

doc = doc_functions();

%%  EJERCICIO 1
disp("EJERCICIO 1")
% Resolver w(t) con condiciones iniciales de un cilindro

% Cilindro
syms m r l
l = 4*r;
I = doc.Inertia_Cylinder(m, r, l);

% Condiciones iniciales
syms W0
w0 = W0*[3/5, 0, 4/5]';

% Variables simbolicas SIN dependiencia temporal
w = sym('w',[3 1]);
dw = sym('dw',[3 1]);

% Torque
T = [0, 0, 0]';

% Ecuacion de Euler -> dw y w
[eq] = doc.Euler_Equation(I, w, dw, T);

% Introducit dependencia temporal: w(t), dw = dw/dt
syms w1(t) w2(t) w3(t)
dw1 = diff(w1, t);
dw2 = diff(w2, t);
dw3 = diff(w3, t);
eq = subs(eq,...
    {w(1) w(2) w(3) dw(1) dw(2) dw(3)}, ...
    {w1, w2, w3, dw1, dw2, dw3});
% pretty(eq)

% Integrar ecuaciones
cond = [w1(0)==w0(1), w2(0)==w0(2), w3(0)==w0(3)];
[w1, w2, w3] = dsolve(eq, cond);
disp('w1 = '); pretty(w1)
disp('w2 = '); pretty(w2)
disp('w3 = '); pretty(w3)
disp("HAY QUE QUITAR EL i DENTRO DE LA EXPONENCIAL")

disp(" "); disp("%   ---   ---   %"); disp(" ")


%% EJERCICIO 2
disp("EJERCICIO 2")
% Se suelta un objeto con velocidad angular

I = doc.Inertia_Matrix(100, 100, 150); % Aproximamos como simetrico (cilindro)
w0 = [0.1, 0.02, 0.5]';

% a) Attitude motion description

% b) Nutation
h = doc.Angular_Momentum_Iw(I, w0);
nutation_angle = doc.Nutation(h);
disp(['Nutation angle = ', num2str(nutation_angle)]);

% c) Precession rate
precession_rate = doc.Precession_Rate(I, h);
disp(['Precession rate = ', num2str(precession_rate)]);

disp(" "); disp("%   ---   ---   %"); disp(" ")


%% EJERCICIO 3
% Fantasia
w = 1*[1.5, 1.5, 1]';
I1 = 2;
I2 = 2;
I3 = 1;
I = doc.Inertia_Matrix(I1, I2, I3);

% Kinetic energy elipsoid
T = doc.Kinetic_Energy(I, w);
ke = w.^2./(2*diag(I));

% Angular momentum elipsoid
h = doc.Angular_Momentum_Iw(I, w);
he = w.^2./(2*h);

h = figure();
    hold on
    [xe, ye, ze] = ellipsoid(0, 0, 0, ke(1), ke(2), ke(3));
    [xh, yh, zh] = ellipsoid(0, 0, 0, he(1), he(2), he(3));
    surf(xe, ye, ze,'FaceColor',[0 0.4470 0.7410], 'FaceAlpha', 0.5, 'EdgeColor','none')
    surf(xh, yh, zh, 'FaceColor',[0.8500 0.3250 0.0980],'FaceAlpha', 0.75, 'EdgeColor','none')
    view(-30, 20)
    grid on; box on; axis equal


    
%% EJERCICIO 4
disp("EJERCICIO 2")
% Integrar ecuaciones dinamica - cinematica de actitud

% Variables independientes
Is = sym('Is',[3 1]);
th = sym('th',[3 1]);
dth = sym('dth',[3 1]);
w = sym('w',[3 1]);
dw = sym('dw',[3 1]);
To = sym('To',[3 1]);

% Condiciones iniciales
th0 = [0, 0, 0]';
w0 = [0.05, 0.02, -0.02]';


% CINEMATICA
% Transformacion velocidad angular
w21 = dth(1)*[1, 0, 0]' +...                       % Ultimo eje de giro - 1
    dth(2)*doc.C1(th(1))*[0, 1, 0]' +...              % Segundo eje de giro - 2
    dth(3)*doc.C1(th(1))*doc.C2(th(2))*[0, 0, 1]';       % Tercer eje de giro - 3

% Relacionar A*dw = w
eq_dth_th = w21 == w;

% Obtener A de la ecuacion anterior e invertirla para tener dw = B*w
vars = dth;
[A,b] = equationsToMatrix(eq_dth_th,vars);
eq_th_dth = dth == A\w;


% DINAMICA
I = doc.Inertia_Matrix(Is(1), Is(2), Is(3));
eq_dw_w = doc.Euler_Equation(I, w, dw, To);

% Subs the data given on the text
eq_dw_w = subs(eq_dw_w, To, [0;0;0]);
eq_dw_w = subs(eq_dw_w, Is, [210;200;180]);

% Differential system of equations
dif_eq = [eq_th_dth;eq_dw_w];
eq_splitted = children(dif_eq);
for i = 1:6
    dif_eq(i) = eq_splitted{i}{1,2};
end


clc
disp(dif_eq);

% Solve
[t,response] = ode45(@Compleate_Equation,[0 60], [th0; w0], odeset('RelTol',1e-4));

for i = 1:6
    h = figure();
    plot(t, response(:,i))
end
    
disp(" "); disp("%   ---   ---   %"); disp(" ")




%% FUNCTIONS

function dot_x = Compleate_Equation(t,x)
th1 = x(1);
th2 = x(2);
th3 = x(3);
w1=x(4);
w2=x(5);
w3=x(6);

dot_x = [...
(w1*cos(th2)*cos(th1)^2 + w3*sin(th2)*cos(th1) + w1*cos(th2)*sin(th1)^2 + w2*sin(th2)*sin(th1))/(cos(th2)*(cos(th1)^2 + sin(th1)^2))
                                                                               (w2*cos(th1) - w3*sin(th1))/(cos(th1)^2 + sin(th1)^2)
                                                                    (w3*cos(th1) + w2*sin(th1))/(cos(th2)*(cos(th1)^2 + sin(th1)^2))
                                                                                                                        (2*w2*w3)/21
                                                                                                                       -(3*w1*w3)/20
                                                                                                                          (w1*w2)/18
];
end
