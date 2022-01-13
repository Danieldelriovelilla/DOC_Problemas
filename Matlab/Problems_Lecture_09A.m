clc
clear
close all

doc = doc_functions();

%%  EJERCICIO 1

% DATOS
% Cilindro
syms m r l
l = 4*r;

% Euler equation
syms W0
w0 = W0*[3/5, 0, 4/5]';

% Angular velocity
% syms w1(t) w2(t) w3(t)  
w = sym('w',[3 1]);

% Angular velocity rate
% dw1 = diff(w1,t); dw2 = diff(w2,t); dw3 = diff(w3,t);
dw = sym('dw',[3 1]);

% Torque
T = [0, 0, 0]';

%  Inetia
[I] = doc.Inertia_Cylinder(m, r, l);

% Euler equation
% [eq] = Euler_Equation(I, [w1; w2; w3], [dw1; dw2; dw3], T)
[eq] = doc.Euler_Equation(I, w, dw, T);


% cond1 = dw1(0) == w0(1);
% cond2 = dw2(0) == w0(2);
% cond3 = dw3(0) == w0(3);
% conds = [cond1 cond2 cond3];
% uSol(t) = dsolve(eq,conds)


%% EJERCICIO 2

I = [98, 0, 0;
     0, 102, 0;
     0, 0, 150];
I(1,1) = 100;   % Aproximamos como simetrico (cilindro)
I(2,2) = 100;   % Aproximamos como simetrico (cilindro)

w0 = [0.1, 0.02, 0.5]';

h = doc.Angular_Momentum_Iw(I, w0);

nutation_angle = doc.Nutation(h);
precession_angle = doc.Precession_Rate(I, h);

%% EJERCICIO 3

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

% Variables independientes
Is = sym('Is',[3 1]);
th = sym('th',[3 1]);
dth = sym('dth',[3 1]);
w = sym('w',[3 1]);
dw = sym('dw',[3 1]);
To = sym('To',[3 1]);
% Condiciones iniciales
t0 = sym('t0',[3 1]);
w0 = sym('w0',[3 1]);


% Relation beween angle and angle rate
w21 = dth(1)*[1, 0, 0]' +...                       % Ultimo eje de giro - 1
    dth(2)*doc.C1(th(1))*[0, 1, 0]' +...              % Segundo eje de giro - 2
    dth(3)*doc.C1(th(1))*doc.C2(th(2))*[0, 0, 1]';       % Tercer eje de giro - 3
eq_dth_th = w21 == w;
% Solve dth = inv(A(th))*w
vars = dth;
[A,b] = equationsToMatrix(eq_dth_th,vars);
invA = inv(A);
eq_th_dth = dth == invA*w;

% Relation between angular velocity and angular velocity rate -> Euler eq
I = doc.Inertia_Matrix(Is(1), Is(2), Is(3));
eq_dw_w = doc.Euler_Equation(I, w, dw, To);
for i = 1:3
    eq_dw_w(i) = isolate(eq_dw_w(i), dw(i));
end

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
% Initial values
th0 = [0, 0, 0]';
w0 = [0.05, 0.02, -0.02]';

% Solve
[t,response] = ode45(@Compleate_Equation,[0 60], [th0; w0], odeset('RelTol',1e-4));

for i = 1:6
    h = figure();
    plot(t, response(:,i))
end
    


%% FUNCTIONS

function dot_x = Compleate_Equation(t,x)
th1 = x(1);
th2 = x(2);
th3 = x(3);
w1=x(4);
w2=x(5);
w3=x(6);

dot_x = [...
w1 + (w3*cos(th1)*sin(th2))/(cos(th2)*cos(th1)^2 + cos(th2)*sin(th1)^2) + (w2*sin(th1)*sin(th2))/(cos(th2)*cos(th1)^2 + cos(th2)*sin(th1)^2)
                                                           (w2*cos(th1))/(cos(th1)^2 + sin(th1)^2) - (w3*sin(th1))/(cos(th1)^2 + sin(th1)^2)
                       (w3*cos(th1))/(cos(th2)*cos(th1)^2 + cos(th2)*sin(th1)^2) + (w2*sin(th1))/(cos(th2)*cos(th1)^2 + cos(th2)*sin(th1)^2)
                                                                                                                                (2*w2*w3)/21
                                                                                                                               -(3*w1*w3)/20
                                                                                                                                  (w1*w2)/18
     ];
end
