clc
clear
close all

%%

doc = doc_functions();


%% EJERCICIO 1
disp("EJERCICIO 1")
% SC orbitando la Tierra

% a) Matriz CbG
theta = deg2rad(45);
Cbg = doc.C3(theta);

% b) Coordenadas Sol Tierra en ejes cuerpo
neg = [-1, 0, 0]'; 
nsg = [0, 1, 0]';

% SC coordinates
neb = Cbg*neg;
nsb = Cbg*nsg;

% c) Triad
ub = neb;
vb = nsb;
ui = neg;
vi = nsg;
[Cb, Ci, Cbi] = doc.Triad_Method(neb, nsb, neg, nsg);
disp(Cbi)

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 2
disp("EJERCICIO 2")
% Aplicar Triad

ub = [0.8273, 0.5541, -0.0920]';
vb = [-0.8285, 0.5522, -0.0955]';

ui = [-0.1517, -0.9669, 0.2050]';
vi = [-0.8393, 0.4494, -0.3044]';

[Cb, Ci, Cbi] = doc.Triad_Method(ub, vb, ui, vi);
disp(Cbi)

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 3
disp("EJERCICIO 3")
% Integrar ecuaciones cinematica con condicion inicial w(0) 

% Variables independientes
th = sym('th',[3 1]);
dth = sym('dth',[3 1]);
syms t
w = [sin(0.1*t), 0, cos(0.1*t)]'*deg2rad(5);% w = sym('w',[3 1]);

% Condiciones iniciales
ci = deg2rad([80, 30, 40]');

% Transformacion velocidad angular
w21 = dth(1)*[1, 0, 0]' +...                            % Ultimo eje de giro - 1
    dth(2)*doc.C1(th(1))*[0, 1, 0]' +...                % Segundo eje de giro - 2
    dth(3)*doc.C1(th(1))*doc.C2(th(2))*[0, 0, 1]';      % Tercer eje de giro - 3

% Relacionar A*dw = w
eq_dth_th = w21 == w;

% Obtener A de la ecuacion anterior e invertirla para tener dw = B*w
vars = dth;
[A,b] = equationsToMatrix(eq_dth_th,vars);
eq_th_dth = dth == A\w;

% Separar x de dx
eq_splitted = children(eq_th_dth);
for i = 1:3
    eq_th_dth(i) = eq_splitted{i}{1,2};
end

% Copiar este display en la funcion del final
disp(eq_th_dth);

% Resolver sistema
[t,theta] = ode45(@kinematics_321,[0 60], ci, odeset('RelTol',1e-4));
theta = rad2deg(theta);
figure();
    plot(t,theta(:,1),t,theta(:,2),t,theta(:,3))
    box on; grid on
    title('Solution with ODE45');
    xlabel('t t');
    ylabel('Solution y');
    legend('\theta_1','\theta_2', '\theta_3', "Location","northwest" )

disp(" "); disp("%   ---   ---   %"); disp(" ")

    
%% EJERCICIO 4
disp("EJERCICIO 4")
% La misma mierda que arriba, pero con cuaterniones

ci = deg2rad([80, 30, 40]');
Ci = doc.C123(ci(1), ci(2), ci(3));
q0 = doc.Quaternions_from_C(Ci);

[t,q] = ode45(@kinematics_q,[0 60], q0, odeset('RelTol',1e-4));
figure();
    plot(t, q(:,1), t, q(:,2), t, q(:,3), t, q(:,4))
    box on; grid on
    title('Solution with ODE45');
    xlabel('t t');
    ylabel('Solution y');
    legend('q_1','q_2', 'q_3', 'q_4', "Location","northwest" )

disp(" "); disp("%   ---   ---   %"); disp(" ")


%% FUNCTIONS

function dot_theta = kinematics_321(t,x)
th1 = x(1);
th2 = x(2);
th3 = x(3);
dot_theta = [...
(pi*(sin(conj(t)/10)*cos(th1)^2*cos(th2) + sin(conj(t)/10)*cos(th2)*sin(th1)^2 + cos(conj(t)/10)*cos(th1)*sin(th2)))/(36*cos(th2)*(cos(th1)^2 + sin(th1)^2))
                                                                                               -(pi*cos(conj(t)/10)*sin(th1))/(36*(cos(th1)^2 + sin(th1)^2))
                                                                                       (pi*cos(conj(t)/10)*cos(th1))/(36*cos(th2)*(cos(th1)^2 + sin(th1)^2))
    ];
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