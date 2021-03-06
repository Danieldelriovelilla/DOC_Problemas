clc
clear
close all

%% Problem 1
%{
I = 1;    % kg m²
Mp = 0.2;
ts = 60;  % s

syms theta u
eq = diff(diff(theta)) * I - u ;

% Parámetros deseados
syms dzeta 
eq = Mp == exp(-pi*dzeta/(1-dzeta)^0.5);
[sol] = vpasolve(eq,dzeta);

dzeta = sol.dzeta

%Definir tf genérica
s = syms2tf(eq);

%Bucle cerrado 

s = s/(1+s);

%Encontrar polos y zeros
z = zero(s);
p = pole(s);

pzplot(s)
grid

%Definir PD
Kp = 1;
Ki = 1;
Kd = 1;
C = Kp + Kd*s
%}

%%

doc = doc_functions();


%% EJERCICIO 1
disp("EJERCICIO 1")
% Diseñar PID con restricciones de Mp y ts

I = 1;
Mp = 0.2;
ts = 60;

xi_wn = 4.4/ts;
wd = -pi*xi_wn/log(Mp);

s = -xi_wn + imag(wd);

Kp = I*norm(s);
Kd = 2*xi_wn*I;

disp(['Control law: u = ', num2str(Kp), 'e - ', num2str(Kd), 'dy'])



%% EJERCICIO 2

syms s I kp kd xi wn wd

% kp = 0.1;
% kd = 0.5;

A = 1/(I*s^2);
B = kd*s;
C = kp;

D = Closed_Loop(A, B);
E = Followed(D, C);
F = Closed_Loop(E, 1);

[Num,Den] = numden(F);
Num = Num/I;
Den = expand(Den/I);

Den = subs(Den,...
    {kp/I, kd/I}, ...
    {wn^2, 2*xi*wn});

pretty(Den)

kp = 0.1;
kd = 0.5;
I = 10;

wn = sqrt(kp/I);
xi = kd/(2*wn*I);

ts = Settling_Time(xi, wn)
Mp = Overshoot_xi(xi)
wd = wd_From_xi_wn(xi,wn)
tr = Rise_Time(xi, wd)












function C = Closed_Loop(G, H)
C = simplify(G/(1 + G*H));
end
function C = Followed(G, H)
C = simplify(G*H);
end
function ts = Settling_Time(xi, wn)
ts = 4.4/(xi/wn);
end
function Mp = Overshoot_xi(xi)
Mp = exp(-pi*xi/sqrt(1-xi^2));
end
function wd = wd_From_xi_wn(xi, wn)
wd = wn*sqrt(1-xi^2);
end
function tr = Rise_Time(xi, wd)
b = atan(sqrt(1-xi^2)/xi);
tr = (pi - b)/wd;
end






function[TF] = syms2tf(eq)
% Get transfer function from equation
% 
%   input eq (sym): symbolic equation
% 
%   output TF (tf): transfer function
%

  G = laplace(eq);             %Get Laplace transform of Eq
  [symNum,symDen] = numden(G); %Get num and den of Symbolic TF
  TFnum = sym2poly(symNum);    %Convert Symbolic num to polynomial
  TFden = sym2poly(symDen);    %Convert Symbolic den to polynomial
  TF =tf(TFnum,TFden);
  
end