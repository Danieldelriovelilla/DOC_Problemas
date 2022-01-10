clc; clear all
%% Problem 1
E.r = 6371e3;    % [m]
G   = 6.674e-11; % [m3/kgs2]
E.m = 5.972e24;  % [kg]
mu  = G*E.m;
Sat1.hp = 300e3;
Sat1.ha = 10000e3;
% a) Semi major axis
Sat1.rp = Sat1.hp + E.r;
Sat1.ra = Sat1.ha + E.r;
Sat1.a  = (Sat1.rp + Sat1.ra)/2;
% b) Excentricity
Sat1.e  = 1-Sat1.rp/Sat1.a;
% c) vp
Sat1.vp = sqrt(2*(-mu/(2*Sat1.a)+mu/Sat1.rp));
% d) va
Sat1.va = sqrt(2*(-mu/(2*Sat1.a)+mu/Sat1.ra));
% e)
Sat1.T  = 2*pi*sqrt(Sat1.a^3/mu);

%% Problem 2
Sat2.hp = 800e3;
Sat2.rp = Sat2.hp+E.r;
Sat2.vp = 8e3;
% a) Semi major axis
Sat2.a = mu/(2*mu/Sat2.rp-Sat2.vp^2);
% b) Excentricity
Sat2.e  = 1-Sat2.rp/Sat2.a;
% c) r apogee
Sat2.ra = 2*Sat2.a - Sat2.rp;

%% Problem 4
omega_avg = 2*pi/(365*24*60*60);
J2        = 1.083e-3;
Sat3.a1   = 10000e3;
Sat3.a2   = 15000e3;
Sat3.a3   = 20000e3;
% Excentricity
Sat3.e1 = sqrt(1-sqrt(3*J2*E.r^2/(2*omega_avg)*sqrt(mu/(5*Sat3.a1^7))));
Sat3.e2 = sqrt(1-sqrt(3*J2*E.r^2/(2*omega_avg)*sqrt(mu/(5*Sat3.a2^7))));
Sat3.e3 = sqrt(1-sqrt(3*J2*E.r^2/(2*omega_avg)*sqrt(mu/(5*Sat3.a3^7))));
% Radii perigee
Sat3.rp1 = Sat3.a1*(1-Sat3.e1);
Sat3.rp2 = Sat3.a2*(1-Sat3.e2);
Sat3.rp3 = Sat3.a3*(1-Sat3.e3);
% Plot
a  = linspace(5000e3,30000e3);
e  = sqrt(1-sqrt(3*J2*E.r^2/(2*omega_avg)*sqrt(mu./(5*a.^7))));
rp = a.*(1-e);
rp(imag(rp)~= 0) = rp(imag(rp)~= 0)*0;
plot(a,rp)