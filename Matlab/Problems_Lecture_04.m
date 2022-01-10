clc; clear all
E.r = 6371e3;    % [m]
G   = 6.674e-11; % [m3/kgs2]
E.m = 5.972e24;  % [kg]
mu  = G*E.m;
%% Problem 1
Sail.a = 10^4; % [m2]
Sail.m = 1000; % [kg]
phi = 1361;    % [W/m2]
x = 4e8;       % [m]
c = 3e8;       % [m/s]
% Time to Moon solar sail
Sail.t = sqrt(2*x*c*Sail.m/(phi*2*Sail.a))/(3600*24);