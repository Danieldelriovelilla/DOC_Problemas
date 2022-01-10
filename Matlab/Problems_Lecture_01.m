clc; clear all
%% Problem 1
Earth.r = 6371e3;    % [m]
G   = 6.674e-11; % [m3/kgs2]
Earth.m = 5.972e24;  % [kg]
% a) sea surface
a.Sea = G*Earth.m/Earth.r^2;
% b) 800 km
a.h800 = G*Earth.m/(Earth.r+800e3)^2;
% c) 20200 km
a.h20200 = G*Earth.m/(Earth.r+20200e3)^2;
% d) 35800 km
a.h35800 = G*Earth.m/(Earth.r+35800e3)^2;
%% Problem 2
h = 6.6262e-34; % Planck constant
c = 3e8; % speed of light [m/s]
% a) green light
E.green = h*c/525e-9;
% b) gamma ray
E.gamma = h*c/10e-12;
% c) radio wave
E.radio = h*c/1;