clc; close all; clear;

r = 0.033; % [m] wheel radius
l = 0.16; % [m] wheel seperation
wheel_separation_multiplier = 1.1112;
l_sudo = l*wheel_separation_multiplier;

vmax = 0.22; % [m/s]
wmax = 2.84; % [rad/s]
cmd = [vmax; wmax];

M = r*[0.5, 0.5; 
    0.5/l, -0.5/l];
M_sudo = r*[0.5, 0.5; 
    0.5/l_sudo, -0.5/l_sudo];

wheel_maxvelocity = inv(M)*cmd
wheel_maxvelocity_sudo = inv(M_sudo)*cmd
