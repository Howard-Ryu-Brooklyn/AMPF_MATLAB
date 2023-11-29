clc; close all; clear;
addpath("Scripts\");
% In paper, localization in 3D is proposed
% but simulation was conducted in 2D

% L struct
% nonholonomic agent + saturation
% feedback linearization
% p.e. input
% localization finish condition

My_Utils();

Set_Variables();

Simulation();

Draw_Plots();

