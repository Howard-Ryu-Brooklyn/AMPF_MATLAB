clc; close all; clear;
addpath('Func\', 'Scripts\');

Declare_3_Agent_Acyclic_Minimally_Persistent_Graph();
Declare_Desired_Formation();

Set_Initial_Position();
ODE_Simulation();

Draw_Plots();


