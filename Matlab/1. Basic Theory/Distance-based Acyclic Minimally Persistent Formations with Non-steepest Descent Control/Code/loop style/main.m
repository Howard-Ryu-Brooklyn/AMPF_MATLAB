clc; close all; clear;
addpath('Func\', 'Scripts\');

My_Utils();

Declare_3_Agent_Acyclic_Minimally_Persistent_Graph();
Declare_Desired_Formation();

Set_Initial_Position();
Simulation();

Draw_Plots();


