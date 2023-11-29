clc; close all; clear;
addpath('Func\', 'Scripts\');
% Distance-based Acyclic Minimally Persistent Formations with Non-steepest Descent Control 
% 코드 형식에서 에이전트 모델 및 포화상태 추가.
% 제어기를 Flexible Coordination Control Law를 추가함

My_Utils();

seed = 1000;
rng(seed);

for c=1:50
Declare_3_Agent_Acyclic_Minimally_Persistent_Graph();
Declare_Desired_Formation();

Set_Initial_Position();
Simulation();

Draw_Plots();
end


