clc; close all; clear;
addpath('Scripts\');
% Distance-based Acyclic Minimally Persistent Formations with Non-steepest Descent Control
% 코드 형식에서 에이전트 모델 및 포화상태 추가.
% 제어기를 Flexible Coordination Control Law를 추가함

seed = 2;
rng(seed);

My_Utils();

for c=1:1
    Set_Formation();
    Set_Variables();
end

Simulation();
Draw_Plots();
