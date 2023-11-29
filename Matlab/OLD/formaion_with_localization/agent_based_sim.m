clc; clear; close all;

addpath("Func/");

SimTime_Utils();

Graph_Formation();

Agent_Init();

Variables_Buffers();

% simulation loop
for k=1:t.length

    Mode_Flag_Managment();

    % Localization Law
    Localization_Law();

    % formation controller
    Formation_Controller();

    % Feedback linearization and saturation
    % Euler Integration
    Control_Input_and_Integral();

    % save buffer
    Save_Buffer();

end

Figure_Plots();