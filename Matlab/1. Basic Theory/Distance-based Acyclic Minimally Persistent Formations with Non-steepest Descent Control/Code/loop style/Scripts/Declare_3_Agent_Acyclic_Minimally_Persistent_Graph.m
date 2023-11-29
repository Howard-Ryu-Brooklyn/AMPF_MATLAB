% ------------------------------------------------------------------------------
% Declare 3-agent ayclic minimally acyclic persistent graph
% ------------------------------------------------------------------------------
n = 3; % # of nodes(agents)
G = digraph([2 3 3],[1 1 2]);
G.Nodes.names = {'1', '2', '3'}';

figure(1),
plot(G,'NodeLabel', G.Nodes.names)
title('network topology of AMPF');
% ------------------------------------------------------------------------------