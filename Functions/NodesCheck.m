N = 4;
nodes = wl_initNodes(N);
wl_nodeCmd(nodes,'identify');
disp(nodes)

nodes(2).transport.getMaxPayload

% OS reduced recv buffer size to 425984
% sudo sysctl -w net.core.rmem_max=8388608
% 
% Jumbo Frames
% ip link show eth1
% sudo ip link set eth2 mtu 9000
%
% PATH
%/usr/local/MATLAB/R2015a/toolbox/local
%sudo chmod a+w pathdef.m