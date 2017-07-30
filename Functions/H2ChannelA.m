function [impulse_resp_50ns,mean_pwr] = H2ChannelA();

% This function generates a channel impulse response according to the HIPERLAN/2 channel 'A'
% Instead of 10 ns taps, the 10 ns impulse response is grouped to give 50 ns taps
% Mike Butler, 20/03/01
%
%Modified to correct errors, Paul Fletcher, September 2001

% Form the 10 ns impulse response
impulse_resp_10ns = zeros(1,40);

delays_10ns = [0 1 2 3 4 5 6 7 8 9 11 14 17 22 24 29 34 39];
tap_power_10ns = [0.0 -0.9 -1.7 -2.6 -3.5 -4.3 -5.2 -6.1 -6.9 -7.8 -4.7 -7.3 -9.9 -12.5 -13.7 -18.0 -22.4 -26.7];

impulse_resp_10ns(delays_10ns+1) = 10.^(tap_power_10ns/20); % Tap power to tap amplitude

% Calculate mean power
mean_pwr = sum(impulse_resp_10ns.^2);

% Impose Rayleigh fading on each tap
a=(1/sqrt(2))*(randn(1,length(impulse_resp_10ns)) + j*randn(1,length(impulse_resp_10ns)));
impulse_resp_10ns = impulse_resp_10ns .* a;

% Add groups of 5 taps to give the 50 ns impulse response 
impulse_resp_50ns = sum(reshape(impulse_resp_10ns,5,8),1);