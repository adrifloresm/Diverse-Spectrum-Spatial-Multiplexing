%channel.m
%
%Channel for 802.11a transmissions
%
%Created by Sadia Quadri 11/3/2013



function [RX] = channel(TX,chan,freq_offset)

switch chan
    case 'AWGN'
        h = 1;
       
    case 'NARROW'
        % create Narrowband rayleigh fading channel for testing
        h =[1/sqrt(2)*(randn(1,1)+1j*randn(1,1)) zeros(1,63)];
        
    case 'ETSI_A'
        % create wideband rayleigh fading channel for testing
        h =[H2ChannelA zeros(1,64-8)];
        
        
end

RX= conv(h,TX);


% add a common frequency offset 
angle = (2*pi*freq_offset) / 20e6;         % frequency offset angle per sample
angle_v = [0:1:size(RX,2)-1] * angle;
cv = cos(angle_v) + j*sin(angle_v);

RX = RX .* cv;


