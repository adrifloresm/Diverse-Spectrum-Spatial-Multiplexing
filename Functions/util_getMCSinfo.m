function [mcsTable_all] = util_getMCSinfo(MCS)
% Exmaple Usage 
% util_getMCSinfo(10)
% Input MCS index, Returns mcsTable_all with information of:
% 
% mcsTable_all = struct( ...
%     'MCS_Index',  MCS,...
%     'ModText',    ModTex,...
%     'Nss',        Nss,...
%     'Datarate',   datarate,...
%     'Rate',       rate, ...
%     'NBPSCS',     Nbpscs, ...
%     'NCBPS',      Ncbps, ...
%     'NDBPS',      Ndbps, ...
%     'NSD',        Nsd, ...
%     'NES',        Nes);
% 
% Supports MCS values only in the range 0-31 for now.

% Strip Nss from MCS to get MC and Nss
Nss = floor(MCS/8)+1;
mc = rem(MCS, 8);
Nsd = 52 ; % Number of Data subcarriers

switch mc
    % 1 SS
    case 0
        ModTex = 'BPSK 1/2';
        rate   = 1/2;
        Nbpscs = 1; % 'BPSK'

    case 1
        ModTex = 'QPSK 1/2';
        rate   = 1/2;
        Nbpscs = 2; % 'QPSK'

    case 2
        ModTex = 'QPSK 3/4';
        rate   = 3/4;
        Nbpscs = 2; % 'QPSK'

    case 3
        ModTex = '16-QAM 1/2';
        Nbpscs = 4; % '16QAM'
        rate   = 1/2;

    case 4
        ModTex = '16-QAM 3/4';
        Nbpscs = 4;
        rate   = 3/4;

    case 5
        ModTex = '64-QAM 2/3';
        Nbpscs = 6; % '64QAM'
        rate   = 2/3;

    case 6
        ModTex = '64-QAM 3/4';
        Nbpscs = 6;
        rate   = 3/4;

    case 7
        ModTex = '64-QAM 5/6';
        Nbpscs = 6;
        rate   = 5/6;

    %-------------------------------
    % 2 SS
    case 8
        ModTex = 'BPSK 1/2';
        Nbpscs = 1; % 'BPSK'
        rate   = 1/2;

    case 9
        ModTex = 'QPSK 1/2';
        Nbpscs = 2; % 'QPSK'
        rate   = 1/2;

    case 10
        ModTex = 'QPSK 3/4';
        Nbpscs = 2;
        rate   = 3/4;

    case 11
        ModTex = '16-QAM 1/2';
        Nbpscs = 4; % '16QAM'
        rate   = 1/2;

    case 12
        ModTex = '16-QAM 3/4';
        Nbpscs = 4;
        rate   = 3/4;

    case 13
        ModTex = '64-QAM 2/3';
        Nbpscs = 6; % '64QAM'
        rate   = 2/3;

    case 14
        ModTex = '64-QAM 3/4';
        Nbpscs = 6;
        rate   = 3/4;

    case 15
        ModTex = '64-QAM 5/6';
        Nbpscs = 6;
        rate   = 5/6;

        
        
  %-------------------------------
    % 3 SS
    case 16
        ModTex = 'BPSK 1/2';
        Nbpscs = 1; % 'BPSK'
        rate   = 1/2;
    case 17
        ModTex = 'QPSK 1/2';
        Nbpscs = 2; % 'QPSK'
        rate   = 1/2;
    case 18
        ModTex = 'QPSK 3/4';
        Nbpscs = 2;
        rate   = 3/4;
    case 19
        ModTex = '16-QAM 1/2';
        Nbpscs = 4; % '16QAM'
        rate   = 1/2;
    case 20
        ModTex = '16-QAM 3/4';
        Nbpscs = 4;
        rate   = 3/4;
    case 21
        ModTex = '64-QAM 2/3';
        Nbpscs = 6; % '64QAM'
        rate   = 2/3;
    case 22
        ModTex = '64-QAM 3/4';
        Nbpscs = 6;
        rate   = 3/4;
    case 23
        ModTex = '64-QAM 5/6';
        Nbpscs = 6;
        rate   = 5/6;
        
        
    %-------------------------------
    % 4 SS
    case 24
        ModTex = 'BPSK 1/2';
        Nbpscs = 1; % 'BPSK'
        rate   = 1/2;
    case 25
        ModTex = 'QPSK 1/2';
        Nbpscs = 2; % 'QPSK'
        rate   = 1/2;
    case 26
        ModTex = 'QPSK 3/4';
        Nbpscs = 2;
        rate   = 3/4;
    case 27
        ModTex = '16-QAM 1/2';
        Nbpscs = 4; % '16QAM'
        rate   = 1/2;
    case 28
        ModTex = '16-QAM 3/4';
        Nbpscs = 4;
        rate   = 3/4;
    case 29
        ModTex = '64-QAM 2/3';
        Nbpscs = 6; % '64QAM'
        rate   = 2/3;
    case 30
        ModTex = '64-QAM 3/4';
        Nbpscs = 6;
        rate   = 3/4;
    case 31
        ModTex = '64-QAM 5/6';
        Nbpscs = 6;
        rate   = 5/6;
end % Switch mc


% bit rate = symbol rate * bits/symbol * GI * FEC
Ncbps = Nsd * Nbpscs * Nss; % NUmber of coded bits per symbol
Ndbps = Ncbps * rate; % number of data bits per symbol
datarate = Ndbps/4e-6; % Ndbps divided by the symbol time 4us ( 3.2us OFDM symbol plus 0.8 GI )

% Any Ndbps>1200 => Nes=2, for 300 Mbps per encoder
%   Confirmed with Tables 20-30 to 20-44.
Nes = ceil(Ndbps/(4*300));
% Nes = % number of encoding streams - NES = 1 for 1x1 and 2x2 systems, and NES = 2 for 3x3 and 4x4 systems. 

mcsTable_all = struct( ...
    'MCS_Index',  MCS,...
    'ModText',    ModTex,...
    'Nss',        Nss,...
    'Datarate',   datarate,...
    'Rate',       rate, ...
    'NBPSCS',     Nbpscs, ...
    'NCBPS',      Ncbps, ...
    'NDBPS',      Ndbps, ...
    'NSD',        Nsd, ...
    'NES',        Nes);