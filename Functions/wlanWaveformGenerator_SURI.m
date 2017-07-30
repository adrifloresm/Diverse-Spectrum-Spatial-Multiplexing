function [txWaveform,mappedData] = wlanWaveformGenerator_SURI(dataBits,cfgFormat,cfgWaveGen)
% wlanWaveformGenerator WLAN waveform generation
%   WAVEFORM = wlanWaveformGenerator(DATA,CFGFORMAT,CFGWAVEGEN) generates a
%   waveform for a given format configuration, waveform generator
%   configuration, and information bits. The waveform contains one or more
%   packets of the same format and configuration. Each packet can contain
%   different information bits. For OFDM formats, the OFDM symbols in each
%   packet are optionally windowed.
%
%   WAVEFORM is a complex Ns-by-Nt matrix containing the generated
%   waveform, where Ns is the number of time domain samples, and Nt is the
%   number of transmit antennas.
%
%   DATA is the information bits including any MAC padding to be coded
%   across the number of packets to generate, i.e., representing multiple
%   concatenated PSDUs. It can be a double or int8 typed binary vector.
%   Alternatively, it can be a scalar cell array or a vector cell array
%   with length equal to number of users. Each element of the cell array
%   must be a double or int8 typed, binary vector. When DATA is a vector or
%   scalar cell array, it applies to all users. When DATA is a vector cell
%   array, each element applies to a single user. For each user, the bit
%   vector applied is looped if the number of bits required across all
%   packets of the generation exceeds the length of the vector provided.
%   This allows a short pattern to be entered, e.g. [1;0;0;1]. This pattern
%   will be repeated as the input to the PSDU coding across packets and
%   users. The number of data bits taken from a data stream for the ith
%   user when generating a packet is given by the ith element of the
%   CFGFORMAT.PSDULength property times eight.
%
%   CFGFORMAT is a format configuration object of type <a href="matlab:help('wlanVHTConfig')">wlanVHTConfig</a>, 
%   <a href="matlab:help('wlanHTConfig')">wlanHTConfig</a>, or <a href="matlab:help('wlanNonHTConfig')">wlanNonHTConfig</a>. The format of the generated waveform 
%   is determined by the type of CFGFORMAT. The properties of CFGFORMAT are
%   used to parameterize the packets generated including the data rate and
%   PSDU length.
%
%   CFGWAVEGEN is a waveform generator configuration object of type 
%   <a href="matlab:help('wlanGeneratorConfig')">wlanGeneratorConfig</a>. The properties of CFGWAVEGEN are used to control 
%   the waveform generator including the number of packets to generate,
%   idle time between packets, the initial state of the scrambler for OFDM
%   formats and spectral controls.
%
%   Examples:
%
%    Example 1:
%    %  Generate a time domain signal txWaveform for an 802.11ac VHT
%    %  transmission with 10 packets and 20 microsecond idle period between
%    %  packets.
%
%       cfgVHT = wlanVHTConfig();       % Create format configuration
%       % Change properties from defaults
%       cfgVHT.NumTransmitAntennas = 2; % 2 transmit antennas
%       cfgVHT.NumSpaceTimeStreams = 2; % 2 spatial streams
%       cfgVHT.MCS = 1;                 % Modulation: QPSK Rate: 1/2 
%       cfgVHT.APEPLength = 1024;       % A-MPDU length in bytes
%
%       cfgGen = wlanGeneratorConfig(); % Create generator configuration
%       cfgGen.NumPackets = 10;         % Generate 10 packets 
%       cfgGen.IdleTime = 20e-6;        % 20 microsecond idle period
%       cfgGen.WindowTransitionTime = 1e-7; % Window length in seconds
%
%       % Create bit vector containing concatenated PSDUs
%       numBits = cfgVHT.PSDULength*8*cfgGen.NumPackets;
%       dataBits = randi([0 1],numBits,1);
%
%       txWaveform = wlanWaveformGenerator(dataBits,cfgVHT,cfgGen);
%
%    Example 2:
%    %  Produce a waveform containing a single 802.11a packet without any 
%    %  windowing.
%          
%       cfgWaveGen = wlanGeneratorConfig(); % Generator configuration
%       cfgWaveGen.Windowing = false;       % Disable windowing
% 
%       cfgNonHT = wlanNonHTConfig(); % Create format configuration
%
%       psdu = randi([0 1],cfgNonHT.PSDULength*8,1); % Create a PSDU
%
%       txWaveform = wlanWaveformGenerator(psdu,cfgNonHT,cfgWaveGen);
% 
%
%   See also wlanGeneratorConfig, wlanVHTConfig, wlanHTConfig,
%            wlanNonHTConfig.

%   Copyright 2015 The MathWorks, Inc.

%#codegen

% Validate the format configuration object is a valid type
validateattributes(cfgFormat, ...
    {'wlanVHTConfig','wlanHTConfig','wlanNonHTConfig'}, ...
    {'scalar'}, mfilename, 'format configuration object');
s = validateConfig(cfgFormat);

% Validate the waveform generator config object is the correct type
validateattributes(cfgWaveGen, {'wlanGeneratorConfig'}, {'scalar'}, ...
   mfilename, 'waveform generator configuration object');      

if isa(cfgFormat, 'wlanVHTConfig')
    numUsers = cfgFormat.NumUsers;
else
    numUsers = 1;
end

% Cross validation
coder.internal.errorIf( ...
    all(size(cfgWaveGen.ScramblerInitialization, 2) ~= [1 numUsers]), ...
    'wlan:wlanWaveformGenerator:ScramInitNotMatchNumUsers');

% Validate that data bits are present if PSDULength is nonzero
if iscell(dataBits) % SU and MU
    % Data must be a scalar cell or a vector cell of length Nu
    coder.internal.errorIf(~isvector(dataBits) || ...
        all(length(dataBits) ~= [1 numUsers]), ...
        'wlan:wlanWaveformGenerator:InvalidDataCell');
    
    for u = 1:length(dataBits)
        if ~isempty(dataBits{u}) && any(cfgFormat.PSDULength > 0) % Data packet
            validateattributes(dataBits{u}, {'double','int8'}, ...
                {'real','integer','vector','binary'}, ...
                mfilename, 'each element in cell data input');
        else
            % Empty data check if not NDP
            coder.internal.errorIf( ...
                any(cfgFormat.PSDULength > 0) && isempty(dataBits{u}), ...
                'wlan:wlanWaveformGenerator:NoData');
        end
    end
    if (numUsers > 1) && isscalar(dataBits) 
        % Columnize and expand to a [1 Nu] cell
        dataCell = repmat({int8(dataBits{1}(:))}, 1, numUsers);
    else % Columnize each element
        dataCell = repmat({int8(1)}, 1, numUsers); 
        for u = 1:numUsers                
            dataCell{u} = int8(dataBits{u}(:));
        end
    end
else % SU and MU: Data must be a vector
    if ~isempty(dataBits) && any(cfgFormat.PSDULength > 0) % Data packet
        validateattributes(dataBits, {'double','int8'}, ...
            {'real','integer','vector','binary'}, ...
            mfilename, 'Data input');

        % Columnize and expand to a [1 Nu] cell
        dataCell = repmat({int8(dataBits(:))}, 1, numUsers);
    else % NDP
        % Empty data check if not NDP
        coder.internal.errorIf(any(cfgFormat.PSDULength > 0) && isempty(dataBits), ...
            'wlan:wlanWaveformGenerator:NoData');

        dataCell = {int8(dataBits(:))};
    end
end

% Number of bits in a PSDU for a single packet (convert bytes to bits)
numPSDUBits = cfgFormat.PSDULength*8;

% Repeat to provide initial state(s) for all users and packets
numPackets = cfgWaveGen.NumPackets;
scramInit = repmat(cfgWaveGen.ScramblerInitialization, 1, ...
    numUsers/size(cfgWaveGen.ScramblerInitialization, 2)); % For all users
pktScramInit = scramInit(mod((0:numPackets-1).', size(scramInit, 1))+1, :);

% Get the sampling rate of the waveform
if (isa(cfgFormat,'wlanNonHTConfig') && ...
        strcmpi(cfgFormat.Modulation,'DSSS')) % DSSS format
    sr = 11e6;
    numTxAnt = 1;
    giType = ''; % for codegen
    FFTLen = 0;  % for codegen
    info = dsssInfo(cfgFormat);
    numPktSamples = info.NumPPDUSamples;

    % disable windowing, not applicable to DSSS
    cfgWaveGen.Windowing = false;
    lstf = []; % for codegen
    lltf = []; % for codegen
    lsig = []; % for codegen
else % OFDM format
    if isa(cfgFormat, 'wlanNonHTConfig')
        giType = 'Long';  % Always
        chanBW = 'CBW20';
    else    % For VHT/HT formats
        giType = cfgFormat.GuardInterval;
        chanBW = cfgFormat.ChannelBandwidth;
    end
    num20 = real(str2double(chanBW(4:end)))/20;
    FFTLen = 64*num20;
    sr = num20*20e6;
    numTxAnt = cfgFormat.NumTransmitAntennas;
    numPktSamples = real(s.NumPPDUSamples); % real for codegen

    % Generate the legacy preamble fields for each PSDU
    lstf = wlanLSTF(cfgFormat);
    lltf = wlanLLTF(cfgFormat);
    lsig = wlanLSIG(cfgFormat);
end

if isa(cfgFormat, 'wlanVHTConfig') % VHT format
    % VHT Format
    vhtsiga = wlanVHTSIGA(cfgFormat);
    vhtstf = wlanVHTSTF(cfgFormat);
    vhtltf = wlanVHTLTF(cfgFormat);
    vhtsigb = wlanVHTSIGB(cfgFormat);
    preamble = [lstf; lltf; lsig; vhtsiga; vhtstf; vhtltf; vhtsigb];

elseif isa(cfgFormat, 'wlanHTConfig') % HT-MF format

    htSig = wlanHTSIG(cfgFormat);
    htstf = wlanHTSTF(cfgFormat);
    htltf = wlanHTLTF(cfgFormat);
    preamble = [lstf; lltf; lsig; htSig; htstf; htltf];

elseif isa(cfgFormat, 'wlanNonHTConfig')

    if strcmp(cfgFormat.Modulation, 'OFDM')
        preamble = [lstf; lltf; lsig];
    else % DSSS
        preamble = [wlanDSSSPreamble(cfgFormat); wlanDSSSHeader(cfgFormat)];
    end
end

% If windowing transition time is 0 then disable windowing
if cfgWaveGen.WindowTransitionTime==0
    cfgWaveGen.Windowing = false;
end

if cfgWaveGen.Windowing==true
    % Calculate parameters for windowing

    % IdleSample offset due to windowing 
    wlength = 2*ceil(cfgWaveGen.WindowTransitionTime*sr/2);
    bLen = wlength/2; % Number of samples overlap at end of packet
    aLen = bLen-1;    % Number of samples overlap at start of packet
    
    windowedPktLength = numPktSamples+wlength-1;
else    
    % Define unused windowing variables for codegen
    wlength = 0;
    windowedPktLength = numPktSamples+wlength-1;
    aLen = 0;
    bLen = 0;
end

% Define a matrix of total simulation length
numIdleSamples = round(sr*cfgWaveGen.IdleTime);
pktWithIdleLength = numPktSamples+numIdleSamples;
txWaveform = complex(zeros(numPackets*pktWithIdleLength,numTxAnt));

for i = 1:numPackets
    % Extract PSDU for the current packet
    psdu = getPSDUForCurrentPacket(dataCell, numPSDUBits, i);

    % Generate the PSDU with the correct scrambler initial state
    if  isa(cfgFormat, 'wlanVHTConfig')
        if any(cfgFormat.APEPLength > 0)
            data = wlanVHTData(psdu, cfgFormat, pktScramInit(i, :));
        else  % NDP
            data = complex(zeros(0, cfgFormat.NumTransmitAntennas));
        end
    elseif isa(cfgFormat, 'wlanHTConfig') % HT-MF format
        if cfgFormat.PSDULength > 0                    
            [data,mappedData] = wlanHTData_SURI(psdu{1}, cfgFormat, pktScramInit(i, :));
        else  % NDP or sounding packet
            [data,mappedData] = complex(zeros(0, cfgFormat.NumTransmitAntennas));
        end
    elseif isa(cfgFormat, 'wlanNonHTConfig') % NonHT format
        if strcmp(cfgFormat.Modulation, 'OFDM')
            data = wlanNonHTData(psdu{1}, cfgFormat, pktScramInit(i, :));
        else % DSSS
            data = wlanDSSSData(psdu{1}, cfgFormat);
        end
    end

    % Construct packet from preamble and data
    packet = [preamble; data];                

    if cfgWaveGen.Windowing==true
        % Window each packet
         windowedPacket = wlanWindowing(packet, FFTLen, wlength, giType, ...
             size(preamble,1));

        % Overlap-add the windowed packets
        if numPackets==1 && numIdleSamples==0 % Only one packet which wraps     
            txWaveform = windowedPacket(aLen+(1:numPktSamples), :);
            % Overlap start of packet with end
            txWaveform(1:bLen, :) = txWaveform(1:bLen, :)+ ...
                windowedPacket(end-bLen+1:end, :);
            % Overlap end of packet with start
            txWaveform(end-aLen+1:end, :) = txWaveform(end-aLen+1:end, :)+ ...
                windowedPacket(1:aLen, :);
        else
            if i==1 % First packet (which wraps)
                % First packet wraps to end of waveform
                txWaveform(1:(numPktSamples+bLen), :) = ...
                    windowedPacket(1+aLen:end, :);
                txWaveform(end-aLen+1:end, :) = windowedPacket(1:aLen, :);
            elseif i==numPackets && numIdleSamples==0 % Last packet which wraps
                % Last packet wraps to start of waveform
                startIdx = (i-1)*pktWithIdleLength-aLen+1;
                txWaveform(startIdx:end, :) = txWaveform(startIdx:end, :)+ ...
                    windowedPacket(1:end-bLen, :);
                txWaveform(1:bLen,:) = txWaveform(1:bLen, :)+ ...
                    windowedPacket(end-bLen+1:end, :);
            else % Packet does not wrap
                % Normal windowing overlap between packets
                idx = (i-1)*pktWithIdleLength-aLen+(1:windowedPktLength);
                txWaveform(idx, :) = txWaveform(idx, :)+windowedPacket;
            end
       end
    else
        % Construct entire waveform
        txWaveform((i-1)*pktWithIdleLength+(1:numPktSamples), :) = packet;
    end
end
end

function psdu = getPSDUForCurrentPacket(dataCell, numPSDUBitsPerPacket, ...
    packetIdx)
    numUsers = length(dataCell); % == length(numPSDUBits)
    psdu = repmat({int8(1)}, 1, numUsers); % Cannot use cell(1, numUsers) for codegen
    for u = 1:numUsers
        idx = mod((packetIdx-1)*numPSDUBitsPerPacket(u)+...
            (0:numPSDUBitsPerPacket(u)-1).', length(dataCell{u})) + 1;
        psdu{u} = dataCell{u}(idx);
    end
end

% [EOF]
