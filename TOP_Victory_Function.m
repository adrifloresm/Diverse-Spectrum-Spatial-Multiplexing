% Copyright Adriana Flores

% Top Function file for General Performance Evaluation (Scaling and Spatial Multiplexing)
% This file is used by all experiment TOP files
% Vaiables are modified depending on the experiment

clc
clear
% close all

addpath 'Functions'
%% Variables
%Variables For OUTSIDE of the Function
numiter=1; % Number of iterations
numpackets=1; % Number of packets
hardware=0; % Using Hardware if 0 = Simulation

savefile=0; % Save var files

nodes=[]; % Initialize nodes in case of Sim
if hardware
    NUMNODES=5;
    nodes = wl_initNodes(NUMNODES);
end

% MCS matrix
% ROWS = NO. Streams COL = MCS type
%  1        2         3        4         5         6        7         8
%BPSK1/2, QPSK1/2, QPSK3/4, 16QAM1/2, 16QAM3/4, 64QAM2/3, 62QAM3/4, 64QAM5/6
mcs_all= ...
    [  0,  1,  2,  3,  4,  5,  6,  7;... %1ss
    8,  9, 10, 11, 12, 13, 14, 15;... %2ss
    16, 17, 18, 19, 20, 21, 22, 23;... %3ss
    24, 25, 26, 27, 28, 29, 30, 31];   %4ss

 mcstoeval=[1 2 3 4 5 6 7 8]; %MCS to eval per transmission (1-8) see comment above
% mcstoeval=[8]; %MCS to eval per transmission (1-8) see comment above

% GAINS WARPLAB
txgains_all= ...
    [ 8 8 8 8;... %1ss
    9 9 8 8;... %2ss
    8 6 7 8;... %3ss
    8 8 8 8];   %4ss

rxgains_all= ...
    [ 12 12 12 11;... %1ss
    9 11 12 11;... %2ss
    10 12 12 12;... %3ss 10 9 9 9
     10 11 12 11];   %4ss   9 9 12 10
failedit=[];


%% Run Script
for numtx=1:4 %Transmitters (1-4)
    % Filename to save data
    filename=sprintf('/media/rrng/HD1TB/Victory/Victory_Tx%d_Iter%d_feb20_DH1049.mat',numtx,numiter);
    %     filename=sprintf('/home/rrng/Dropbox/ChameleonExperiments/Victory/Data/Victory_Tx%d_Iter%d.mat',numtx,numiter);
    
    
    for imcs=1:length(mcstoeval)  %MCS vector to evaluate (1 to 8)
        for k=1:numiter % Number of packets
            try
                mcs_sel = mcs_all(numtx,mcstoeval(imcs));
                tx_gain_sel=txgains_all(numtx,:); % rows tx, cols all gains
                rx_gain_sel=rxgains_all(numtx,:); % rows tx, cols all gains
                %% Variables inside Function
                USE_WARPLAB_TXRX            = hardware;    % Enable WARPLab-in-the-loop (otherwise sim-only)
                TX_SCALE                    = 1.0;         % Scale for Tx waveform ([0:1])
                if hardware
                    INTERP_RATE             = 2;           % Interpolation rate (1 or 2)
                    LTF_thresh              = 0.8;          % 0.8 / 0.1 to fail  - Higher threshold for Hardware
                else
                    INTERP_RATE             = 1;           % Interpolation rate (1 or 2)
                    LTF_thresh              = 0.3;
                end
                DECIMATE_RATE               = INTERP_RATE;
                
                % WARPLab experiment params
                USE_AGC                     = false;        % Use the AGC if running on WARP hardware
                rxagc_targ                  = -12; % Rx agc target when agc automatic
                MAX_TX_LEN                  = 2^17;        % Maximum number of samples to use for this experiment
                chan24                      = 14;           % Channel at 2.4 GHz
                USE_EXTERNAL_TRIGGER        = 1; % Use external trigger between Tx nodes
                trigger_delay               = [0 0 0]; % Delay per slave 1,2 or 3
                txgains                     = tx_gain_sel; %warplab Tx gain per transmitter (1-4)
                RxGainBB_vec                = rx_gain_sel; % Rx Gains BB
                RxGainRF_vec                = [3 3 3 3]; % Rx Gains RF
                
                %Start creating a 802.11n packet
                % Create a format configuration object for a HT transmission
                cfgHT                       = wlanHTConfig;
                cfgHT.ChannelBandwidth      = 'CBW20'; % 20 MHz channel bandwidth
                cfgHT.NumTransmitAntennas   = numtx;    % 2 transmit antennas
                cfgHT.NumSpaceTimeStreams   = cfgHT.NumTransmitAntennas;    % 2 space-time streams
                cfgHT.PSDULength            = 200;  % PSDU length in bytes
                cfgHT.MCS                   = mcs_sel;    %MCS
                NumReceiveAntennas          = 4;% cfgHT.NumTransmitAntennas;  % Number of Rx antennas
                MCSinfo                     = util_getMCSinfo(mcs_sel); % ALl info of MCS
                
                % Plotting and RX Stats
                cf                          = 99; % Figure counter
                ConstPlot                   = 0; % Show Constellation Plot
                plot_txrxair                = 0; % Plot Tx and Rx Vector Air
                RxStats                     = 1; % Display Rx Stats
                
                % Create and configure a waveform generator configuration object
                NoLTS                       = 0;  % No LTS
                cfgWaveGen                  = wlanGeneratorConfig;
                cfgWaveGen.NumPackets       = numpackets;
                
                % Define a half-band 2x interpolation filter response
                interp_filt2 = zeros(1,43);
                interp_filt2([1 3 5 7 9 11 13 15 17 19 21]) = [12 -32 72 -140 252 -422 682 -1086 1778 -3284 10364];
                interp_filt2([23 25 27 29 31 33 35 37 39 41 43]) = interp_filt2(fliplr([1 3 5 7 9 11 13 15 17 19 21]));
                interp_filt2(22) = 16384;
                interp_filt2 = interp_filt2./max(abs(interp_filt2));
                
                fprintf('\nTransmitters %d at MCS %d, %s : %d Mbps, Hardware %d Iteration: %d\n',numtx, mcs_sel, MCSinfo.ModText,MCSinfo.Datarate/1e6 ,hardware, k);
                %% SETUP WARPLAB
                if(USE_WARPLAB_TXRX)
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Set up the WARPLab experiment
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Create a vector of node objects
                    node_tx4 = nodes(5);
                    node_tx3 = nodes(4);
                    node_tx2 = nodes(3);
                    node_tx1 = nodes(2);
                    node_rx = nodes(1);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %TRIGER
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Create a UDP broadcast trigger and tell each node to be ready for it
                    eth_trig = wl_trigger_eth_udp_broadcast;
                    wl_triggerManagerCmd(nodes, 'add_ethernet_trigger', [eth_trig]);
                    
                    % Read Trigger IDs into workspace
                    [T_IN_ETH_A, T_IN_ENERGY, T_IN_AGCDONE, T_IN_REG, T_IN_D0, T_IN_D1, T_IN_D2, T_IN_D3, T_IN_ETH_B] =  wl_getTriggerInputIDs(node_tx1);
                    [T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(node_tx1);
                    
                    if (USE_EXTERNAL_TRIGGER)
                        %Receiver-AP  - allow Ethernet to trigger the buffer baseband, the AGC,
                        node_rx.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0],[T_IN_ETH_A]);
                        
                        switch cfgHT.NumTransmitAntennas
                            case 1
                                % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
                                wl_triggerManagerCmd(nodes, 'output_config_input_selection', [T_OUT_BASEBAND, T_OUT_AGC], [T_IN_ETH_A]);
                                
                                % Set the trigger output delays.
                                nodes.wl_triggerManagerCmd('output_config_delay', [T_OUT_BASEBAND], 0);
                                nodes.wl_triggerManagerCmd('output_config_delay', [T_OUT_AGC], 3000);     %3000 ns delay before starting the AGC
                            case 2
                                % Activating Master through Ethernet and output
                                % pins D0 (8) - delay and D3(11) - guide
                                
                                % Master Tx
                                % allow Ethernet to trigger the buffer baseband, the AGC, and debug0 (which is mapped to pin 8 on the debug header)
                                % T_OUT_D0 = 8
                                node_tx1.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0, T_OUT_D3],[T_IN_ETH_A]);
                                
                                %Slave
                                % T_IN_D3 = pin 15
                                %Allow debug 3 (15) to trigger buffer baseband, and the AGC
                                node_tx2.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                %Enable the debounce circuitry on the trigger input to help with noise on the signal line
                                node_tx2.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                
                                % Since the debounce circuitry is enabled, there will be a delay at the receiver node for its input trigger.
                                % To better align the transmitter and receiver, we can artificially delay the transmitters trigger outputs that drive the buffer baseband and the AGC.
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_BASEBAND, T_OUT_AGC], 56.25); % 56.25ns delay
                                node_rx.wl_triggerManagerCmd('output_config_delay', [T_OUT_AGC], 3000);                   % 3000ns delay
                                
                                %Delay
                                %On the master node: pin 8 (D0) -> node 3
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D0], trigger_delay(1));  % Node 1 with Serial # 210 has a delay step of 6.250 ns and a maximum delay of 193.750 ns.
                                
                                %pin 11 (D3) on the master node always outputs 0 delay for reference
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D3], 0);
                                
                            case 3
                                % Activating Master through Ethernet and output
                                % pins D0 (8)  pin D1 (9) - delay
                                % D3(11) - guide
                                % T_IN_D3 = 15
                                %All slave nodes are triggered through pin 15 on the debugger header
                                node_tx1.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D3],[T_IN_ETH_A]);
                                node_rx.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0],[T_IN_ETH_A]);
                                
                                node_tx2.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                node_tx2.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                node_tx3.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                node_tx3.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                
                                %On the master node: pin 8 (D0) -> node 3(slave 1), pin 9 (D1) -> node 4 (slave2)
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D0], trigger_delay(1));
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D1], trigger_delay(2));
                                
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_BASEBAND, T_OUT_AGC], 56.25); % 56.25ns delay
                                node_rx.wl_triggerManagerCmd('output_config_delay', [T_OUT_AGC], 3000);                   % 3000ns delay
                                
                                %pin 11 (D3) on the master node always outputs 0 delay for reference
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D3], 0);
                            case 4
                                % Activating Master through Ethernet and output
                                % pins D0 (8)  pin D1 (9) pin D2(10) - delay
                                % D3(11) - guide
                                % T_IN_D3 = 15
                                %All slave nodes are triggered through pin 15 on the debugger header
                                node_tx1.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3],[T_IN_ETH_A]);
                                node_rx.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC, T_OUT_D0],[T_IN_ETH_A]);
                                
                                node_tx2.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                node_tx2.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                node_tx3.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                node_tx3.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                node_tx4.wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND, T_OUT_AGC], [T_IN_D3]);
                                node_tx4.wl_triggerManagerCmd('input_config_debounce_mode', [T_IN_D3], 'enable');
                                
                                %On the master node: pin 8 (D0) -> node 3, pin 9 (D1) -> node 4, pin 10 (D2) -> node 5
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D0], trigger_delay(1));
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D1], trigger_delay(2));
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D2], trigger_delay(3));
                                
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_BASEBAND, T_OUT_AGC], 56.25); % 56.25ns delay
                                node_rx.wl_triggerManagerCmd('output_config_delay', [T_OUT_AGC], 3000);                   % 3000ns delay
                                
                                %pin 11 (D3) on the master node always outputs 0 delay for reference
                                node_tx1.wl_triggerManagerCmd('output_config_delay', [T_OUT_D3], 0);
                        end % switch TX for trigger
                    else
                        % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
                        wl_triggerManagerCmd(nodes, 'output_config_input_selection', [T_OUT_BASEBAND, T_OUT_AGC], [T_IN_ETH_A]);
                        
                        % Set the trigger output delays.
                        nodes.wl_triggerManagerCmd('output_config_delay', [T_OUT_BASEBAND], 0);
                        nodes.wl_triggerManagerCmd('output_config_delay', [T_OUT_AGC], 3000);     %3000 ns delay before starting the AGC
                    end % external trigger
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Interfaces
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % Get IDs for the interfaces on the boards. Since this example assumes each
                    % board has the same interface capabilities, we only need to get the IDs
                    % from one of the boards
                    [RFA,RFB,RFC,RFD] = wl_getInterfaceIDs(node_rx);
                    
                    % Set up the interface for the experiment
                    wl_interfaceCmd(node_tx1, RFA, 'tx_gains', 1, txgains(1));
                    wl_interfaceCmd(node_tx2, RFA, 'tx_gains', 1, txgains(2));
                    wl_interfaceCmd(node_tx3, RFA, 'tx_gains', 1, txgains(3));
                    wl_interfaceCmd(node_tx4, RFA, 'tx_gains', 1, txgains(4));
                    
                    %Channel
                    wl_interfaceCmd(node_tx1,RFA, 'channel',2.4, chan24);
                    wl_interfaceCmd(node_tx2,RFA, 'channel',2.4, chan24);
                    wl_interfaceCmd(node_tx3,RFA, 'channel',2.4, chan24);
                    wl_interfaceCmd(node_tx4,RFA, 'channel',2.4, chan24);
                    wl_interfaceCmd(node_rx,RFA+RFB+RFC+RFD, 'channel',2.4, chan24);
                    
                    if(USE_AGC)
                        wl_interfaceCmd(node_rx, RFA+RFB+RFC+RFD, 'rx_gain_mode', 'automatic');
                        wl_basebandCmd(node_rx, 'agc_target', rxagc_targ);
                        wl_basebandCmd(node_rx, 'agc_dco', true);
                    else
                        wl_interfaceCmd(node_rx, RFA+RFB+RFC+RFD, 'rx_gain_mode', 'manual');
                        %                     RxGainRF = 1;                  % Rx RF Gain in [1:3]
                        %                     RxGainBB = 15;                 % Rx Baseband Gain in [0:31]
                        %                     wl_interfaceCmd(node_rx, RFA+RFB+RFC+RFD, 'rx_gains', RxGainRF, RxGainBB);
                        wl_interfaceCmd(node_rx, RFA, 'rx_gains', RxGainRF_vec(1), RxGainBB_vec(1));
                        wl_interfaceCmd(node_rx, RFB, 'rx_gains', RxGainRF_vec(2), RxGainBB_vec(2));
                        wl_interfaceCmd(node_rx, RFC, 'rx_gains', RxGainRF_vec(3), RxGainBB_vec(3));
                        wl_interfaceCmd(node_rx, RFD, 'rx_gains', RxGainRF_vec(4), RxGainBB_vec(4));
                    end
                    %------------------------------------------------------------------------------------------------
                    % Set up the TX / RX nodes and RF interfaces
                    % Get parameters from the node
                    SAMP_FREQ    = wl_basebandCmd(node_tx1, 'tx_buff_clk_freq');
                    Ts           = 1/SAMP_FREQ;
                    
                    % We will read the transmitter's maximum I/Q buffer length
                    % and assign that value to a temporary variable.
                    %
                    % node_tx - WARPLab node object for the transmitter30
                    % RF_TX - index of RF interface used for transmission
                    
                    maximum_buffer_len = wl_basebandCmd(node_tx1, RFA, 'tx_buff_max_num_samples');
                    
                    
                    % Our transmission length for this example does not need
                    % to fill the entire transmit buffer, so we will use the smaller
                    % of two values: the maximum buffer length the board
                    % can support or an arbitrary value defined by this script
                    
                    TX_NUM_SAMPS = min(MAX_TX_LEN, maximum_buffer_len);
                    
                    
                    % Set up the baseband for the experiment
                    wl_basebandCmd(nodes, 'tx_delay', 0);
                    wl_basebandCmd(nodes, 'tx_length', TX_NUM_SAMPS);      % Number of samples to send
                    wl_basebandCmd(nodes, 'rx_length', TX_NUM_SAMPS);      % Number of samples to receive
                    example_mode_string = 'hw';
                    
                else % SIMULATION
                    % Use sane defaults for hardware-dependent params in sim-only version
                    TX_NUM_SAMPS        = 2^15;        % 32768 samples
                    SAMP_FREQ           = 40e6;
                    example_mode_string = 'sim';
                end % setup warplab/sim
                %% Transmitter
                
                % Create an instance of the frequency impairment object
                PFO = comm.PhaseFrequencyOffset;
                PFO.SampleRate = SAMP_FREQ;
                PFO.PhaseOffset = 0;
                PFO.FrequencyOffsetSource = 'Input port';
                
                % FCS generator
                fcsGen = comm.CRCGenerator([32 26 23 22 16 12 11 10 8 7 5 4 2 1 0]);
                fcsGen.InitialConditions =1;
                fcsGen.DirectMethod = true;
                fcsGen.FinalXOR = 1;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Payload MPDU Generation
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%,
                % Generate a packet waveform
                txPSDU = randi([0 1],cfgHT.PSDULength*8,1); % PSDULength in bytes %8000x1
                %                    tx = wlanWaveformGenerator(txPSDU,cfgHT,cfgWaveGen);  % 25440 x 1
                [tx,tx_syms] = wlanWaveformGenerator_SURI(txPSDU,cfgHT,cfgWaveGen);
                
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % INTERPOLATION
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Pad with zeros for transmission
                [numSamps,~] = size(tx);
                tx_vec_padded = [tx; zeros((TX_NUM_SAMPS/INTERP_RATE)-numSamps , numtx )]; % 25440 x 1  to  32768 x 1
                
                [numSamps,~] = size(tx_vec_padded);
                if(INTERP_RATE == 1)
                    tx_vec_air = tx_vec_padded;
                elseif(INTERP_RATE == 2)
                    tx_vec_2x = zeros(2*numSamps, numtx);
                    tx_vec_2x(1:2:end,:) = tx_vec_padded;
                    tx_vec_air = filter(interp_filt2, 1, tx_vec_2x);
                end
                %             size(tx_vec_air) % 32768 x 1
                
                % Scale the Tx vector
                for j=1:numtx
                    tx_vec_air(:,j) = TX_SCALE .* tx_vec_air(:,j) ./ max(abs(tx_vec_air(:,j))); % 32768 x 1
                end
                
                %% WARPLab Tx/Rx
                if(USE_WARPLAB_TXRX)
                    
                    % Write the Tx waveform to the Tx node
                    switch cfgHT.NumTransmitAntennas
                        case 1
                            wl_basebandCmd(node_tx1,[RFA], 'write_IQ', tx_vec_air); %  : , 1
                            % Enable the Tx
                            wl_interfaceCmd(node_tx1,RFA, 'tx_en');
                            % Enable the Tx buffers
                            wl_basebandCmd(node_tx1,RFA, 'tx_buff_en');
                        case 2
                            wl_basebandCmd(node_tx1,[RFA], 'write_IQ', tx_vec_air(:,1));
                            wl_basebandCmd(node_tx2,[RFA], 'write_IQ', tx_vec_air(:,2));
                            % Enable the Tx
                            wl_interfaceCmd(node_tx1,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx2,RFA, 'tx_en');
                            % Enable the Tx buffers
                            wl_basebandCmd(node_tx1,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx2,RFA, 'tx_buff_en');
                        case 3
                            wl_basebandCmd(node_tx1,[RFA], 'write_IQ', tx_vec_air(:,1) );
                            wl_basebandCmd(node_tx2,[RFA], 'write_IQ', tx_vec_air(:,2));
                            wl_basebandCmd(node_tx3,[RFA], 'write_IQ', tx_vec_air(:,3));
                            % Enable the Tx
                            wl_interfaceCmd(node_tx1,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx2,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx3,RFA, 'tx_en');
                            % Enable the Tx buffers
                            wl_basebandCmd(node_tx1,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx2,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx3,RFA, 'tx_buff_en');
                        case 4
                            wl_basebandCmd(node_tx1,[RFA], 'write_IQ', tx_vec_air(:,1));
                            wl_basebandCmd(node_tx2,[RFA], 'write_IQ', tx_vec_air(:,2));
                            wl_basebandCmd(node_tx3,[RFA], 'write_IQ', tx_vec_air(:,3));
                            wl_basebandCmd(node_tx4,[RFA], 'write_IQ', tx_vec_air(:,4));
                            % Enable the Tx
                            wl_interfaceCmd(node_tx1,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx2,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx3,RFA, 'tx_en');
                            wl_interfaceCmd(node_tx4,RFA, 'tx_en');
                            % Enable the Tx buffers
                            wl_basebandCmd(node_tx1,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx2,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx3,RFA, 'tx_buff_en');
                            wl_basebandCmd(node_tx4,RFA, 'tx_buff_en');
                    end %switch tx antennas
                    switch NumReceiveAntennas
                        case 1
                            Rxarray=RFA;
                            Rxarray2=[RFA];
                        case 2
                            Rxarray=[RFA+RFB];
                            Rxarray2=[RFA,RFB];
                        case 3
                            Rxarray=[RFA+RFB+RFC];
                            Rxarray2=[RFA,RFB,RFC];
                        case 4
                            Rxarray=[RFA+RFB+RFC+RFD];
                            Rxarray2=[RFA,RFB,RFC,RFD];
                    end %switch rxantennas
                    % Enable the RX
                    wl_interfaceCmd(node_rx,Rxarray, 'rx_en');
                    %     wl_interfaceCmd(node_rx,RFA+RFB+RFC+RFD, 'rx_en');
                    % Enable the Rx buffers
                    wl_basebandCmd(node_rx,Rxarray, 'rx_buff_en');
                    %      wl_basebandCmd(node_rx,RFA+RFB+RFC+RFD, 'rx_buff_en');
                    
                    % Trigger the Tx/Rx cycle
                    eth_trig.send();
                    
                    % Pause for the samples to be processed at the node
                    pause(1.2 * TX_NUM_SAMPS * Ts);
                    
                    % Retrieve the received waveform from the Rx node
                    rx_vec_air   = wl_basebandCmd(node_rx, Rxarray2, 'read_IQ', 0, TX_NUM_SAMPS); % 32768 x 1
                    % rx_vec_air   = wl_basebandCmd(node_rx, [RFA,RFB,RFC,RFD], 'read_IQ', 0, TX_NUM_SAMPS);
                    
                    % Disable the Tx/Rx radios and buffers
                    wl_basebandCmd(nodes, 'RF_ALL', 'tx_rx_buff_dis');
                    wl_interfaceCmd(nodes, 'RF_ALL', 'tx_rx_dis');
                else
                    rx_vec_air =tx_vec_air; % 32768 x 1
                end % End tx/rx
                
                %% Receiver
                
                if plot_txrxair
                    [cf] = plot_txrx_vecair (tx_vec_air,rx_vec_air,cf,numtx, NumReceiveAntennas, tx);
                end
                size(rx_vec_air); % 32768 x 1
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % DECIMATE
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                for n= 1:NumReceiveAntennas
                    if(DECIMATE_RATE == 1)
                        raw_rx_dec_1(:,n) = rx_vec_air(:,n);  % 32768 x 1
                    elseif(DECIMATE_RATE == 2)
                        raw_rx_dec(:,n)   = filter(interp_filt2, 1, rx_vec_air(:,n));
                        raw_rx_dec_1(:,n) = raw_rx_dec(1:2:end,n);
                    end
                end
                
                rx = raw_rx_dec_1; %16384 x1
                size(rx); %16384 x1
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % LTS Correlation
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %pktStartIdx = packetDetect(rx,cfgHT.ChannelBandwidth); %ORIGINAL
                [pktStartIdx,SNR_W] = packetDetect1(rx,cfgHT.ChannelBandwidth,LTF_thresh); % Return SNR
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % NO LTS - failed packet
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if isempty(pktStartIdx) % If empty no L-STF detected; packet error
                    fprintf('\nNo LTS\n');
                    % Variables for Packet errors
                    BitErrors=cfgHT.PSDULength*8; % In case No LTS all bits are in error
                    BER = 100;
                    packetError = 1; % Was packet in error
                    NoLTS=1; % No LTS
                    chanEst=[]; % Channel Estimates
                    SNR_W= NaN;
                    rx_evm=[];
                    rx_syms=[];
                    
                    if RxStats
                        fprintf('\nBit Errors=%d\n',BitErrors);
                        fprintf('BER =%3.2f%c\n',BER,'%');
                        fprintf('Packet Error=%d\n',packetError);
                    end
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Store Values for Parsing / Plotting
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    A_BitErrors(k,imcs)=BitErrors; % Bit Errors
                    A_BER(k,imcs)= BER; % Bit Errors
                    A_PacketError(k,imcs) = packetError; % Was packet in error
                    A_LTSp(k,imcs)=NoLTS; % No LTS
                    
                    A_MCS{k,imcs} = MCSinfo; % MCS information (index, datarate, etc)
                    A_rx_H_est_Results{k,imcs}=chanEst; % Channel Estimates
                    A_SNR_W{k,imcs}=SNR_W;
                    A_rx_evm{k,imcs} = rx_evm;
                    
                    %                 A_tx_vec_air{k,imcs}=tx_vec_air; %Tx vector
                    %                 A_rx_vec_air{k,imcs}=rx_vec_air; % Rx vector
                    A_rx_syms{k,imcs}=rx_syms; % RX symbols
                    A_tx_syms{k,imcs}=tx_syms; % Tx symbols
                    
                    A_TXgains{k,imcs}=tx_gain_sel; % Selected Tx gains
                    A_RXgains{k,imcs}=rx_gain_sel; % Selected RX gains
                    continue
                    
                end
                
                % Packet offset from start of waveform
                pktOffset = pktStartIdx-1;
                
                % Indices for accessing each field within the time-domain packet
                ind = wlanFieldIndices(cfgHT);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Coarse Frequency Offset Correction
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Extract L-STF and perform coarse frequency offset correction
                lstf = rx(pktOffset+(ind.LSTF(1):ind.LSTF(2)),:);
                coarseFreqOff = wlanCoarseCFOEstimate(lstf,cfgHT.ChannelBandwidth);
                rx = step(PFO,rx,-coarseFreqOff);
                release(PFO); % Release object for subsequent processing
                
                % Extract the Non-HT fields and determine start of L-LTF
                nonhtfields = rx(pktOffset+(ind.LSTF(1):ind.LSIG(2)),:);
                lltfIdx = symbolTiming(nonhtfields,cfgHT.ChannelBandwidth,LTF_thresh);
                
                % Synchronize the received waveform given the offset between the
                % expected start of the L-LTF and actual start of L-LTF
                pktOffset = pktOffset+lltfIdx-double(ind.LLTF(1));
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % NO LTS/ Bad Timing - failed packet
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % If no L-LTF detected or if packet detected outwith the range of
                % expected delays from the channel modeling; packet error
                if isempty(lltfIdx) || pktOffset<0
                    fprintf('\nOffset is Negative\n');
                    
                    % Variables for Packet errors
                    BitErrors=cfgHT.PSDULength*8; % In case No LTS all bits are in error
                    BER = 100;
                    packetError = 1; % Was packet in error
                    NoLTS=1; % No LTS
                    chanEst=[]; % Channel Estimates
                    SNR_W= NaN;
                    rx_evm=[];
                    rx_syms=[];
                    
                    if RxStats
                        fprintf('\nBit Errors=%d\n',BitErrors);
                        fprintf('BER =%3.2f%c\n',BER,'%');
                        fprintf('Packet Error=%d\n',packetError);
                    end
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Store Values for Parsing / Plotting
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    A_BitErrors(k,imcs)=BitErrors; % Bit Errors
                    A_BER(k,imcs)= BER; % Bit Errors
                    A_PacketError(k,imcs) = packetError; % Was packet in error
                    A_LTSp(k,imcs)=NoLTS; % No LTS
                    
                    A_MCS{k,imcs} = MCSinfo; % MCS information (index, datarate, etc)
                    A_rx_H_est_Results{k,imcs}=chanEst; % Channel Estimates
                    A_SNR_W{k,imcs}=SNR_W;
                    A_rx_evm{k,imcs} = rx_evm;
                    
                    %                 A_tx_vec_air{k,imcs}=tx_vec_air; %Tx vector
                    %                 A_rx_vec_air{k,imcs}=rx_vec_air; % Rx vector
                    A_rx_syms{k,imcs}=rx_syms; % RX symbols
                    A_tx_syms{k,imcs}=tx_syms; % Tx symbols
                    
                    A_TXgains{k,imcs}=tx_gain_sel; % Selected Tx gains
                    A_RXgains{k,imcs}=rx_gain_sel; % Selected RX gains
                    continue 
                end
                
                %Update Rx with offset
                rx = rx(1+pktOffset:end,:);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Fine Frequency Offset Correction
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Extract L-LTF and perform fine frequency offset correction
                lltf = rx(ind.LLTF(1):ind.LLTF(2),:);
                fineFreqOff = wlanFineCFOEstimate(lltf,cfgHT.ChannelBandwidth);
                rx = step(PFO,rx,-fineFreqOff);
                release(PFO); % Release object for subsequent processing
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Channel Estimation
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Extract HT-LTF samples from the waveform, demodulate and perform
                % channel estimation
                htltf = rx(ind.HTLTF(1):ind.HTLTF(2),:);
                htltfDemod = wlanHTLTFDemodulate(htltf,cfgHT);
                chanEst = wlanHTLTFChannelEstimate(htltfDemod,cfgHT);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Recover PSDU
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Recover the transmitted PSDU in HT Data
                % Extract HT Data samples from the waveform and recover the PSDU
                htdata = rx(ind.HTData(1):ind.HTData(2),:);
                % Equalization method
                cfgRec = wlanRecoveryConfig('EqualizationMethod', 'ZF');
                % Noise Variance
                nVar=0;
                
                %             rxPSDU = wlanHTDataRecover(htdata,chanEst,nVar,cfgHT);
                [rxPSDU,eqDataSym] = wlanHTDataRecover(htdata,chanEst,nVar,cfgHT);
                rx_syms= eqDataSym; % Rx Symbols % 52 x 2/4/6/8 x Nss
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % RX STATS
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % Determine if any bits are in error, i.e. a packet error
                packetError = double(any(biterr(txPSDU,rxPSDU)));
                BitErrors = biterr(txPSDU,rxPSDU);
                BER = (BitErrors/(cfgHT.PSDULength*8))*100; % FIXME: CHECK IT!
                
                for jj=1:cfgHT.NumTransmitAntennas
                    rx_syms_temp=rx_syms(:,:,jj);
                    tx_syms_temp=tx_syms(:,:,jj);
                    rx_syms_reshaped = reshape(rx_syms_temp, 1, 52*size(rx_syms_temp,2)); % all need to be in a single array
                    tx_syms_reshaped = reshape(tx_syms_temp, 1, 52*size(tx_syms_temp,2));
                    rx_evm(jj)= sqrt(sum((real(rx_syms_reshaped) - real(tx_syms_reshaped)).^2 + (imag(rx_syms_reshaped) - imag(tx_syms_reshaped)).^2)/(52 * length(tx_syms)));
                    fprintf('EVM %d: %1.3f%%\n', jj, rx_evm(jj)*100);
                end
                
                
                if RxStats
                    fprintf('\nBit Errors=%d',BitErrors);
                    fprintf('\nBER =%3.2f%c\n',BER,'%');
                    fprintf('Packet Error=%d\n',packetError);
                end
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Constellation
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Show Constallation Plot
                if ConstPlot
                    [ cf ] = plot_SymConst(numtx , cf, rx_syms, MCSinfo,tx_syms);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  pause(1)
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Store Values for Parsing / Plotting
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                A_BitErrors(k,imcs)=BitErrors; % Bit Errors
                A_BER(k,imcs)= BER; % Bit Errors
                A_PacketError(k,imcs) = packetError; % Was packet in error
                A_LTSp(k,imcs)=NoLTS; % No LTS
                
                A_MCS{k,imcs} = MCSinfo; % MCS information (index, datarate, etc)
                A_rx_H_est_Results{k,imcs}=chanEst; % Channel Estimates
                A_SNR_W{k,imcs}=SNR_W;
                A_rx_evm{k,imcs} = rx_evm;
                
                %             A_tx_vec_air{k,imcs}=tx_vec_air; %Tx vector
                %             A_rx_vec_air{k,imcs}=rx_vec_air; % Rx vector
                A_rx_syms{k,imcs}=rx_syms; % RX symbols
                A_tx_syms{k,imcs}=tx_syms; % Tx symbols
                
                A_TXgains{k,imcs}=tx_gain_sel; % Selected Tx gains
                A_RXgains{k,imcs}=rx_gain_sel; % Selected RX gains
            catch
                fprintf('\n====Failed Iteration: %d =====\n', k);
                failedit(k,imcs)=1;
                continue
            end
            
        end % Number of iteratinon (k)
        
        %SAVE FILE after each MCS
        if savefile
            try
                save(filename)
            catch
                fprintf('\n====Failed Save=====\n');
                continue
            end
        end
    end % mcs (imcs)
    %             %SAVE FILE
    %         if savefile
    %             save(filename, '-v7.3')
    %         end
    
end % Loop num TX