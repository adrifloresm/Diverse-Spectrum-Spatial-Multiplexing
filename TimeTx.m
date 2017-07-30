% Copyright Adriana Flores
% Top Function file for Performance Evaluation: Total Air time

close all
clear
clc

% Rates 60
ControlPHY=[27.5]; % Mbps
SCrates=[385 770 962.5 1155 1251.25 1540 1925 2310 2502.5 3080 3850 4620]; % Mbps
OFDMrates=[693.00 866.25 1386.00 1732.50 2079.00 2772.00 3465.00 4158.00 4504.50 5197.50 6237.00 6756.75]; % OFDM 60 rates Mbps
Rates60=[ControlPHY SCrates OFDMrates]; % ALL RATES 802.11ad
Rates60=sort(Rates60);

% Rates 2.4
Rates11n_20_800=[6.5 13 19.5 26 39 52 58.5 65]; % SISO RATES 11n 20 Mhz 800ns
Rates11n_40_800=[13.5 27 40.5 54 81 108 121.5 135]; % SISO RATES 11n 20 Mhz 400ns
Rates11n_20_400=[7.2 14.4 21.7 28.9 43.3 57.8 65 72.2]; % SISO RATES 11n 40 Mhz 800ns
Rates11n_40_400=[15 30 45 60 90 120 135 150]; % SISO RATES 11n 40 Mhz 400ns

% Rates11n1SS=[Rates11n_20_800 Rates11n_40_800 Rates11n_20_400 Rates11n_40_400]; % all rates 802.11n 1SS
Rates11n1SS=[Rates11n_20_800]; % all rates 802.11n 1SS

Rates11n1SS=sort(Rates11n1SS);


% packet size
X = 1000*8 ;%packet size in bits

%Knobs for lines
% M= 4; % num tx

for M=2:4
	for DR24ind = 1:length(Rates11n1SS) % 2.4 Rate
		for DR60ind=1:length(Rates60) % 60 Rate
			
			DR60 = Rates60(DR60ind)*1e6; % bits per sec
			DR24 = Rates11n1SS(DR24ind)*1e6; %bits per sec
			
			t_today(DR60ind,DR24ind)=X/DR24; % seconds
			t_DSSM(DR60ind,DR24ind)= ((M-1)*(X/DR60)) + ((X/M)/DR24); % seconds
			Ratio_Rates(DR60ind,DR24ind)=DR24/DR60; % ratio 2.4 /60
			
		end%60 rates
	end %2.4 rates

	% Flatten arrays 
	Ratio_all=reshape(Ratio_Rates, [size(Ratio_Rates,1)*size(Ratio_Rates,2)],1); % reshape to 1 x all array
	t_today_all=reshape(t_today, [size(t_today,1)*size(t_today,2)],1); % reshape to 1 x all array
	t_DSSM_all=reshape(t_DSSM, [size(t_DSSM,1)*size(t_DSSM,2)],1); % reshape to 1 x all array

	% Join all data to sort by Ratio
	ALL_Data=[Ratio_all t_today_all t_DSSM_all]; % Ratio %today %cham 3 columns
	Sorted_ALL_Data{M}=sort(ALL_Data,1);

	%all saved as array
	t_today_allMat{M}=t_today./1e-3;
	t_DSSM_allMat{M}=t_DSSM./1e-3;
end % Num Tx

colororder = [
    0.00  0.00  1.00
    0.00  0.50  0.00
    1.00  0.00  0.00
    0.00  0.75  0.75
    0.75  0.00  0.75
    0.75  0.75  0.00
    0.25  0.25  0.25
    0.75  0.25  0.25
    0.95  0.95  0.00
    0.25  0.25  0.75
    0.75  0.75  0.75
    0.00  1.00  0.00
    0.76  0.57  0.17
    0.54  0.63  0.22
    0.34  0.57  0.92
    1.00  0.10  0.60
    0.88  0.75  0.73
    0.10  0.49  0.47
    0.66  0.34  0.65
    0.99  0.41  0.23
    ];

figure(2); clf;
hold on
legstr{1}=['SISO TX time'];
Xdata=Sorted_ALL_Data{2}(:,1);
Y24=Sorted_ALL_Data{2}(:,2)./1e-3;
plot(Xdata,Y24,'v-','LineWidth',1,'Color', colororder(1,:))

for M=2:4
Xdata=Sorted_ALL_Data{M}(:,1);
Y60=Sorted_ALL_Data{M}(:,3)./1e-3;
plot(Xdata,Y60,'*-','LineWidth',1, 'Color', colororder(M,:))
legstr{M}=['Chameleon Tx time ' num2str(M) ' Transmitters'];
end

xlabel('Ratio Data rates (2.4 DR / 60 DR)')
ylabel('Tx time (ms)')
grid on
xlim([0 1])
legend(legstr,'Location','Best')


% PLOT FOR PAPER
TxTime=figure(1); clf;
%WORST 2.4
ax2=subplot(1,2,1)
sel24=1;
hold on
plot(Rates60',t_today_allMat{2}(:,sel24),'-.','Color', colororder(1,:))
for M=2:4
plot(Rates60, t_DSSM_allMat{M}(:,sel24),'*-','Color', colororder(M,:))
end
% ylim([0 max(t_today_allMat{2}(:,sel24))+0.1])
ylim([0 1.4])
xlim([0 7000])
xlabel('60 GHz Data Rate (Mbps)''FontSize',14)
ylabel('Tx time (ms)''FontSize',14)
legend(legstr,'Location','Northeast''FontSize',14)
title(['Worst 2.4 GHz SISO Rate ' num2str(Rates11n1SS(sel24)) ' Mbps'])
grid on

%BEST
ax1=subplot(1,2,2)
sel24=length(Rates11n1SS);
hold on
plot(Rates60',t_today_allMat{2}(:,sel24),'-.','Color', colororder(1,:))
for M=2:4
plot(Rates60, t_DSSM_allMat{M}(:,sel24),'*-','Color', colororder(M,:))
end
xlabel('60 GHz Data Rate (Mbps)','FontSize',14)
ylabel('Tx time (ms)')
legend(legstr,'Location','Northeast','FontSize',14)
grid on
title(['Best 2.4 GHz SISO Rate ' num2str(Rates11n1SS(sel24)) ' Mbps'])
linkaxes([ax2,ax1],'xy')

% mySaveAs(TxTime,'Plots/TxTime',10,7);
