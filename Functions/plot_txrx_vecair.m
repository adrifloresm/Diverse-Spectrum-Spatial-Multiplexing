function [cf] = plot_txrx_vecair (tx_vec_air,rx_vec_air,cf,numtx, NumReceiveAntennas,tx)

cf = cf + 1;
figure(cf); clf;
for nt=1:numtx
    %Tx vector air
    subplot(1,numtx,nt)
    plot(real(tx_vec_air(:,nt)));
    title(['Tx vec air Tx: ' num2str(nt)]);
    grid on
    ylim([-1 1])
    xlim([0 size(tx,1)*2+1000])
    
end
cf = cf + 1;
figure(cf); clf;
for nr=1:NumReceiveAntennas
    %Rx vector air
    subplot(1,NumReceiveAntennas,nr)
    plot(real(rx_vec_air(:,nr)));title(['Rx vec air Rx: ' num2str(nr)]);
        grid on
    ylim([-1 1])
    xlim([0 size(tx,1)*2+1000])
end

end