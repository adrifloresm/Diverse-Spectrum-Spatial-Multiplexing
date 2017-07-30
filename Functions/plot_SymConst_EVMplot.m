function [ cf ] = plot_SymConst_EVMplot(numtx , cf, rx_sys_mat,  tx_syms)


cf = cf + 1;
figure(cf); clf;
for idx=1:numtx
    %========= Symbol constellation ========================
    % ZF-----------------------------------------------------
    subplot(numtx,1,idx)
    clear payload_tmp tx_syms_tmp
    payload_tmp = squeeze(rx_sys_mat(:,:,idx));
    tx_syms_tmp = squeeze(tx_syms(:,:,idx));
    
    plot(payload_tmp(:),'r.'); % Rx symbols
    axis square; axis(1.5*[-1 1 -1 1]);
    grid on;
    hold on;
    
%     if MCSinfo.MCS_Index == 0 || MCSinfo.MCS_Index ==  8 || MCSinfo.MCS_Index ==  16 || MCSinfo.MCS_Index ==  24
%         plot(complex(tx_syms_tmp(:),0),'bo');
%     else 
        plot(tx_syms_tmp(:),'bo');
%     end
    
%     tt2=sprintf('Nss=%d, %s, Data rate: %d Mbps',MCSinfo.Nss, MCSinfo.ModText,MCSinfo.Datarate/1e6);

     title('Nss=2, ')
end

end

