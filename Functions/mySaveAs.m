function [] = mySaveAs(hand, savepath, width, height)

% PATH SHOULD NOT HAVE FILE EXTENSION!!

set(hand, 'PaperUnits', 'inches');
set(hand, 'Units', 'inches');
set(hand, 'PaperSize', [width height]);
set(hand, 'Position',[0 0 width height])
set(hand, 'PaperPosition',[0 0 width height]);


epspath = [savepath '.eps'];
%pdfpath = [savepath '.pdf'];

 saveas(hand,[savepath '.eps'], 'epsc2') ;
 system(['epstopdf ' epspath]);

% [status, result] = system(cmd);
 %disp(['i_view32 ' epspath ' /convert=' pdfpath])

%saveas(hand,[savepath '.emf'], 'emf') ;
 
end