function nPlotVertLines(timeLines,timeLables)
fig=gcf; hold on 
fig.Children.XGrid='off';
for i =1 : length(timeLines)-1
    plot(timeLines(i)*[1 1],fig.Children.YLim,'LineWidth',0.5,'LineStyle',':','Tag','test');
    
    if (timeLines(i+1)-timeLines(i))> (fig.Children.XLim(2) - fig.Children.XLim(1))/1000
        %two small
        textSuffix='';
        textXposOff=0;
    else
        textSuffix='><';
        textXposOff=-(fig.Children.XLim(2) - fig.Children.XLim(1))/100;
    end
    
    if mod(i,2)>0
        textYpos=fig.Children.YLim*[0.1;0.9];
    else
        textYpos=fig.Children.YLim*[0.9;0.1];
    end
    textXpos = (timeLines(i)+timeLines(i+1))/2;
    text(textXpos,textYpos, [timeLables(i) textSuffix],'HorizontalAlignment','center');
end                
hold off