clear
clc
close all
tail = 70;
color_end = [hex2dec('ff')/255, hex2dec('00')/255, hex2dec('ff')/255];
color_start = [hex2dec('00')/255, hex2dec('00')/255, hex2dec('ff')/255];
tail_color(:,1) = linspace(color_start(1), color_end(1), tail);
tail_color(:,2) = linspace(color_start(2), color_end(2), tail);
tail_color(:,3) = linspace(color_start(3), color_end(3), tail);
marker = ['o', 's'];
x = linspace(0,0.5,100);
y = linspace(0,3.5,tail);
y = repmat(y', 1, 100);
figure(1)
hold on

for i = 1:70
    plot(x,y(i,:),'marker', 's','markeredgecolor', tail_color(i,:),'markerfacecolor', tail_color(i,:),'markersize',10);
end
axis equal
axis([0 0.5 0 3.5]);
set(gca, 'xtick', []);
% set(gca, 'ytick', []);
ylabel('time [s]','Fontname', 'Times New Roman','FontSize',12);