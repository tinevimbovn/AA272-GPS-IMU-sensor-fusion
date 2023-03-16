clc
clear
close all


data = readtable('LatLong.xlsx');
lat = data.LatitudeDegrees;
lon = data.LongitudeDegrees;
geobasemap streets
geoplot(lat,lon, 'LineWidth',2)
hold on 
geoplot(lat(1), lon(1),'o', MarkerFaceColor='g' , MarkerSize=10)
lstr = {'FontName','Times New Roman','FontSize',16};
tstr = {'FontName','Times New Roman','FontSize',18};
legend('Trajectory', 'Start position')
title('Trajectory Given by GNSS Logger', tstr{:})
saveas(gcf,'gnss logger trajectory.png')