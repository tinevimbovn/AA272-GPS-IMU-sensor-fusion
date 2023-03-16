clc
clear
close all

c =299792458;

% Setting the directory and extracting ephem data
dirName = 'C:\Users\tinev\OneDrive\Documents\Global Positioning Systems\Final Project';
dataFilter = SetDataFilter;
fileName = 'gnss_log_raw_data_2.txt';
[gnssRaw,gnssAnalysis] = ReadGnssLogger(dirName,fileName,dataFilter);
gnssMeas = ProcessGnssMeas(gnssRaw);

fctSeconds = 1e-3*double(gnssRaw.allRxMillis(end));
utcTime = Gps2Utc([],fctSeconds);
allGpsEph = GetNasaHourlyEphemeris(utcTime,dirName);
[gpsEph,iSv] = ClosestGpsEph(allGpsEph,gnssMeas.Svid,fctSeconds);
svid = gnssMeas.Svid;
tS = gnssMeas.tRxSeconds - (gnssMeas.tRxSeconds/c);
gpsWeek = floor(-1*(double(gnssRaw.FullBiasNanos))/604800e9);


%Calculating X,Y,Z coordinates of satellites in ECEF and B
for i = 1:length(gnssMeas.tRxSeconds)
    [DtsvS] = GpsEph2Dtsv(gpsEph,tS(i,:)); %clock bias
    ttxSec = gnssMeas.tRxSeconds - gnssMeas.PrM/c - DtsvS;
    gpsTime = [gpsWeek(1:length(svid)), ttxSec(i,:)'];
    [xyzM,dtsvS] = GpsEph2Xyz(gpsEph,gpsTime);
    xyzM = rmmissing(xyzM);
    dtsvS = rmmissing(dtsvS);
    B = dtsvS.*c;
    prange = rmmissing(gnssMeas.PrM(i,:))';
    xyzBp_array{i} = [xyzM B prange];
end

x0 = [-2705053; -4260114; 3887314];

% Calculating X,Y,Z pos from satellite data
for i = 1:length(gnssMeas.tRxSeconds)
    h = xyzBp_array{i};
    X = h(:, 1);
    Y = h(:, 2);
    Z = h(:, 3);
    B = h(:, 4);
    prange = h(:, 5);

    [x_new, b_new] = solve_pos(x0, X, Y, Z, B, prange);
    x0 = x_new;
    bu0 = b_new;

    plot_X(i, :) = x0(1);
    plot_Y(i, :) = x0(2);
    plot_Z(i, :) = x0(3);
    bu0_store(i, 1) = bu0;

end

% Converting ECEF to ENU
wgs84 = wgs84Ellipsoid('meter');
lat0 = 37.795204920476706;
lon0 = -122.40051765050474;
h0 = 0;

figure(1)
[xEast,yNorth,zUp] = ecef2enu(plot_X,plot_Y,plot_Z,lat0,lon0,h0,wgs84);
plot3(xEast,yNorth,zUp, LineWidth=1)
lstr = {'FontName','Times New Roman','FontSize',16};
tstr = {'FontName','Times New Roman','FontSize',18};
title('Trajectory ENU', tstr{:})
xlabel('X [m]', lstr{:})
ylabel('Y [m]', lstr{:})
zlabel('Z [m]', lstr{:})
grid on
grid minor
saveas(gcf,'GPS trajectory enu 3D.png')



figure(2)
plot(1:length(gnssMeas.tRxSeconds), xEast, LineWidth=1)
hold on
plot(1:length(gnssMeas.tRxSeconds), yNorth, LineWidth=1)
plot(1:length(gnssMeas.tRxSeconds), zUp, LineWidth=1)
legend('East', 'North', 'Up', 'Location','southwest', lstr{:})
xlabel('Time [s]', lstr{:})
ylabel('Distance [m]', lstr{:})
title('Position', tstr{:} )
grid on
grid minor
saveas(gcf,'GPS trajectory enu vs time.png')

figure(3)
plot(xEast,yNorth, LineWidth=1)
lstr = {'FontName','Times New Roman','FontSize',16};
tstr = {'FontName','Times New Roman','FontSize',18};
title('Trajectory ENU', tstr{:})
xlabel('X [m]', lstr{:})
ylabel('Y [m]', lstr{:})
grid on
grid minor
saveas(gcf,'GPS trajectory enu 2D.png')
