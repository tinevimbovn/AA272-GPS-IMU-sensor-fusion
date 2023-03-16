clc;
clear;
close all;

%%
data = readtable("IMU Raw Data 2.xls");
x = data.xpos;
y = data.ypos;
z = data.zpos;

pos = [x, y, z];

orx = data.GyroscopeX_rad_s_;
ory = data.GyroscopeY_rad_s_;
orz = data.GyroscopeZ_rad_s_;

orientation = [orx, ory, orz];

lat = 37.79 * pi/180;
lon = -122.4 * pi/180;

R = [ -sin(lon), cos(lon), 0;
       -cos(lon)*sin(lat), -sin(lon)*sin(lat), cos(lat);
        cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat)];

for i = 1:length(pos)
    posvector = transpose(pos(i,:));
    orientationvector = transpose(orientation(i,:));

    ECEFpos(i,:) = R * posvector;
    ECEFor(i,:) = R * orientationvector;

end

ECEFposx = ECEFpos(:,1);
ECEFposy = ECEFpos(:,2);
ECEFposz = ECEFpos(:,3);

ECEForx = ECEFor(:,1);
ECEFory = ECEFor(:,2);
ECEForz = ECEFor(:,3);

wgs84 = wgs84Ellipsoid('meter');

[ENUx, ENUy, ENUz] = ecef2enu(ECEFposx, ECEFposy, ECEFposz, -122.4, 37.79, 0, wgs84);

plot3(ENUx(:,1), ENUy(:,1), ENUz(:,1),'LineWidth',2);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

