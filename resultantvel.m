function velres = resultantvel

data = readtable('IMU Raw Data 2.xls');
accel_x = data.LinearAccelerationX_m_s_2_;
accel_y = data.LinearAccelerationY_m_s_2_;
accel_z = data.LinearAccelerationZ_m_s_2_;

% ypos = data.ypos;
time = data.Time_s_;

%%
% *Integrate acceleration to get velocity:*
vel_x = cumtrapz(time, accel_x);
vel_y = cumtrapz(time, accel_y);
vel_z = cumtrapz(time, accel_z);

vel = [vel_x, vel_y, vel_z];

lat = 37.79 * pi/180;
lon = -122.4 * pi/180;

R = [ -sin(lon), cos(lon), 0;
       -cos(lon)*sin(lat), -sin(lon)*sin(lat), cos(lat);
        cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat)];

for i = 1:length(vel_x)
    velvector = vel(i,:);

    ECEFvel(i,:) = R * transpose(velvector);

end

ECEFvelx = ECEFvel(:,1);
ECEFvely = ECEFvel(:,2);
ECEFvelz = ECEFvel(:,3);

wgs84 = wgs84Ellipsoid('meter');

[ENUvelx, ENUvely, ENUvelz] = ecef2enu(ECEFvelx, ECEFvely, ECEFvelz, -122.4, 37.79, 0, wgs84);


velres = sqrt(ECEFvelx.^2 + ECEFvely.^2 + ECEFvelz.^2);

end