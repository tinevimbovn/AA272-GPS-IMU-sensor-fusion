clc
clear
close all

% earthpos = readtable("earthpos.csv");
% load('removedups.mat');
% ephemfile = readtable('ephem_data.csv'); 
% gnss_log_raw_data_2 = readtable('gnss_log_raw_data_2.csv');
% data = outerjoin(ephemfile,gnss_log_raw_data_2,"LeftKey","Svid","RightKey","Svid");
% data = rmmissing(data);
% data = sortrows(data, 'Indexnumber');

% Calculating new satellite XYZ:
% tof = data.ReceivedSvTimeNanos/1e9 - data.Toc;
% earthrate = (2*pi)/86400;
% R = rotz(earthrate.*tof);
% 
% X = R.*data.X;
% Y = R.*data.Y;
% Z = R.*data.Z;
% 
% 
% X = data.X;
% Y = data.Y;
% Z = data.Z;
% 
% timesteps = removedups(:,42);
% [G, groupID] = findgroups(timesteps);
% n_groups = numel(groupID);
% new_t = cell(1, n_groups);
% 
% for ii = 1:n_groups
%     new_t{ii} = removedups(G == ii,:);
% end
% 
% earthpos.Properties.VariableNames{1} = 'time';
% earthpos.Properties.VariableNames{2} = 'x';
% earthpos.Properties.VariableNames{3} = 'y';
% earthpos.Properties.VariableNames{4} = 'z';
% earthpos.Properties.VariableNames{5} = 'b';
% 
% xecef = earthpos.x;
% yecef = earthpos.y;
% zecef = earthpos.z;
% 
% % ADDED:
% wgs84 = wgs84Ellipsoid('meter');
% [x, y, z] = ecef2enu(xecef, yecef, zecef, -122.4, 37.79, 0, wgs84);

data = readtable('GPSdata_enu.csv');

x = data.xEast;
y = data.yNorth;
z = data.zUp;

P0 = diag([25, 25, 1]);

% [x,y,theta]
% mu_curr = [x(1); y(1); z(1); atan2(y(1),x(1))];
mu_curr = [x(1); y(1); atan2(y(1),x(1))];

% x0  = normrnd(mu_curr(1), sqrt(P0(1,1)));
% y0 = normrnd(mu_curr(2), sqrt(P0(2,2)));
% theta0 = normrnd(mu_curr(3), sqrt(P0(3,3)));
% mu_curr = [x0; y0; theta0];
% 
%u_curr = [st; alphat]
Q = diag([0.1, 0.1, 0.1]);
diff_freq = 211; % IMU data sampling was 212Hz and GPS is 10Hz
true = [0;0;0];
true_store(:,1) = true;
Pt = P0;
mu(:, 1) = mu_curr;

for i = 1:length(x)-1 % Loop through the GPS data (positions)
%     h = new_t{i};
%     x = h(:, 38); %x
%     y = h(:, 39); %y
%     z = h(:, 40); %z
%     pos = zeros(length(x),2);
    
    pos = zeros(length(x),2);
    for j = 1:length(x)
        pos(j, 1) = x(j,1); % Filling the positions matrix
        pos(j, 2) = y(j,1);
    end
    %mu_curr = [x(i*diff_freq);y(1);atan2(y(1),x(1))];

    velres = resultantvel;
    start = (i-1)*211 + 1;
    for k = start:start + diff_freq 
        u_curr = [velres(k); 0];
        [mu_t, Ft, Pt_t] = ekf_predict(mu_curr, u_curr, Q, Pt);
        mu_curr = mu_t;
        Pt = Pt_t;
    end

    R = 10*eye(length(x));
    
    
    [Pt_t,mut_t] = ekf_update(true, mu_t, Pt,R, pos);

    mu(:, i) = mut_t;
    mu_curr = mut_t;
    Pt = Pt_t;


    f = meas_mdl(true, pos);
%     true = f;
%     true_store(:,i) = true;
        

        
        
        
        
%         for k = 2:100
%             [mu_t, Ft, Pt] = ekf_predict(mu_curr, u_curr, Q, Pt);
%             [Pt_t,mut_t] = ekf_update(true, mu_t, Pt,R);
% 
%             mu(:,k) = mut_t;
%             mu_curr = mut_t;
%             Pt = Pt_t;
% 
% 
%             f = simulate_noisy_state_trans(true, u_curr);
%             true = f;
%             true_store(:,k) = true;
%         end
        
        
end
    
       
        
 

%Pt = P0;
% true = [0;0;0];
% true_store(:,1) = true;
% mu(:, 1) = mu_curr;
% 
% for i=2:100
%     [mu_t, Ft, Pt] = ekf_predict(mu_curr, u_curr, Q, Pt);
%     [Pt_t,mut_t] = ekf_update(true, mu_t, Pt,R);
% 
%     mu(:,i) = mut_t;
%     mu_curr = mut_t;
%     Pt = Pt_t;
% 
%     
%     f = simulate_noisy_state_trans(true, u_curr);
%     true = f;
%     true_store(:,i) = true;
% end
% 
% 
% figure(3)
% plot(true_store(1,:),true_store(2,:))
% hold on
% plot(mu(1,:), mu(2,:))
% plot(0,0,'.','MarkerSize',10, Color=[0, 0.4470, 0.7410])
% plot(x0,y0, '.','MarkerSize',10, Color=[0.8500, 0.3250, 0.0980])
% plot(pos(1,1),pos(1,2),'.','MarkerSize',10)
% plot(pos(2,1),pos(2,2),'.','MarkerSize',10)
% plot(pos(3,1),pos(3,2),'.','MarkerSize',10)
% plot(pos(4,1),pos(4,2),'.','MarkerSize',10)
% legend('True trajectory', 'Estimated trajectory','true starting point', ...
%     'estimated starting point', 'Beacon 1', 'Beacon 2', 'Beacon 3',...
%     'Beacon 4', 'Location', 'southwest', 'NumColumns',2)
% xlabel('x (m)')
% ylabel('y (m)')
% title('Estimated state trajectory and True state trajectory' )
% grid on
% grid minor
% saveas(gcf,'PS4Q3g.png')

lstr = {'FontName','Times New Roman','FontSize',16};
tstr = {'FontName','Times New Roman','FontSize',18};

posres = sqrt(mu(1,:).^2 + mu(2,:).^2 + mu(3,:).^2);
figure(1)
plot3(mu(1,:),mu(2,:),mu(3,:),'LineWidth',2)
box on
grid on
grid minor
xlabel('X [m]', lstr{:});
ylabel('Y [m]', lstr{:});
zlabel('Z [m]', lstr{:});
title(['Trajectory using diag(Q) = [',sprintf('%.2f ',diag(Q)),']'],tstr{:});