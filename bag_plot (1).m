clear all, close all, clc

bag = rosbag('2021-01-27-10-06-39.bag');
rosbag info '2021-01-27-10-06-39.bag'

% % EKF
b_EKF = select(bag,'Topic','/ekf');
msgStructsEKF = readMessages(b_EKF,'DataFormat','struct');
msgStructsEKF{1}
x_EKF = cellfun(@(m) double(m.Linear.X),msgStructsEKF);
y_EKF = cellfun(@(m) double(m.Linear.Y),msgStructsEKF);

% Mavros
b_Mavros = select(bag,'Topic','/mavros/vision_pose/pose');
msgStructsMavros = readMessages(b_Mavros,'DataFormat','struct');
msgStructsMavros{1} 
x_Mavros = cellfun(@(m) double(m.Pose.Position.X),msgStructsMavros);
y_Mavros = cellfun(@(m) double(m.Pose.Position.Y),msgStructsMavros);

% % QR det
b_QR_det = select(bag,'Topic','/qr_det');
msgStructsMQR_det = readMessages(b_QR_det,'DataFormat','struct');
msgStructsMQR_det{1}
x_QR_det = cellfun(@(m) double(m.Linear.X),msgStructsMQR_det);
y_QR_det = cellfun(@(m) double(m.Linear.Y),msgStructsMQR_det);


% % % QR raw
bQR_raw = select(bag,'Topic','/qr_raw');
msgStructsMQR_raw = readMessages(bQR_raw,'DataFormat','struct');
msgStructsMQR_raw{1}
x_QR_raw = cellfun(@(m) double(m.Linear.X),msgStructsMQR_raw);
y_QR_raw = cellfun(@(m) double(m.Linear.Y),msgStructsMQR_raw);

% % vrpn
% b_vrpn = select(bag,'Topic','/vrpn_client_node/SDP2020/pose');
% msgStructs_vrpn = readMessages(b_vrpn,'DataFormat','struct');
% msgStructs_vrpn{1}
% x_vrpn = cellfun(@(m) double(m.Pose.Position.X),msgStructs_vrpn);
% y_vrpn = cellfun(@(m) double(m.Pose.Position.Y),msgStructs_vrpn);

% UWB
buwb = select(bag,'Topic','/uwb');
msgStructs_uwb = readMessages(buwb,'DataFormat','struct');
msgStructs_uwb{1}
d_uwb = cellfun(@(m) double(m.Data),msgStructs_uwb);
% plot(d_uwb)

% hold on
% x_anchor = -1.63
% y_anchor = 0.77
% d=zeros(368,1)
% for i=1:368
%     d(i,1) = sqrt(power(x_anchor-x_vrpn(i*27,1),2)+ power(y_anchor-y_vrpn(27*i,1),2))
% end   
% plot(d)

% Vicon
b_vicon = select(bag,'Topic','/p_vicon');
msgStructsp_vicon = readMessages(b_vicon,'DataFormat','struct');
msgStructsp_vicon{1}
x_vicon = cellfun(@(m) double(m.Linear.X),msgStructsp_vicon);
y_vicon = cellfun(@(m) double(m.Linear.Y),msgStructsp_vicon);

% P_mav
b_P_mav = select(bag,'Topic','/p_mav');
msgStructsp_P_mav = readMessages(b_P_mav,'DataFormat','struct');
msgStructsp_P_mav{1}
x_P_mav = cellfun(@(m) double(m.Linear.X),msgStructsp_P_mav);
y_P_mav = cellfun(@(m) double(m.Linear.Y),msgStructsp_P_mav);

b_meas_mode  = select(bag,'Topic','/meas_mode');
msgStructsp_meas_mode  = readMessages(b_meas_mode,'DataFormat','struct');
msgStructsp_meas_mode{1}
if_measured = cellfun(@(m) double(m.Linear.X),msgStructsp_meas_mode);
% y_P_mav = cellfun(@(m) double(m.Linear.Y),msgStructsp_P_mav);


% % plot(x_QR_det,y_QR_det)
% % hold on
% plot(x_EKF,y_EKF, 'marker', 'x','Color',[0.85 0.080 0.18])
% hold on
% plot(x_P_mav,y_P_mav)
% % hold on
% % plot(d_uwb)
% legend("ekf","p_mav")

%%
figure(5)
% axis([-1.5 2.5 -2.2 2.2])
xlabel("X Position")
ylabel("Y Position")
title("Drone Localization with Etended QR Code & UWB & EKF")
hold on
% for t=200:5:1136
%     figure(5)
%     hold on
%     plot(x_EKF(start:t,1),y_EKF(start:t,1), 'rx')
%     hold on
%     plot(x_P_mav(start:t,1),y_P_mav(start:t,1),'g--','Linewidth',1.8)
%     if if_measured(t) == 0
%         hold on
%         plot(x_QR_det(t,1),y_QR_det(t,1),'k+','Linewidth',1.8, 'Markersize', 12)
%     end
%     pause(0.2)
%     %legend("EKF","PMAV","QR")
%     %clf
% end

start =195
start = 1
figure(5)
axis([-2.6 2 -3.68 2.42])
hold on
% 6.1
% 4.6 
for t=start:5:1136
    hold on
    plot(-x_EKF(start:2:t,1),-y_EKF(start:2:t,1), 'rx', 'Linewidth',1.5)
    hold on
    plot(-x_P_mav(start:2:t,1),-y_P_mav(start:2:t,1),'g--','Linewidth',2)
    if if_measured(t) == 0
        hold on
        plot(-x_QR_det(t,1),-y_QR_det(t,1),'k+','Linewidth',1.8, 'Markersize', 10)
    end
    pause(0.1)
    legend()
    %clf
end
% legend("EKF","Ground Truth","Camera Based")














% 
%     plot(x_EKF(start:t,1),y_EKF(start:t,1), 'marker', 'x','Color',[0.85 0.080 0.18])
%     hold on
%     plot(x_vicon(start:t,1),y_vicon(start:t,1),'LineStyle', '--', 'Color',[0.5,0.9,0.2],'Linewidth',1.8)




