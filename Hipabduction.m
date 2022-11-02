clear all ;close all, clc; 

directory = strcat('D:\Data\Exp_data\SOFT_EXOS\Abduction\Simulation\');
% directory = strcat('D:\Data\Exp_data\Incline_CW_pilot\total\');
files1 = dir(directory);  
num_files1 = size(files1,1)-2;
for i = 1:num_files1
    file_name1 = files1(i+2).name;
    name1 = [directory file_name1];
    raw{i} = readmatrix(name1);
end

directory2 = strcat('D:\Data\Exp_data\SOFT_EXOS\Abduction\thigh\');
% directory = strcat('D:\Data\Exp_data\Incline_CW_pilot\total\');
files2 = dir(directory2);  
num_files2 = size(files2,1)-2;
for i = 1:num_files2
    file_name2 = files2(i+2).name;
    name2 = [directory2 file_name2];
    raw2{i} = readmatrix(name2);
end

FSR = raw2{1,1};
time_fsr = FSR(:,1);
% cut3 = find(isnan(FSR(:,1)),1,'first');
% time_fsr1 = FSR(1:cut3-1,1);
% Lh1 = FSR(1:cut3-1,2);
% Lt1 = FSR(1:cut3-1,4);
% Rh1 = FSR(1:cut3-1,34);
% Rt1 = FSR(1:cut3-1,36);
% time_gy = FSR(:,15);
% cut4 = find(isnan(FSR(:,15)),1,'first');
% time_gy = FSR(1:cut4-1,15);
% Lgy_x = FSR(1:cut4-1,16);
% Lgy_y = FSR(1:cut4-1,18);
Lgy_z_k = FSR(:,20);
% Rgy_x = FSR(1:cut4-1,28);
% Rgy_y = FSR(1:cut4-1,30);
% Rgy_z = -FSR(1:cut4-1,32);
Lgy_zk_lp=lopass_butterworth(Lgy_z_k,4,370,2);
Lgy_zk_bp = highpass(Lgy_zk_lp,0.01);
figure(44)
hold on
plot(Lgy_z_k)
plot(Lgy_zk_lp)
plot(Lgy_zk_bp)


FSR = raw2{1,3};
% time_fsr = FSR(:,1);
cut3 = find(isnan(FSR(:,1)),1,'first');
time_fsr1 = FSR(1:cut3-1,1);
Lh1 = FSR(1:cut3-1,2);
Lt1 = FSR(1:cut3-1,4);
Rh1 = FSR(1:cut3-1,34);
Rt1 = FSR(1:cut3-1,36);
% time_gy = FSR(:,15);
cut4 = find(isnan(FSR(:,15)),1,'first');
time_gy = FSR(1:cut4-1,15);
Lgy_x = FSR(1:cut4-1,16);
Lgy_y = FSR(1:cut4-1,18);
Lgy_z = FSR(1:cut4-1,20);
Rgy_x = FSR(1:cut4-1,28);
Rgy_y = FSR(1:cut4-1,30);
Rgy_z = -FSR(1:cut4-1,32);


data = raw2{1,4};
cut1 = find(isnan(data(:,1)),1,'first');
time_fsr2 = data(1:cut1-1,1);
Lh2 = data(1:cut1-1,2);
Lt2 = data(1:cut1-1,4);
Rh2 = data(1:cut1-1,26);
Rt2 = data(1:cut1-1,28);
% time_an = data(:,11);
cut2 = find(isnan(data(:,11)),1,'first');
time_an_pr = data(1:cut2-1,11);
Lan_p = data(1:cut2-1,12);
Lan_r = data(1:cut2-1,14);
Lan_y = data(1:cut2-1,16);
Ran_p = data(1:cut2-1,20);
Ran_r = data(1:cut2-1,22);
Ran_y = data(1:cut2-1,24);






hold on



%% filter
Lgy_x_lp=lopass_butterworth(Lgy_x,4,370,2);
Lgy_y_lp=lopass_butterworth(Lgy_y,4,370,2);
Lgy_z_lp=lopass_butterworth(Lgy_z,4,370,2);
Rgy_x_lp=lopass_butterworth(Rgy_x,4,370,2);
Rgy_y_lp=lopass_butterworth(Rgy_y,4,370,2);
Rgy_z_lp=lopass_butterworth(Rgy_z,4,370,2);
Lgy_x_bp = highpass(Lgy_x_lp,0.01);
Lgy_y_bp = highpass(Lgy_y_lp,0.01);
Lgy_z_bp = highpass(Lgy_z_lp,0.01);
Rgy_x_bp = highpass(Rgy_x_lp,0.01);
Rgy_y_bp = highpass(Rgy_y_lp,0.01);
Rgy_z_bp = highpass(Rgy_z_lp,0.01);
Lan_p_lp=lopass_butterworth(Lan_p,6,74,4);
Lan_r_lp=lopass_butterworth(Lan_r,6,74,4);
Lan_y_lp=lopass_butterworth(Lan_y,6,74,4);
Ran_p_lp=lopass_butterworth(Ran_p,6,74,4);
Ran_r_lp=lopass_butterworth(Ran_r,6,74,4);
Ran_y_lp=lopass_butterworth(Ran_y,6,74,4);
Lh1_lp=lopass_butterworth(Lh1,6,519,2);
Lt1_lp=lopass_butterworth(Lt1,6,519,2);
Rh1_lp=lopass_butterworth(Rh1,6,519,2);
Rt1_lp=lopass_butterworth(Rt1,6,519,2);
Lh2_lp=lopass_butterworth(Lh2,6,519,2);
Lt2_lp=lopass_butterworth(Lt2,6,519,2);
Rh2_lp=lopass_butterworth(Rh2,6,519,2);
Rt2_lp=lopass_butterworth(Rt2,6,519,2);

% Lgy_z_hp = highpass(Lgy_z,0.01);
% Lgy_z_bp = highpass(Lgy_z_lp,0.01);



figure(1)
hold on
plot(Lgy_x)
plot(Lgy_x_lp)

figure(2)
hold on
plot(Lgy_y)
plot(Lgy_y_lp)

figure(3)
hold on
plot(Rt1)
plot(Rt1_lp)

figure(4)
hold on
plot(Lan_r)
plot(Lan_r_lp)

figure(6)
hold on
plot(Lgy_z)
plot(Lgy_z_lp)
plot(Lgy_z_hp)
plot(Lgy_z_bp)
legend

%% ground truth
figure(5)
subplot(2,1,1)
hold on
% plot(time_fsr,Lh)
% plot(time_fsr,Lt)
% plot(time_fsr,Rh)
% plot(time_fsr,Rt)
plot(Lh1)
plot(Lt1)
plot(Rh1)
plot(Rt1)
legend
subplot(2,1,2)
hold on
% plot(time_fsr,Lh)
% plot(time_fsr,Lt)
% plot(time_fsr,Rh)
% plot(time_fsr,Rt)
plot(Lh2)
plot(Lt2)
plot(Rh2)
plot(Rt2)
legend

clear cheakp1 temp1 HS1 TO1 HS2 TO2
[LHS1,LTO1]= Gait_fsr(time_fsr1,Lh1_lp,Lt1_lp,zeros(length(time_fsr1),1),zeros(length(time_fsr1),1),6);
[RHS1,RTO1]= Gait_fsr(time_fsr1,Rh1_lp,Rt1_lp,zeros(length(time_fsr1),1),zeros(length(time_fsr1),1),6);
[LHS2,LTO2]= Gait_fsr(time_fsr2,Lh2_lp,Lt2_lp,zeros(length(time_fsr2),1),zeros(length(time_fsr2),1),6);
[RHS2,RTO2]= Gait_fsr(time_fsr2,Rh2_lp,Rt2_lp,zeros(length(time_fsr2),1),zeros(length(time_fsr2),1),6);
% [cheakp1,temp1] = Gait_fsr(time_fsr1,Lh1_lp,Lt1_lp,zeros(length(time_fsr1),1),zeros(length(time_fsr1),1),7);
% [cheakp1,temp1] = Gait_fsr(time_fsr2,a,b,c,d,e)


figure(5) 
subplot(2,1,1)
hold on
% xlim([21000,22400])
plot(Lh1_lp(:,1))
% plot(Rgy_x(:,1))
plot(Lt1_lp(:,1))
plot(LHS1(1:end-1,1),Lh1_lp(LHS1(1:end-1,1)),'r*')
% plot(temp_L_gy(:,),Lgy_z_lp(temp_L_gy(:,2)),'b*')
plot(LTO1(1:end-1,1),Lt1_lp(LTO1(1:end-1,1)),'b*')
% plot(temp_R_gy(:,2),Rgy_z_lp(temp_R_gy(:,2)),'b*')
subplot(2,1,2)
hold on
% xlim([21000,22400])
plot(Rh1_lp(:,1))
% plot(Rgy_x(:,1))
plot(Rt1_lp(:,1))
plot(RHS1(1:end-1,1),Rh1_lp(RHS1(1:end-1,1)),'r*')
% plot(temp_L_gy(:,),Lgy_z_lp(temp_L_gy(:,2)),'b*')
plot(RTO1(1:end-1,1),Rt1_lp(RTO1(1:end-1,1)),'b*')
% plot(temp_R_gy(:,2),Rgy_z_lp(temp_R_gy(:,2)),'b*')
figure(6)
subplot(2,1,1)
hold on
% xlim([21000,22400])
plot(Lh2_lp(:,1))
% plot(Rgy_x(:,1))
plot(Lt2_lp(:,1))
plot(LHS2(1:end-1,1),Lh2_lp(LHS2(1:end-1,1)),'r*')
% plot(temp_L_gy(:,),Lgy_z_lp(temp_L_gy(:,2)),'b*')
plot(LTO2(1:end-1,1),Lt2_lp(LTO2(1:end-1,1)),'b*')
% plot(temp_R_gy(:,2),Rgy_z_lp(temp_R_gy(:,2)),'b*')
subplot(2,1,2)
hold on
% xlim([21000,22400])
plot(Rh2_lp(:,1))
% plot(Rgy_x(:,1))
plot(Rt2_lp(:,1))
plot(RHS2(1:end-1,1),Rh2_lp(RHS2(1:end-1,1)),'r*')
% plot(temp_L_gy(:,),Lgy_z_lp(temp_L_gy(:,2)),'b*')
plot(RTO2(1:end-1,1),Rt2_lp(RTO2(1:end-1,1)),'b*')
% plot(temp_R_gy(:,2),Rgy_z_lp(temp_R_gy(:,2)),'b*')
for k=1:length(LHS1)
[~,LHS1_gy(k,1)] = min(abs(time_fsr1(LHS1(k,1))-time_gy));
end
for k=1:length(LTO1)
[~,LTO1_gy(k,1)] = min(abs(time_fsr1(LTO1(k,1))-time_gy));
end
for k=1:length(RHS1)
[~,RHS1_gy(k,1)] = min(abs(time_fsr1(RHS1(k,1))-time_gy));
end
for k=1:length(RTO1)
[~,RTO1_gy(k,1)] = min(abs(time_fsr1(RTO1(k,1))-time_gy));
end

for k=1:length(LHS2)
[~,LHS2_an(k,1)] = min(abs(time_fsr2(LHS2(k,1))-time_an_pr));
end
for k=1:length(LTO2)
[~,LTO2_an(k,1)] = min(abs(time_fsr2(LTO2(k,1))-time_an_pr));
end
for k=1:length(RHS2)
[~,RHS2_an(k,1)] = min(abs(time_fsr2(RHS2(k,1))-time_an_pr));
end
for k=1:length(RTO2)
[~,RTO2_an(k,1)] = min(abs(time_fsr2(RTO2(k,1))-time_an_pr));
end



%% data modify

figure(3)
% subplot(2,1,1)
xlim([51000,53000])
hold on
plot(Lgy_z_bp(:,1))
plot(LHS1_gy(1:end-1,1),Lgy_z_bp(LHS1_gy(1:end-1,1)),'r*')
plot(LTO1_gy(1:end-1,1),Lgy_z_bp(LTO1_gy(1:end-1,1)),'b*')
% subplot(2,1,2)
% xlim([50000,55000])
% hold on
plot(Rgy_z_bp(:,1))
plot(RHS1_gy(1:end-1,1),Rgy_z_bp(RHS1_gy(1:end-1,1)),'ro')
plot(RTO1_gy(1:end-1,1),Rgy_z_bp(RTO1_gy(1:end-1,1)),'bo')

figure(33)
% subplot(2,1,1)
% xlim([51000,53000])
hold on
plot(Lan_r_lp(:,1))
plot(LHS2_an(1:end-1,1),Lan_r_lp(LHS2_an(1:end-1,1)),'r*')
plot(LTO2_an(1:end-1,1),Lan_r_lp(LTO2_an(1:end-1,1)),'b*')
% subplot(2,1,2)
% xlim([50000,55000])
% hold on
plot(Ran_r_lp(:,1))
plot(RHS2_an(1:end-1,1),Ran_r_lp(RHS2_an(1:end-1,1)),'ro')
plot(RTO2_an(1:end-1,1),Ran_r_lp(RTO2_an(1:end-1,1)),'bo')







cp1 = [LHS1(100,1),LHS1(101,1),LHS1(102,1) ; LTO1(100,1),LTO1(101,1),LTO1(102,1);...
    RHS1(100,1),RHS1(101,1),RHS1(102,1) ; RTO1(100,1),RTO1(101,1),RTO1(102,1)];
cp_t = time_fsr1(cp1);
[~,cp_gy(1,1)] = min(abs(cp_t(1,1)-time_gy));
[~,cp_gy(1,2)] = min(abs(cp_t(1,2)-time_gy));
[~,cp_gy(1,3)] = min(abs(cp_t(1,3)-time_gy));
[~,cp_gy(2,1)] = min(abs(cp_t(2,1)-time_gy));
[~,cp_gy(2,2)] = min(abs(cp_t(2,2)-time_gy));
[~,cp_gy(2,3)] = min(abs(cp_t(2,3)-time_gy));
[~,cp_gy(3,1)] = min(abs(cp_t(3,1)-time_gy));
[~,cp_gy(3,2)] = min(abs(cp_t(3,2)-time_gy));
[~,cp_gy(3,3)] = min(abs(cp_t(3,3)-time_gy));
[~,cp_gy(4,1)] = min(abs(cp_t(4,1)-time_gy));
[~,cp_gy(4,2)] = min(abs(cp_t(4,2)-time_gy));
[~,cp_gy(4,3)] = min(abs(cp_t(4,3)-time_gy));
rr = 2000/(cp_gy(1,3)-cp_gy(1,1)+1);
cp_gy_ratio = rr* ((cp_gy-cp_gy(1,1)+1)) ;
xx = (0:(cp_gy(1,3)-cp_gy(1,1)+1)/1999:cp_gy(1,3)-cp_gy(1,1)+1)';
Lgy_z_pr = Lgy_z_bp(cp_gy(1,1):cp_gy(1,3)-1,1);
Lgy_z_it = interp1(Lgy_z_pr,xx);
Rgy_z_pr = Rgy_z_bp(cp_gy(1,1):cp_gy(1,3)-1,1);
Rgy_z_it = interp1(Rgy_z_pr,xx);

hold on
plot(Lgy_z_pr)
plot(Lgy_z_it)

figure(11)
hold on
plot(Lgy_z_it,'linewidth',4)
plot(Rgy_z_it,'linewidth',4)
ax = gca ;
ax.FontSize = 30 ;
t=title('Thigh angular velocity on frontal plane');
t.FontSize = 40;
xlabel('Gait cycle percentage(%)','FontSize',20)
ylabel('Angular velocity (deg/s)','FontSize',20)
xline(cp_gy_ratio(1,2),'b','linewidth',2) % Lhs
xline(cp_gy_ratio(2,1),'--b','linewidth',2) %lto
xline(cp_gy_ratio(2,2),'--b','linewidth',2) %lto
xline(cp_gy_ratio(3,2),'r','linewidth',2) % rhs
xline(cp_gy_ratio(3,3),'r','linewidth',2) % rhs
xline(cp_gy_ratio(4,1),'--r','linewidth',2) %rto
xline(cp_gy_ratio(4,2),'--r','linewidth',2) %rto
l = legend('Left' , 'Right');
l.FontSize = 14;
hold off

hold on
plot(Lgy_z_it)
plot(Rgy_z_it)



%%

% temp_L = [30055;30619;31176];
% temp_R = [30363;30926];

% [~,temp_L_gy(1,1)] = min(abs(time_gy - time_fsr(30055)));
% [~,temp_L_gy(2,1)] = min(abs(time_gy - time_fsr(30619)));
% [~,temp_L_gy(3,1)] = min(abs(time_gy - time_fsr(31176)));
% % [~,temp_L_gy(2,2)] = min(abs(time_gy - time_fsr(5310)));
% 
% [~,temp_R_gy(1,1)] = min(abs(time_gy - time_fsr(30363)));
% [~,temp_R_gy(2,1)] = min(abs(time_gy - time_fsr(30926)));
% % [~,temp_R_gy(1,2)] = min(abs(time_gy - time_fsr(5051)));
% % [~,temp_R_gy(2,2)] = min(abs(time_gy - time_fsr(5584)));
% 
% 
% figure(5) 
% hold on
% xlim([21000,22400])
% % plot(Lgy_x(:,1))
% plot(Lgy_z_lp(:,1))
% % plot(Rgy_x(:,1))
% plot(Rgy_z_lp(:,1))
% plot(temp_L_gy(:,1),Lgy_z_lp(temp_L_gy(:,1)),'r*')
% % plot(temp_L_gy(:,),Lgy_z_lp(temp_L_gy(:,2)),'b*')
% plot(temp_R_gy(:,1),Rgy_z_lp(temp_R_gy(:,1)),'ro')
% % plot(temp_R_gy(:,2),Rgy_z_lp(temp_R_gy(:,2)),'b*')

% figure(9) 
% hold on
% xlim([21000,22400])
% plot(Lgy_z(:,1))
% % plot(Lgy_z_lp(:,1))
% plot(Rgy_z(:,1))
% % plot(Rgy_z_lp(:,1))
% plot(temp_L_gy(:,1),Lgy_z(temp_L_gy(:,1)),'r*')
% plot(temp_L_gy(:,2),Lgy_z(temp_L_gy(:,2)),'b*')
% plot(temp_R_gy(:,1),Rgy_z(temp_R_gy(:,1)),'r*')
% plot(temp_R_gy(:,2),Rgy_z(temp_R_gy(:,2)),'b*')

xspa2 = (0:802/20000:802)';
% Lgy_pr  = spline(time_gy(3152:3964,1),Lgy_z_lp(3152:3964,1),xspa2);
% Rgy_pr  = spline(,Rgy_z_lp(3152:3964,1),xspa2);
lz = Lgy_z_lp(21468:22269,1);
rz = Rgy_z_lp(21468:22269,1);
Lgy_it  = interp1(lz,xspa2);
Rgy_it  = interp1(rz,xspa2);
Lgy_pr = [(0:0.01:200)' , Lgy_it];
Rgy_pr = [(0:0.01:200)' , Rgy_it];


figure(11)
hold on
plot(Lgy_pr(:,1),Lgy_pr(:,2),'linewidth',5)
plot(Rgy_pr(:,1),Rgy_pr(:,2),'linewidth',5)
ax = gca ;
ax.FontSize = 30 ;
t=title('Thigh angular velocity on frontal plane');
t.FontSize = 40;
xlabel('Gait cycle percentage(%)','FontSize',20)
ylabel('Angular velocity (deg/s)','FontSize',20)
xline(((21688-21468)*200)/802,'r','linewidth',3)
xline(((21871-21468)*200)/802,'b','linewidth',3)
xline(((22090-21468)*200)/802,'r','linewidth',3)
xline((((21688-21468)*200)+((22090-21688)*0.6*200))/802,'--r','linewidth',3)
xline((0+((21871-21468)*0.63*200))/802,'--b','linewidth',3)
xline((((21871-21468)*200)+((22269-21871)*0.63*200))/802,'--b','linewidth',3)
l = legend('Left' , 'Right');
l.FontSize = 14;
hold off
%%
M_a = raw{1,1};
M_h = raw{1,2};
P_a = raw{1,3};
P_h = raw{1,4};
M_h_lp = lopass_butterworth(M_h(:,2),6,100,4);
M_a_lp = lopass_butterworth(M_a(:,2),6,100,4);
P_h_lp = lopass_butterworth(P_h(:,2),6,100,4);
P_a_lp = lopass_butterworth(P_a(:,2),6,100,4);
hold on 
plot(M_h(:,2))
plot(M_h_lp)
plot(M_a(:,2))
plot(M_a_lp)
plot(P_a(:,2))
plot(P_a_lp)
plot(P_h(:,2))
plot(P_h_lp)


xspa = (0:0.01:100)' ;
% M_h_pr  = spline(M_h(:,1),M_h_lp,xspa);
% M_a_pr = spline(M_a(:,1),M_a_lp,xspa);
% P_h_pr  = spline(P_h(:,1),P_h_lp,xspa);
% P_a_pr = spline(P_a(:,1),P_a_lp,xspa);
M_h_pr  = pchip(M_h(:,1),M_h_lp,xspa);
M_a_pr = pchip(M_a(:,1),M_a_lp,xspa);
P_h_pr  = pchip(P_h(:,1),P_h_lp,xspa);
P_a_pr = pchip(P_a(:,1),P_a_lp,xspa);
M_h_pr_sm = smooth(M_h_pr,200);
M_a_pr_sm = smooth(M_a_pr,200);
P_h_pr_sm = smooth(P_h_pr,200);
P_a_pr_sm = smooth(P_a_pr,200);



figure(6)
xlim([0,100])
ylim([-0.5,2])
hold on
% 
plot(xspa,M_h_pr_sm,'linewidth',5)
plot(xspa,M_a_pr_sm,'linewidth',5)
ax = gca ;
ax.FontSize = 30 ;
t=title('Simulated Moment');
t.FontSize = 40;
xlabel('time (% Gait cycle)','FontSize',20)
ylabel('Moment (N*m/kg)','FontSize',20)
yline(0)
xline(12)
xline(31)
xline(50)
xline(62)
xline(75)
xline(87)
l = legend('human abduction moment' , 'Mechanical abduction moment');
l.FontSize = 14;


figure(7)
xlim([0,100])
hold on
plot(xspa,P_h_pr_sm,'linewidth',5)
plot(xspa,P_a_pr_sm,'linewidth',5)
ax = gca ;
ax.FontSize = 30 ;
t=title('Simulated Power');
t.FontSize = 40;
xlabel('time (% Gait cycle)','FontSize',20)
ylabel('Power (W/kg)','FontSize',20)
yline(0)
xline(12)
xline(31)
xline(50)
xline(62)
xline(75)
xline(87)
l = legend('human abduction power' , 'Mechanical abduction power');
l.FontSize = 14;




Hip_abduction.Moment.human = M_h_pr ;
Hip_abduction.Moment.actuator = M_a_pr ;
Hip_abduction.Power.human = P_h_pr;
Hip_abduction.Power.actuator = P_a_pr ;


save Hip_abduction

%% 
load GT
L_angle = GT.sub1.Left.angle;
L_HS = GT.sub1.Left.HS.raw ; 
R_angle = GT.sub1.Right.angle;
R_HS = GT.sub1.Right.HS.raw ; 

L_an = L_angle(L_HS(100,1):L_HS(102,1),1);
R_an = R_angle(L_HS(100,1):L_HS(102,1),1);
xx = (0:200/(L_HS(102,1)-L_HS(100,1)):200)';
xspa = (0:0.01:200)' ;
L_angle_pr = spline(xx,L_an,xspa);
R_angle_pr = spline(xx,R_an,xspa);

figure(1)
hold on
plot(xspa,L_angle_pr,'linewidth',5)
plot(xspa,R_angle_pr,'linewidth',5)
ax = gca ;
ax.FontSize = 30 ;
t=title('Thigh angule on frontal plane');
t.FontSize = 40;
xlabel('Gait cycle percentage(%)','FontSize',20)
ylabel('Angular velocity (deg/s)','FontSize',20)
xline(((21688-21468)*200)/802,'r','linewidth',3)
xline(((21871-21468)*200)/802,'b','linewidth',3)
xline(((22090-21468)*200)/802,'r','linewidth',3)
xline((((21688-21468)*200)+((22090-21688)*0.6*200))/802,'--r','linewidth',3)
xline((0+((21871-21468)*0.63*200))/802,'--b','linewidth',3)
xline((((21871-21468)*200)+((22269-21871)*0.63*200))/802,'--b','linewidth',3)
l = legend('Left' , 'Right');
l.FontSize = 14;
hold off

plot(




