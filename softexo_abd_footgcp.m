%%
clc; clear all; close all;
%% data load
% directory = strcat('EXP_data\SOFT_EXOS\Abduction\ABD_force_ctl\');
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\20211116_footimutest\control\');
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211118_foot\control\');
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211123_foot\control\');  
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211124_foot_algorithm_test\control\');
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211129_motor\control3\');
% directory = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_220113_form\');
directory = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\control\');
files1 = dir(directory);  
num_files1 = size(files1,1)-2;
for i = 1:num_files1
    file_name1 = files1(i+2).name;
    name1 = [directory file_name1];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw_control{i} = readmatrix(name1); 
%     raw{i} =load(name1) ;
end

% directory2 = strcat('Exp_data\SOFT_EXOS\Abduction\20211116_footimutest\analog\');
% directory2 = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211119_foot\analog\');
% directory2 = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211123_foot\analog\');
% directory2 = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211124_foot_algorithm_test\analog\');
directory2 = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\motion_sync\');
files2 = dir(directory2);  
num_files2 = size(files2,1)-2;
for i = 1:num_files2
    file_name2 = files2(i+2).name;
    name2 = [directory2 file_name2];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw_sync{i} = readmatrix(name2); 
%     raw{i} =load(name1) ;
end

% directory3 = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211123_foot\motion\');
% directory3 = strcat('Exp_data\SOFT_EXOS\Abduction\ABD_211124_foot_algorithm_test\motion\');
directory3 = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\\motion_final\');
files3 = dir(directory3);  
num_files3 = size(files3,1)-2;
for i = 1:num_files3
    file_name3 = files3(i+2).name;
    name3 = [directory3 file_name3];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw_motion{i} = readmatrix(name3); 
%     raw{i} =load(name1) ;
end

directory4 = strcat('EXP_data\SOFT_EXOS\Abduction\HIP_ABD_main\metabolic\');
files4 = dir(directory4);  
num_files4 = size(files4,1)-2;
for i = 1:num_files4
    file_name4 = files4(i+2).name;
    name4 = [directory4 file_name4];
    raw2{i} = readmatrix(name4);
    raw3{i} = readtable(name4);
end

directory5 = strcat('EXP_data\SOFT_EXOS\Abduction\HIP_ABD_main\Xsensor_PSD\');
files5 = dir(directory5);  
num_files5 = size(files5,1)-2;
for i = 1:num_files5
    file_name5 = files5(i+2).name;
    name5 = [directory5 file_name5];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw4{i} = readmatrix(name5);
%     raw3{i} = xlsread(name2);
%     raw2{i} = readmatrix(name2);
%     raw{i} =load(name1) ;
end


directory6 = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\motion_final_static\');
files6 = dir(directory6);  
num_files6 = size(files6,1)-2;
for i = 1:num_files6
    file_name6 = files6(i+2).name;
    name6 = [directory6 file_name6];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw_static{i} = readmatrix(name6); 
%     raw{i} =load(name1) ;
end

%%
% j=30;
for j=1:50
%%
close all
clear -regexp ^yy_ ^PSD_ ^temp ^L_ ^R_ ^COM_ ^Pelvis_ ^MoS_ ^XCOM_ ^ind ^Q_sync ^time_ ^ST ^P1_ ^P2_ ^P3_ ^P4_ ^NW_ ^Time_ ^B_ ^Q_ ^power_
clear -regexp ^acc_
clear LHS LTO RHS RTO time_mo time_mos i k data7 aa aaa g leg pp qq nn step_mf_x xspa xx leng ans data2 COP_related contact_pressure data4...
    kk xxx RHS_X formal hh e H2H bb SW SWdev SWs xxs time RGP LGP kg PP PN PT rr eff eff DST_time data11 data111 
% % % 
% HIP_ABD.sub01 = sub01;
% HIP_ABD.sub02 = sub02;
% HIP_ABD.sub03 = sub03;
% HIP_ABD.sub04 = sub04;
% HIP_ABD.sub05 = sub05;
% HIP_ABD.sub06 = sub06;
% HIP_ABD.sub07 = sub07;
% % % 
% % save HIP_ABD_supp_load '-v7.3'
% save HIP_ABD_10_participants
%% variables - analog

data2 = raw_sync{j}(3:end,:);
% data2_timing = raw_sync{j}(3:end,1);
time_sync = (1:length(data2))';
for k=1:length(data2)-1
    if data2(k,3) < 1.5 && data2(k+1,3) > 1.5
       ind1 = k+1;
    elseif data2 (k,3) > 1.5 && data2(k+1,3) < 1.5 
       ind2 = k; 
       break
    end
end
num1 = 2;
Q_sync_ds(1,1) = data2(1,3);
for k=1:length(data2)-1
if data2(k,1) ~= data2(k+1,1)
    Q_sync_ds(num1,1) = data2(k+1,3);
    num1=num1+1;
end
end
% ind1 = 1856;
% ind2 = 206124;
if rem(j,5) ~=1
   ind11 = data2(ind1,1);
   ind22 = data2(ind2,1);
   ind33 = ind22-ind11;
   Q_sync_dss = Q_sync_ds(ind11:ind22,1);
end
%% variables - motion
% j=29;

data7 = raw_motion{j};
data8 = raw_static{j};
if rem(j,5) == 1 
   data7 = raw_motion{j};  
elseif j==15
    data7 = raw_motion{j}; 
elseif j~=15
    data7 = data7(ind11:ind22,:);
end
%%% x : mediolateral y : anterior-posterior z: vertical
time_mo = (1:length(data7))';
for k = 2:70
nn =  isnan(data7(:,k));
for i=2:length(data7(:,k))-1
    if nn(i,1) == 1 
       data7(i,k) = 0;
    end
end
end

data7(1,:) = 0;
data7(end,:) = 0;
data71 = lopass_butterworth(data7,5,100,2);
% MS = sort([RMS;LMS]);
% L_to = data7(:,19:21);
% R_to = data7(:,22:24); 
% L_heel = data7(:,1:3);
% R_heel = data7(:,10:12);
% R_mf_v = data7(:,7);
% L_mf_v = data7(:,4);

L_Heel_x = data7(:,2);
L_Heel_y = data7(:,3);
L_Heel_z = data7(:,4);
L_Met1_x = data7(:,5);
L_Met1_y = data7(:,6);
L_Met1_z = data7(:,7);
L_Met5_x = data7(:,8);
L_Met5_y = data7(:,9);
L_Met5_z = data7(:,10);
L_ANK_angle_x = data7(:,38)-mean(data8(:,38),'omitnan');
L_ANK_angle_y = data7(:,39)-mean(data8(:,39),'omitnan');
L_ANK_angle_z = data7(:,40)-mean(data8(:,40),'omitnan');
L_KNE_angle_x = data7(:,44)-mean(data8(:,44),'omitnan');
L_KNE_angle_y = data7(:,45)-mean(data8(:,45),'omitnan');
L_KNE_angle_z = data7(:,46)-mean(data8(:,46),'omitnan');
L_HIP_angle_x = data7(:,41)-mean(data8(:,41),'omitnan');
L_HIP_angle_y = data7(:,42)-mean(data8(:,42),'omitnan');
L_HIP_angle_z = data7(:,43)-mean(data8(:,43),'omitnan');
L_mf_x = data7(:,23);
L_mf_y = data7(:,24);
L_mf_z = data7(:,25);
L_mf_v = data7(:,70);


R_Heel_x = data7(:,11);
R_Heel_y = data7(:,12);
R_Heel_z = data7(:,13);
R_Met1_x = data7(:,14);
R_Met1_y = data7(:,15);
R_Met1_z = data7(:,16);
R_Met5_x = data7(:,17);
% R_Met5_x = data7(:,17);-mean(data8(:,17),'omitnan');
R_Met5_y = data7(:,18);
R_Met5_z = data7(:,19);
R_ANK_angle_x = data7(:,50)-mean(data8(:,50),'omitnan');
R_ANK_angle_y = data7(:,51)-mean(data8(:,51),'omitnan');
R_ANK_angle_z = data7(:,52)-mean(data8(:,52),'omitnan');
R_KNE_angle_s = data7(:,56)-mean(data8(:,56),'omitnan');
R_KNE_angle_f = data7(:,57)-mean(data8(:,57),'omitnan');
R_KNE_angle_z = data7(:,58)-mean(data8(:,58),'omitnan');
R_HIP_angle_s = data7(:,53)-mean(data8(:,53),'omitnan'); %% x : SAGITTAL
R_HIP_angle_f = data7(:,54)-mean(data8(:,54),'omitnan'); %% y : Forntal
R_HIP_angle_z = data7(:,55)-mean(data8(:,55),'omitnan'); %% z : transverse
R_HIP_angvel_s = gradient(R_HIP_angle_s,0.01);
R_HIP_angvel_s_3 =lopass_butterworth(R_HIP_angvel_s,6,100,2);
R_HIP_angvel_f = gradient(R_HIP_angle_f,0.01);
R_HIP_angvel_f_3 =lopass_butterworth(R_HIP_angvel_f,6,100,2);
R_HIP_angvel_z = gradient(R_HIP_angle_z,0.01);
R_HIP_angvel_z_3 =lopass_butterworth(R_HIP_angvel_z,6,100,2);

% figure(3)
% hold on
% plot(R_HIP_angle_y)
% % plot(R_HIP_angvel_y_rad)
% plot(R_HIP_angvel_y_rad3)

R_mf_x = data7(:,29);
R_mf_y = data7(:,30);
R_mf_z = data7(:,31);

R_mf_v = data7(:,67);

COM_x  = data7(:,59);
COM_y  = data7(:,60);
COM_z = data7(:,61);
COM_vel_x = data7(:,62);
COM_vel_y = data7(:,63);
COM_vel_z = data7(:,64);
% COM_vel_x_lp = lopass_butterworth(COM_vel_x,6,100,2);
% COM_vel_y_lp = lopass_butterworth(COM_vel_y,6,100,2);
% COM_vel_z_lp = lopass_butterworth(COM_vel_z,6,100,2);

% COM_x  = data7(:,59);
% COM_y  = data7(:,60);
% COM_z = data7(:,61);
% COM_vel_x = data7(:,62);
% COM_vel_y = data7(:,63);
% COM_vel_z = data7(:,64);

COM_acc_x = gradient(COM_vel_x,0.01);
COM_acc_y = gradient(COM_vel_y,0.01);
COM_acc_z = gradient(COM_vel_z,0.01);
COM_acc_x_lp = lopass_butterworth(COM_acc_x,6,100,2);
COM_acc_y_lp = lopass_butterworth(COM_acc_y,6,100,2);
COM_acc_z_lp = lopass_butterworth(COM_acc_z,6,100,2);
COM_vel_yz = sqrt(COM_vel_y.^2 +COM_vel_z.^2);
COM_vel_r = sqrt(COM_vel_x.^2 +COM_vel_y.^2 +COM_vel_z.^2) ;

Pelvis_tilt_x = data7(:,47)-mean(data8(:,47),'omitnan');
Pelvis_tilt_y = data7(:,48)-mean(data8(:,48),'omitnan');
Pelvis_tilt_z = data7(:,49)-mean(data8(:,49),'omitnan');

L_mf = L_mf_v(:,1);
R_mf = R_mf_v(:,1);
L_mf_lp= lopass_butterworth(L_mf,7,100,4);
R_mf_lp= lopass_butterworth(R_mf,7,100,4);
time_mos = (1:length(L_mf_lp))';

[RHS,RTO,RST] = Gait_footvel(R_mf_lp,[0.6,0.1,0.1,0.9]);
[LHS,LTO,LST] = Gait_footvel(L_mf_lp,[0.6,0.1,0.1,0.9]);

while RHS(end,1) > RTO(end,1)
    RHS = RHS(1:end-1,1);
end

while LHS(end,1) > RTO(end,1)
    LHS = LHS(1:end-1,1);
end


W_vel_pre = [COM_vel_y(LHS(end-119:end,1)) ,COM_vel_z(LHS(end-119:end,1))];
W_del = [COM_vel_y(RTO(end-119:end,1))-COM_vel_y(LHS(end-119:end,1)),COM_vel_z(RTO(end-119:end,1))-COM_vel_z(LHS(end-119:end,1))];
W_po = (dot(W_vel_pre, W_del,2)).^2;

W_vel_pre2 = [COM_vel_x(LHS(end-119:end,1)),COM_vel_y(LHS(end-119:end,1)) ,COM_vel_z(LHS(end-119:end,1))];
W_del2 = [COM_vel_x(RTO(end-119:end,1))-COM_vel_x(LHS(end-119:end,1)),COM_vel_y(RTO(end-119:end,1))-COM_vel_y(LHS(end-119:end,1)),...
    COM_vel_z(RTO(end-119:end,1))-COM_vel_z(LHS(end-119:end,1))];
W_po2 = (dot(W_vel_pre2, W_del2,2)).^2;

% figure(886)
% hold on
% plot(time_mos,R_mf_lp)
% plot(time_mos(RHS),R_mf_lp(RHS),'*r')
% plot(time_mos(RTO),R_mf_lp(RTO),'*b')
% 
% figure(887)
% hold on
% plot(time_mos,L_mf_lp)
% plot(time_mos(LHS),L_mf_lp(LHS),'*r')
% plot(time_mos(LTO),L_mf_lp(LTO),'*b')

leng = max(COM_z);
g=9.81 ; 

XCOM_x = COM_x + COM_vel_x*sqrt(leng/g);
XCOM_y = COM_y + COM_vel_y*sqrt(leng/g);
XCOM_z = COM_z + COM_vel_z*sqrt(leng/g);
XCOM_r = sqrt(XCOM_x.^2 +XCOM_y.^2 +XCOM_z.^2) ;

MoS_x_L = XCOM_x - L_Met5_x;  
MoS_x_R = R_Met5_x - XCOM_x; 
% 
% MoS_y_L = L_Heel_y - XCOM_y;  
MoS_y_R = R_Heel_y - XCOM_y; 

for pp= 1:120 
    [MoS_x_R_min(pp,1),qq] = min(MoS_x_R(RHS(end-pp,1):RTO(end-pp+1,1)));
    temp_bos = R_Met5_x(RHS(end-pp,1):RTO(end-pp+1,1));
    temp_xcom = XCOM_x(RHS(end-pp,1):RTO(end-pp+1,1));
    BoS_x_R_min(pp,1) = temp_bos(qq,1);
    XCOM_x_R_min(pp,1) = temp_xcom(qq,1);
    MoS_x_L_min(pp,1) = min(MoS_x_L(RHS(end-pp,1):RTO(end-pp+1,1)));
end

if RHS(end,1) < RTO(end,1) 
   STS_time = (RTO(end-119:end,1) - RHS(end-119:end,1))*0.01;
   SW_time = (RHS(end-119:end,1) - RTO(end-120:end-1,1))*0.01; 
elseif RHS(end,1) > RTO(end,1)
   STS_time = (RTO(end-119:end,1) - RHS(end-120:end-1,1))*0.01;
   SW_time = (RHS(end-119:end,1) - RTO(end-119:end,1))*0.01; 
end

for kk=1:length(time_mos)-1
    if RST(kk,1) == 1 && LST(kk,1) ==1 
       DST_time(kk,1) = 1;
    else
       DST_time(kk,1) =0;
    end
end

H2H = R_Heel_y-L_Heel_y;
ST_time = (RHS(2:end,1) - RHS(1:end-1,1))*0.01;
ST_freq = 1./ST_time;
bb=length(RHS)-120;
for pp= 1:120 
    ST_length(pp,1) = H2H(RHS(bb+pp,1),1);
%     MoS_y_R_min(pp,1) = min(MoS_y_R(RHS(end-pp,1):RHS(end-pp+1,1)));
end

R_cross = COM_y(RHS(end-119,1):RHS(end,1),1)-R_Heel_y(RHS(end-119,1):RHS(end,1),1);
L_cross = COM_y(RHS(end-119,1):RHS(end,1),1)-L_Heel_y(RHS(end-119,1):RHS(end,1),1);
step_mf_x = R_mf_x(RHS(end-119,1):RHS(end,1),1)-L_mf_x(RHS(end-119,1):RHS(end,1),1);

qq = 1;
for i=1:length(R_cross)-1
    if R_cross(i,1)< 0 && R_cross(i+1,1) > 0 
       R_cc(qq,1) = i;
       qq = qq+1;
    end
end

qq = 1;
for i=1:length(L_cross)-1
    if L_cross(i,1) < 0 && L_cross(i+1,1) > 0 
       L_cc(qq,1) = i;
       qq = qq+1;
    end
end

R_stepwidth = step_mf_x(R_cc);
L_stepwidth = step_mf_x(L_cc);

% SW{j,1} = R_stepwidth;

    
PSD_COM_x=[];
PSD_COM_y=[];
PSD_COM_z=[];
PSD_COM_vel_x=[];
PSD_COM_vel_y=[];
PSD_COM_vel_z=[];
PSD_COM_vel_yz = [];
PSD_COM_vel_r=[];
PSD_COM_acc_x=[];
PSD_COM_acc_y=[];
PSD_COM_acc_z=[];
PSD_DST = [];
PSD_XCOM_x = [];
PSD_XCOM_y = [];
PSD_XCOM_z = [];
PSD_XCOM_r = [];
PSD_Hip_ang_s = [];
PSD_Hip_ang_f = [];
PSD_Hip_ang_z = [];
PSD_Hip_angvel_s = [];
PSD_Hip_angvel_f = [];
PSD_Hip_angvel_z = [];
PSD_MOS_M = [];
PSD_MOS_L = [];
PSD_MOS_AP = [];
PSD_Kne_ang_s = [];
PSD_Kne_ang_f = [];
PSD_Kne_ang_z = [];

PSD_R_met5_x = [];
PSD_L_met5_x = [];
for i = length(RHS)-119:length(RHS)
    xx = (0:100/(RHS(i)-1-RHS(i-1,1)):100)';
%     yy_COM_vel_x = COM_vel_x(RHS(i-1):RHS(i)-1,1)-COM_vel_x(RHS(i-1),1);
%     yy_COM_vel_y = COM_vel_y(RHS(i-1):RHS(i)-1,1)-COM_vel_y(RHS(i-1),1);
%     yy_COM_vel_z = COM_vel_z(RHS(i-1):RHS(i)-1,1)-COM_vel_z(RHS(i-1),1); 
    yy_COM_vel_x = COM_vel_x(RHS(i-1):RHS(i)-1,1);
    yy_COM_vel_y = COM_vel_y(RHS(i-1):RHS(i)-1,1);
    yy_COM_vel_z = COM_vel_z(RHS(i-1):RHS(i)-1,1); 
    yy_COM_vel_yz = COM_vel_yz(RHS(i-1):RHS(i)-1,1)-COM_vel_yz(RHS(i-1),1);
    yy_COM_vel_r = COM_vel_r(RHS(i-1):RHS(i)-1,1)-COM_vel_r(RHS(i-1),1); 
    yy_COM_x = COM_x(RHS(i-1):RHS(i)-1,1);
    yy_COM_y = COM_y(RHS(i-1):RHS(i)-1,1);
    yy_COM_z = COM_z(RHS(i-1):RHS(i)-1,1);
    yy_COM_acc_x = COM_acc_x_lp(RHS(i-1):RHS(i)-1,1);
    yy_COM_acc_y = COM_acc_y_lp(RHS(i-1):RHS(i)-1,1);
    yy_COM_acc_z = COM_acc_z_lp(RHS(i-1):RHS(i)-1,1);
%         yy_COM_x = COM_x(RHS(i-1):RHS(i)-1,1)-COM_x(RHS(i-1),1);
%     yy_COM_y = COM_y(RHS(i-1):RHS(i)-1,1)-COM_y(RHS(i-1),1);
%     yy_COM_z = COM_z(RHS(i-1):RHS(i)-1,1)-COM_z(RHS(i-1),1);
%     yy_COM_x = COM_x(RHS(i-1):RHS(i)-1,1)-mean(data8(:,59));
%     yy_COM_y = COM_y(RHS(i-1):RHS(i)-1,1)-mean(data8(:,60));
%     yy_COM_z = COM_z(RHS(i-1):RHS(i)-1,1)-mean(data8(:,61));
    yy_DST = DST_time(RHS(i-1):RHS(i)-1,1);
    yy_XCOM_x = XCOM_x(RHS(i-1):RHS(i)-1,1);
    yy_XCOM_y = XCOM_y(RHS(i-1):RHS(i)-1,1);
    yy_XCOM_z = XCOM_z(RHS(i-1):RHS(i)-1,1);
    yy_XCOM_r = XCOM_r(RHS(i-1):RHS(i)-1,1);
    yy_hip_ang_s = R_HIP_angle_s(RHS(i-1):RHS(i)-1,1);
    yy_hip_ang_f = R_HIP_angle_f(RHS(i-1):RHS(i)-1,1);
    yy_hip_ang_z = R_HIP_angle_z(RHS(i-1):RHS(i)-1,1);
    yy_hip_angvel_s = R_HIP_angvel_s_3(RHS(i-1):RHS(i)-1,1);
    yy_hip_angvel_f = R_HIP_angvel_f_3(RHS(i-1):RHS(i)-1,1);
    yy_hip_angvel_z = R_HIP_angvel_z_3(RHS(i-1):RHS(i)-1,1);
    yy_MOS_L = MoS_x_R(RHS(i-1):RHS(i)-1,1);
    yy_MOS_AP = MoS_y_R(RHS(i-1):RHS(i)-1,1);
    yy_kne_ang_s = R_KNE_angle_s(RHS(i-1):RHS(i)-1,1);
    yy_kne_ang_f = R_KNE_angle_f(RHS(i-1):RHS(i)-1,1);
    yy_kne_ang_z = R_KNE_angle_z(RHS(i-1):RHS(i)-1,1);
    yy_R_met5_x = R_Met5_x(RHS(i-1):RHS(i)-1,1);

    
    xspa = 0:0.1:100 ;
    temp41 = interp1(xx,yy_COM_acc_x,xspa);
    temp42 = interp1(xx,yy_COM_acc_y,xspa);
    temp43 = interp1(xx,yy_COM_acc_z,xspa);
    temp44 = interp1(xx,yy_COM_vel_yz,xspa);
    temp45 = interp1(xx,yy_COM_vel_r,xspa);
    temp46 = interp1(xx,yy_COM_vel_x,xspa);
    temp47 = interp1(xx,yy_COM_vel_y,xspa);
    temp48 = interp1(xx,yy_COM_vel_z,xspa);
    temp49 = interp1(xx,yy_COM_x,xspa);
    temp50 = interp1(xx,yy_COM_y,xspa);
    temp51 = interp1(xx,yy_COM_z,xspa);
    temp52 = interp1(xx,yy_DST,xspa);
    temp53 = interp1(xx,yy_XCOM_x,xspa);
    temp54 = interp1(xx,yy_XCOM_y,xspa);
    temp55 = interp1(xx,yy_XCOM_z,xspa);
    temp56 = interp1(xx,yy_XCOM_r,xspa);
    temp57 = interp1(xx,yy_hip_ang_s,xspa);
    temp58 = interp1(xx,yy_hip_ang_f,xspa);
    temp59 = interp1(xx,yy_hip_ang_z,xspa);
    temp60 = interp1(xx,yy_hip_angvel_s,xspa);
    temp61 = interp1(xx,yy_hip_angvel_f,xspa);
    temp62 = interp1(xx,yy_hip_angvel_z,xspa);
    temp63 = interp1(xx,yy_MOS_L,xspa);
    temp64 = interp1(xx,yy_MOS_AP,xspa);
    temp65 = interp1(xx,yy_kne_ang_s,xspa);
    temp66 = interp1(xx,yy_kne_ang_f,xspa);
    temp67 = interp1(xx,yy_kne_ang_z,xspa);
    temp68 = interp1(xx,yy_R_met5_x,xspa);
    
    
    PSD_COM_acc_x=[PSD_COM_acc_x, temp41'];
    PSD_COM_acc_y=[PSD_COM_acc_y, temp42'];
    PSD_COM_acc_z=[PSD_COM_acc_z, temp43'];

    PSD_COM_vel_yz = [PSD_COM_vel_yz, temp44'];
    PSD_COM_vel_r = [PSD_COM_vel_r , temp45'];
    PSD_COM_vel_x = [PSD_COM_vel_x , temp46'];
    PSD_COM_vel_y = [PSD_COM_vel_y , temp47'];
    PSD_COM_vel_z = [PSD_COM_vel_z , temp48'];
    PSD_COM_x = [PSD_COM_x , temp49'];
    PSD_COM_y = [PSD_COM_y , temp50'];
    PSD_COM_z = [PSD_COM_z , temp51'];   
    PSD_DST = [PSD_DST, temp52'];
    PSD_XCOM_x = [PSD_XCOM_x temp53'];
    PSD_XCOM_y = [PSD_XCOM_y temp54'];
    PSD_XCOM_z = [PSD_XCOM_z temp55'];
    PSD_XCOM_r = [PSD_XCOM_r temp56'];   
    PSD_Hip_ang_s = [PSD_Hip_ang_s temp57'];
    PSD_Hip_ang_f = [PSD_Hip_ang_f temp58'];
    PSD_Hip_ang_z = [PSD_Hip_ang_z temp59'];  
    PSD_Hip_angvel_s = [PSD_Hip_angvel_s temp60'];
    PSD_Hip_angvel_f = [PSD_Hip_angvel_f temp61'];
    PSD_Hip_angvel_z = [PSD_Hip_angvel_z temp62'];  
    PSD_MOS_L = [PSD_MOS_L temp63'];  
    PSD_MOS_AP = [PSD_MOS_AP temp64']; 
    PSD_Kne_ang_s = [PSD_Kne_ang_s temp65'];
    PSD_Kne_ang_f = [PSD_Kne_ang_f temp66'];
    PSD_Kne_ang_z = [PSD_Kne_ang_z temp67'];
    PSD_R_met5_x = [PSD_R_met5_x temp68'];
end
for pp = 1:120
    for qq = 2:300
        if PSD_DST(qq-1,pp) >= 0.5 && PSD_DST(qq,pp) < 0.5
           PSD_DST2(pp,1) = qq;
        end
    end
    for rr = 400:900
        if PSD_DST(rr-1,pp) < 0.5 && PSD_DST(rr,pp) >= 0.5
           PSD_DST2(pp,2) = rr;
        elseif PSD_DST(rr-1,pp) >= 0.5 && PSD_DST(rr,pp) < 0.5
           PSD_DST2(pp,3) = rr;
        end
    end
  
end


COM{j,1} = PSD_COM_x-mean(mean(PSD_COM_x));
COM{j,2} = PSD_COM_y-mean(mean(PSD_COM_y));
COM{j,3} = PSD_COM_z-mean(mean(PSD_COM_z));
COM{j,4} = PSD_COM_vel_x;
COM{j,5} = PSD_COM_vel_y;
COM{j,6} = PSD_COM_vel_z;
COM{j,7} = PSD_COM_vel_r;
COM{j,8} = PSD_XCOM_x;
COM{j,9} = PSD_XCOM_y;
COM{j,10} = PSD_XCOM_z;
COM{j,11} = PSD_XCOM_r;
COM{j,12} = PSD_COM_vel_yz;
COM{j,13} = PSD_COM_acc_x;
COM{j,14} = PSD_COM_acc_y;
COM{j,15} = PSD_COM_acc_z;


RMET{j,1} = PSD_R_met5_x;

MOS{j,1} = PSD_MOS_L;
MOS{j,2} = PSD_MOS_AP;
MOS{j,3} = MoS_x_R_min ;
MOS{j,4} = MoS_x_L_min ;
MOS{j,5} = BoS_x_R_min;
MOS{j,6} = XCOM_x_R_min;

Time{j,1} = STS_time;
Time{j,2} = SW_time;
Time{j,3} = PSD_DST;
Time{j,4} = PSD_DST2(:,1);
Time{j,5} = PSD_DST2(:,2);
Time{j,6} = PSD_DST2(:,3);
Time{j,7} = (PSD_DST2(:,3)-PSD_DST2(:,3))./100;


HIP{j,1} = PSD_Hip_ang_s;
HIP{j,2} = PSD_Hip_ang_f;
HIP{j,3} = PSD_Hip_ang_z;
HIP{j,4} = PSD_Hip_angvel_s;
HIP{j,5} = PSD_Hip_angvel_f;
HIP{j,6} = PSD_Hip_angvel_z;  
% end

W{j,1} = W_vel_pre;
W{j,2} = W_del;
W{j,3} = W_po;
W{j,4} = W_vel_pre2;
W{j,5} = W_del2;
W{j,6} = W_po2;

aa = mean(PSD_Hip_ang_s,2);

KNE{j,1} = PSD_Kne_ang_s;
KNE{j,2} = PSD_Kne_ang_f;
KNE{j,3} = PSD_Kne_ang_z;

%%%%%%%%%%%%%%%%% spario-temporal param %%%%%%%%%%%%%%%%%%%
BR{j,1} = PSD_MOS_L ; %%lateral mos
BR{j,2} = PSD_XCOM_x ; %%%% XCOM
BR{j,3} = PSD_R_met5_x ; %%%%%% BOS
BR{j,4} = R_stepwidth ; %% ave. stepwidth
BR{j,5} = std(R_stepwidth) ;


%%%%%%%%%%%%%%%%% spario-temporal param %%%%%%%%%%%%%%%%%%%
s_stp{j,1} = ST_length;
s_stp{j,2} = ST_freq(end-119:end,1);
s_stp{j,3} = STS_time;
s_stp{j,4} = (PSD_DST2(:,3)-PSD_DST2(:,2))./100;
s_stp{j,5} = SW_time;




%% variables - control
if rem(j,5)~=1
data1 = raw_control{j}; %(25000:40000,:); %(2797:2900,:)  ;  % right 기준
% data = raw{2}; %(11668:11790,:) ;  % right 기준
%%% 데이터 큐 기준 마지막 1분 cut
Q_sync = data1(:,46);
for k=1:length(Q_sync)-1
    if Q_sync(k,1) == 0 && Q_sync(k+1,1) == 1
       index1 = k+1;
    elseif Q_sync(k,1) == 1 && Q_sync(k+1,1) == 0 
       index2 = k; 
       break
    end
end
if j == 9
data = data1(1132:1132+ind33,:);
% data = data1; 
elseif j==15
data = data1(index2-18000:index2,:);
else%     data = data1(index1:index2,:);   
    data = data1(index1:index1+ind33,:);   
end

else
data = raw_control{j} ;  
end


time = (1:length(data))'; 
L_force = data(:,2);
L_ang = data(:,4);
L_ang_h = data(:,5);
L_ang_f = data(:,6);
L_acc_x = data(:,7);
L_acc_y = data(:,8);
L_acc_z = data(:,9);

L_gcp = data(:,31); 
L_num = data(:,32);
L_period = data(:,33);
L_pos = data(:,40);
L_vel = data(:,41);
% L_pos_tar = data(:,46);
% L_vel_tar = data(:,47);
L_cur_tar = data(:,39);
L_cur_act = data(:,52);
L_angvel_s = data(:,22);
L_angvel_h = data(:,23);
L_angvel_f = data(:,24);
L_gcp_1k = data(:,49);
% L_run_ok = data(:,53);
L_force_ref= data(:,48);
L_force_mode= data(:,50);
L_ang_filt = data(:,63);


R_force = data(:,3);
R_ang = data(:,10);
R_ang_h = data(:,11);
R_ang_f = data(:,12);
R_acc_x = data(:,13);
R_acc_y = data(:,14);
R_acc_z = data(:,15);
R_gcp = data(:,34);
R_num = data(:,35);
R_period = data(:,36);
R_pos = data(:,37);
R_vel = data(:,48);
% R_pos_tar = data(:,41);
% R_vel_tar = data(:,42);
R_cur_tar = data(:,42);
R_cur_act = data(:,53);
R_angvel_s = data(:,25);
R_angvel_h = data(:,26);
R_angvel_f = data(:,27);

R_onset_ctl = data(:,43);
R_peak_ctl= data(:,44);
R_release_ctl = data(:,45);
R_force_ref= data(:,49);
R_force_mode= data(:,51);
Q_sync = data(:,46);
R_opt_count = data(:,47);
R_init_count = data(:,54);
R_angvel_raw = data(:,55);
R_angvel_filt = data(:,56);
L_angvel_raw = data(:,57);
L_angvel_filt = data(:,58);
L_gcp_G = data(:,59);
L_num_G = data(:,60);
R_gcp_G = data(:,61);
R_num_G = data(:,62);
R_motor_status = data(:,63);
L_motor_status = data(:,64);
Q_PSD = data(:,65);
Q_MOTOR = data(:,66);
Q_GCP = data(:,67);
Time_SAVE_loop = data(:,68);
Time_SAVE_int = data(:,69);
Time_GCP_loop = data(:,70);
Time_GCP_int = data(:,71);
Time_MOTOR_loop = data(:,72);
Time_MOTOR_int = data(:,73);
Time_PSD_loop = data(:,74);
Time_PSD_int = data(:,75);
R_angvel_filt_62 = data(:,76);
L_angvel_filt_62 = data(:,77);

% Time_saveloop = data(:,70);

B_ang = data(:,16);
B_ang_f = data(:,17);
B_ang_h = data(:,18);
B_acc_x = data(:,19);
B_acc_y = data(:,20);
B_acc_z = data(:,21);
B_angvel_s = data(:,28);
B_angvel_f = data(:,29);
B_angvel_h = data(:,30);
if rem(j,5)~= 1
B_angvel_f_lp = lopass_butterworth((B_angvel_f),6,100,2);
B_angvel_s_lp = lopass_butterworth((B_angvel_s),6,100,2);
else
B_angvel_f_lp = NaN;
B_angvel_s_lp = NaN;   
end
% R_angvel_s_lp = lopass_butterworth((R_angvel_s),6,100,2);


%% Power analysis

kg = [75;75;75;75;75;73;73;73;73;73;70;70;70;70;70;67;67;67;67;67;65;65;65;65;65;86;86;86;86;86;94;94;94;94;94;63;63;63;63;63;...
    75;75;75;75;75;90;90;90;90;90]; 

RGP =[];
LGP = [];
num1 = 1;

for k=1:length(R_num)-1
    if floor(R_num(k,1)) ~= floor(R_num(k+1,1))
       RGP(num1,1) = k+1;
       num1 = num1+1;
    end    
end

num1 = 1;
for k=1:length(L_num)-1
    if L_num(k,1) ~= L_num(k+1,1)
       LGP(num1,1) = k+1;
       num1 = num1+1;
    end    
end



% figure(44)
% yyaxis left
% hold on
% plot(time,R_angvel_filt_62)
% plot(time(RGP,1),R_angvel_filt_62(RGP,1),'b*')
% plot(time(RMS,1),R_angvel_filt_62(RMS,1),'r*')
% % plot(time(RGP2,1),R_angvel_s_lp(RGP2,1),'r*')
% 
% yyaxis right
% plot(time_mo,R_mf_lp)
% % plot(time_mo,Q_sync_ds)
% plot(time_mo(RHS,1),R_mf_lp(RHS,1),'r*')
% plot(time_mo(RTO,1),R_mf_lp(RTO,1),'b*')
% figure(45)
% hold on
% % plot(RHS)
% % plot(RGP)
% plot(RHS(end-119:end,1)-RGP(end-119:end,1))


R_moment = R_force.*0.06/kg(j,1);
L_moment = L_force.*0.06/kg(j,1);
R_power = R_force.*B_angvel_f_lp*(pi/180).*(0.06/kg(j,1));
% plot(R_power)
% hold on
% plot(-R_power_mo)
R_power_mo = R_moment.*(R_HIP_angvel_f_3)*(pi/180);
R_power_mo_s = R_moment.*(R_HIP_angvel_s_3)*(pi/180);
PSD_R_force = [];
PSD_R_force_r = [];
PSD_L_force_r = [];
PSD_L_force_rh = [];
PSD_R_moment = [];
PSD_R_power = [];
PSD_R_power_mo = [];
PSD_R_power_mo_s = [];
PSD_B_angvel_f = [];
PSD_B_angvel_s = [];
PSD_R_HIP_angvel_f_rad = [];
PSD_R_HIP_angvel_s_rad = [];
PSD_L_force = [];
PSD_R_angvel = [];
PSD_R_cur_tar = [];
PSD_R_cur_act = [];
PSD_L_angvel = [];
PSD_L_cur_tar = [];
PSD_L_cur_act = [];
PSD_L_moment = [];
if rem(j,5)~=1
    for i = length(RGP)-119:length(RGP)
        xx = (0:100/(RGP(i)-1-RGP(i-1,1)):100)';
        yy_R_force = R_force(RGP(i-1):RGP(i)-1,1);
        yy_L_force_rh = L_force(RGP(i-1):RGP(i)-1,1);
        yy_R_moment = R_moment(RGP(i-1):RGP(i)-1,1);
        yy_R_power = R_power(RGP(i-1):RGP(i)-1,1);
        yy_R_power_mo = R_power_mo(RGP(i-1):RGP(i)-1,1);
        yy_B_angvel_f = B_angvel_f_lp(RGP(i-1):RGP(i)-1,1);
        yy_B_angvel_s = B_angvel_s_lp(RGP(i-1):RGP(i)-1,1);
        yy_R_HIP_angvel_f_rad = R_HIP_angvel_f_3(RGP(i-1):RGP(i)-1,1); 
    %     yy_R_HIP_angvel_s_rad = R_HIP_angvel_s_3(RHS(i-1):RHS(i)-1,1); 
        yy_R_power_mo_s = R_power_mo_s(RGP(i-1):RGP(i)-1,1);
        yy_R_force_r= R_force_ref(RGP(i-1):RGP(i)-1,1);
        yy_R_angvel_s = R_angvel_s(RGP(i-1):RGP(i)-1,1);
        yy_R_cur_tar = R_cur_tar(RGP(i-1):RGP(i)-1,1);
        yy_R_cur_act = R_cur_act(RGP(i-1):RGP(i)-1,1);

        xspa = 0:0.1:100 ;
        tempp29 = interp1(xx,yy_R_force,xspa);  
        tempp30 = interp1(xx,yy_R_power,xspa);
        tempp31 = interp1(xx,yy_R_moment,xspa);
        tempp32 = interp1(xx,yy_R_power_mo,xspa);
        tempp33 = interp1(xx,yy_B_angvel_f,xspa);
        tempp34 = interp1(xx,yy_R_HIP_angvel_f_rad,xspa);
        tempp35 = interp1(xx,yy_B_angvel_s,xspa);
        tempp36 = interp1(xx,yy_L_force_rh,xspa);
        tempp37 = interp1(xx,yy_R_power_mo_s,xspa);
        tempp38 = interp1(xx,yy_R_force_r,xspa);
        tempp39 = interp1(xx,yy_R_angvel_s,xspa);
        tempp40 = interp1(xx,yy_R_cur_tar,xspa);
        tempp41 = interp1(xx,yy_R_cur_act,xspa);
        
        
        PSD_R_force = [PSD_R_force, tempp29'];
        PSD_R_power = [PSD_R_power , tempp30'];
        PSD_R_moment = [PSD_R_moment , tempp31'];
        PSD_R_power_mo = [PSD_R_power_mo , tempp32'];
        PSD_B_angvel_f = [PSD_B_angvel_f,tempp33'];
        PSD_R_HIP_angvel_f_rad = [PSD_R_HIP_angvel_f_rad,tempp34'];
        PSD_B_angvel_s = [PSD_B_angvel_s tempp35'];
        PSD_L_force_rh = [PSD_L_force_rh tempp36'];
        PSD_R_power_mo_s = [PSD_R_power_mo_s tempp37'];
        PSD_R_force_r = [PSD_R_force_r tempp38'];
        PSD_R_angvel = [PSD_R_angvel,tempp39'];
        PSD_R_cur_tar = [PSD_R_cur_tar,tempp40'];
        PSD_R_cur_act = [PSD_R_cur_act tempp41'];         
        
    end

    aa = (RGP(end,1)-LGP);
    aaa = find(aa>0,1,'last');

    for i = aaa-118:aaa
        xx = (0:100/(LGP(i)-1-LGP(i-1,1)):100)';
        yy_L_force_r = L_force_ref(LGP(i-1):LGP(i)-1,1);
        yy_L_force = L_force(LGP(i-1):LGP(i)-1,1);
        yy_L_angvel_s = L_angvel_s(LGP(i-1):LGP(i)-1,1);
        yy_L_cur_tar = L_cur_tar(LGP(i-1):LGP(i)-1,1);
        yy_L_cur_act = L_cur_act(LGP(i-1):LGP(i)-1,1);
        yy_L_moment = L_moment(LGP(i-1):LGP(i)-1,1);
    %     yy_L_power = L_power(LGP(i-1):LGP(i)-1,1);

        xspa = 0:0.1:100 ;

        tempp10 = interp1(xx,yy_L_force,xspa);
        tempp11 = interp1(xx,yy_L_force_r,xspa);
        tempp12 = interp1(xx,yy_L_angvel_s,xspa);
        tempp13 = interp1(xx,yy_L_cur_tar,xspa);
        tempp14 = interp1(xx,yy_L_cur_act,xspa);
        tempp15 = interp1(xx,yy_L_moment,xspa);
    %     temp11 = interp1(xx,yy_L_ANK_angle_y,xspa);

        PSD_L_force = [PSD_L_force , tempp10'];
        PSD_L_force_r = [PSD_L_force_r tempp11'];
        PSD_L_angvel = [PSD_L_angvel, tempp12'];
        PSD_L_cur_tar = [PSD_L_cur_tar, tempp13'];
        PSD_L_cur_act = [PSD_L_cur_act, tempp14'];
        PSD_L_moment = [PSD_L_moment tempp15'];
    end
end

% figure(12)
% yyaxis left
% hold on
% plot(mean(PSD_R_HIP_angvel_f_rad,2))
% yyaxis right
% plot(mean(PSD_Hip_angvel_f,2))

% figure(13)
% hold on
% plot(mean(PSD_R_power,2))
% plot(mean(PSD_R_power_mo,2))
% figure(14)
% hold on
% plot(time_mo,R_HIP_angvel_f_3)
% plot(time_mo(RHS,1),R_HIP_angvel_f_3(RHS,1),'r*')
% plot(time_mo(RGP,1),R_HIP_angvel_f_3(RGP,1),'b*')

CON{j,1} = PSD_R_power;
CON{j,2} = PSD_R_force;
CON{j,3} = PSD_L_force;
CON{j,4} = PSD_R_moment;
CON{j,5} = PSD_R_power_mo;
CON{j,6} = PSD_B_angvel_f;
CON{j,7} = PSD_B_angvel_s;
CON{j,8} = PSD_Hip_angvel_f;
CON{j,9} = PSD_Hip_angvel_s;
CON{j,10} = PSD_R_power_mo_s;
CON{j,11} = PSD_L_force_rh;




ctl{j,1} = PSD_R_force_r;
ctl{j,2} = PSD_R_force;
ctl{j,3} = PSD_L_force_r;
ctl{j,4} = PSD_L_force;
ctl{j,5} = PSD_R_moment;
ctl{j,6} = PSD_R_power;
ctl{j,7} = PSD_B_angvel_f ; %% thigh angular velocity
ctl{j,8} = PSD_R_angvel;
ctl{j,9} = PSD_R_cur_tar;
ctl{j,10} = PSD_R_cur_act ; %% thigh angular velocity
ctl{j,11} = PSD_L_angvel;
ctl{j,12} = PSD_L_cur_tar;
ctl{j,13} = PSD_L_cur_act ; %% thigh angular velocity
ctl{j,14} = PSD_L_moment;

end


%% 
xxs = 0:0.1:100;

for j=1:50
    if rem(j,5) ~= 1
    a1 = CON{j,1};
    for qq=1:120
        a_p = find(a1(:,qq)>=0);
        a_p2(qq,1) = mean(a1(a_p,1));
    end
        a_p3(1,j) = mean(a_p2);
    else
        a_p3(1,j) = 0;
    end
end

a_p4 = [a_p3(1,1:5);a_p3(1,6:10);a_p3(1,11:15);a_p3(1,16:20);a_p3(1,21:25);a_p3(1,26:30);...
    a_p3(1,31:35);a_p3(1,36:40);a_p3(1,41:45);a_p3(1,46:50)];

SW = [mean(BR{1,4}),mean(BR{2,4}),mean(BR{3,4}),mean(BR{4,4}),mean(BR{5,4});...
    mean(BR{6,4}),mean(BR{7,4}),mean(BR{8,4}),mean(BR{9,4}),mean(BR{10,4});...
    mean(BR{11,4}),mean(BR{12,4}),mean(BR{13,4}),mean(BR{14,4}),mean(BR{15,4});...
    mean(BR{16,4}),mean(BR{17,4}),mean(BR{18,4}),mean(BR{19,4}),mean(BR{20,4});...
    mean(BR{21,4}),mean(BR{22,4}),mean(BR{23,4}),mean(BR{24,4}),mean(BR{25,4});...
    mean(BR{26,4}),mean(BR{27,4}),mean(BR{28,4}),mean(BR{29,4}),mean(BR{30,4});...
    mean(BR{31,4}),mean(BR{32,4}),mean(BR{33,4}),mean(BR{34,4}),mean(BR{35,4});...
    mean(BR{36,4}),mean(BR{37,4}),mean(BR{38,4}),mean(BR{39,4}),mean(BR{40,4});...
    mean(BR{41,4}),mean(BR{42,4}),mean(BR{43,4}),mean(BR{44,4}),mean(BR{45,4});...
    mean(BR{46,4}),mean(BR{47,4}),mean(BR{48,4}),mean(BR{49,4}),mean(BR{50,4})];

s_sl = [mean(s_stp{1,1}),mean(s_stp{2,1}),mean(s_stp{3,1}),mean(s_stp{4,1}),mean(s_stp{5,1});...
    mean(s_stp{6,1}),mean(s_stp{7,1}),mean(s_stp{8,1}),mean(s_stp{9,1}),mean(s_stp{10,1});...
    mean(s_stp{11,1}),mean(s_stp{12,1}),mean(s_stp{13,1}),mean(s_stp{14,1}),mean(s_stp{15,1});...
    mean(s_stp{16,1}),mean(s_stp{17,1}),mean(s_stp{18,1}),mean(s_stp{19,1}),mean(s_stp{20,1});...
    mean(s_stp{21,1}),mean(s_stp{22,1}),mean(s_stp{23,1}),mean(s_stp{24,1}),mean(s_stp{25,1});...
    mean(s_stp{26,1}),mean(s_stp{27,1}),mean(s_stp{28,1}),mean(s_stp{29,1}),mean(s_stp{30,1});...
    mean(s_stp{31,1}),mean(s_stp{32,1}),mean(s_stp{33,1}),mean(s_stp{34,1}),mean(s_stp{35,1});...
    mean(s_stp{36,1}),mean(s_stp{37,1}),mean(s_stp{38,1}),mean(s_stp{39,1}),mean(s_stp{40,1});...
    mean(s_stp{41,1}),mean(s_stp{42,1}),mean(s_stp{43,1}),mean(s_stp{44,1}),mean(s_stp{45,1});...
    mean(s_stp{46,1}),mean(s_stp{47,1}),mean(s_stp{48,1}),mean(s_stp{49,1}),mean(s_stp{50,1})];

s_sf = [mean(s_stp{1,2}),mean(s_stp{2,2}),mean(s_stp{3,2}),mean(s_stp{4,2}),mean(s_stp{5,2});...
    mean(s_stp{6,2}),mean(s_stp{7,2}),mean(s_stp{8,2}),mean(s_stp{9,2}),mean(s_stp{10,2});...
    mean(s_stp{11,2}),mean(s_stp{12,2}),mean(s_stp{13,2}),mean(s_stp{14,2}),mean(s_stp{15,2});...
    mean(s_stp{16,2}),mean(s_stp{17,2}),mean(s_stp{18,2}),mean(s_stp{19,2}),mean(s_stp{20,2});...
    mean(s_stp{21,2}),mean(s_stp{22,2}),mean(s_stp{23,2}),mean(s_stp{24,2}),mean(s_stp{25,2});...
    mean(s_stp{26,2}),mean(s_stp{27,2}),mean(s_stp{28,2}),mean(s_stp{29,2}),mean(s_stp{30,2});...
    mean(s_stp{31,2}),mean(s_stp{32,2}),mean(s_stp{33,2}),mean(s_stp{34,2}),mean(s_stp{35,2});...
    mean(s_stp{36,2}),mean(s_stp{37,2}),mean(s_stp{38,2}),mean(s_stp{39,2}),mean(s_stp{40,2});...
    mean(s_stp{41,2}),mean(s_stp{42,2}),mean(s_stp{43,2}),mean(s_stp{44,2}),mean(s_stp{45,2});...
    mean(s_stp{46,2}),mean(s_stp{47,2}),mean(s_stp{48,2}),mean(s_stp{49,2}),mean(s_stp{50,2})];

s_sts = [mean(s_stp{1,3}),mean(s_stp{2,3}),mean(s_stp{3,3}),mean(s_stp{4,3}),mean(s_stp{5,3});...
    mean(s_stp{6,3}),mean(s_stp{7,3}),mean(s_stp{8,3}),mean(s_stp{9,3}),mean(s_stp{10,3});...
    mean(s_stp{11,3}),mean(s_stp{12,3}),mean(s_stp{13,3}),mean(s_stp{14,3}),mean(s_stp{15,3});...
    mean(s_stp{16,3}),mean(s_stp{17,3}),mean(s_stp{18,3}),mean(s_stp{19,3}),mean(s_stp{20,3});...
    mean(s_stp{21,3}),mean(s_stp{22,3}),mean(s_stp{23,3}),mean(s_stp{24,3}),mean(s_stp{25,3});...
    mean(s_stp{26,3}),mean(s_stp{27,3}),mean(s_stp{28,3}),mean(s_stp{29,3}),mean(s_stp{30,3});...
    mean(s_stp{31,3}),mean(s_stp{32,3}),mean(s_stp{33,3}),mean(s_stp{34,3}),mean(s_stp{35,3});...
    mean(s_stp{36,3}),mean(s_stp{37,3}),mean(s_stp{38,3}),mean(s_stp{39,3}),mean(s_stp{40,3});...
    mean(s_stp{41,3}),mean(s_stp{42,3}),mean(s_stp{43,3}),mean(s_stp{44,3}),mean(s_stp{45,3});...
    mean(s_stp{46,3}),mean(s_stp{47,3}),mean(s_stp{48,3}),mean(s_stp{49,3}),mean(s_stp{50,3})];

s_ds = [mean(s_stp{1,4}),mean(s_stp{2,4}),mean(s_stp{3,4}),mean(s_stp{4,4}),mean(s_stp{5,4});...
    mean(s_stp{6,4}),mean(s_stp{7,4}),mean(s_stp{8,4}),mean(s_stp{9,4}),mean(s_stp{10,4});...
    mean(s_stp{11,4}),mean(s_stp{12,4}),mean(s_stp{13,4}),mean(s_stp{14,4}),mean(s_stp{15,4});...
    mean(s_stp{16,4}),mean(s_stp{17,4}),mean(s_stp{18,4}),mean(s_stp{19,4}),mean(s_stp{20,4});...
    mean(s_stp{21,4}),mean(s_stp{22,4}),mean(s_stp{23,4}),mean(s_stp{24,4}),mean(s_stp{25,4});...
    mean(s_stp{26,4}),mean(s_stp{27,4}),mean(s_stp{28,4}),mean(s_stp{29,4}),mean(s_stp{30,4});...
    mean(s_stp{31,4}),mean(s_stp{32,4}),mean(s_stp{33,4}),mean(s_stp{34,4}),mean(s_stp{35,4});...
    mean(s_stp{36,4}),mean(s_stp{37,4}),mean(s_stp{38,4}),mean(s_stp{39,4}),mean(s_stp{40,4});...
    mean(s_stp{41,4}),mean(s_stp{42,4}),mean(s_stp{43,4}),mean(s_stp{44,4}),mean(s_stp{45,4});...
    mean(s_stp{46,4}),mean(s_stp{47,4}),mean(s_stp{48,4}),mean(s_stp{49,4}),mean(s_stp{50,4})]*10;

s_swt = [mean(s_stp{1,5}),mean(s_stp{2,5}),mean(s_stp{3,5}),mean(s_stp{4,5}),mean(s_stp{5,5});...
    mean(s_stp{6,5}),mean(s_stp{7,5}),mean(s_stp{8,5}),mean(s_stp{9,5}),mean(s_stp{10,5});...
    mean(s_stp{11,5}),mean(s_stp{12,5}),mean(s_stp{13,5}),mean(s_stp{14,5}),mean(s_stp{15,5});...
    mean(s_stp{16,5}),mean(s_stp{17,5}),mean(s_stp{18,5}),mean(s_stp{19,5}),mean(s_stp{20,5});...
    mean(s_stp{21,5}),mean(s_stp{22,5}),mean(s_stp{23,5}),mean(s_stp{24,5}),mean(s_stp{25,5});...
    mean(s_stp{26,5}),mean(s_stp{27,5}),mean(s_stp{28,5}),mean(s_stp{29,5}),mean(s_stp{30,5});...
    mean(s_stp{31,5}),mean(s_stp{32,5}),mean(s_stp{33,5}),mean(s_stp{34,5}),mean(s_stp{35,5});...
    mean(s_stp{36,5}),mean(s_stp{37,5}),mean(s_stp{38,5}),mean(s_stp{39,5}),mean(s_stp{40,5});...
    mean(s_stp{41,5}),mean(s_stp{42,5}),mean(s_stp{43,5}),mean(s_stp{44,5}),mean(s_stp{45,5});...
    mean(s_stp{46,5}),mean(s_stp{47,5}),mean(s_stp{48,5}),mean(s_stp{49,5}),mean(s_stp{50,5})];


CON_NW = [mean([CON{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan'),mean([CON{[1,6,11,16,21,26,31,36,41,46],2}],2,'omitnan'),...
    mean([CON{[1,6,11,16,21,26,31,36,41,46],3}],2,'omitnan'),mean([CON{[1,6,11,16,21,26,31,36,41,46],4}],2,'omitnan'),...
    mean([CON{[1,6,11,16,21,26,31,36,41,46],5}],2,'omitnan'),mean([CON{[1,6,11,16,21,26,31,36,41,46],6}],2,'omitnan'),...
    mean([CON{[1,6,11,16,21,26,31,36,41,46],7}],2,'omitnan'),mean([CON{[1,6,11,16,21,26,31,36,41,46],10}],2,'omitnan'),...
    mean([CON{[1,6,11,16,21,26,31,36,41,46],11}],2,'omitnan')];
CON_P1 = [mean([CON{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan'),mean([CON{[2,7,12,17,22,27,32,37,42,47],2}],2,'omitnan'),...
    mean([CON{[2,7,12,17,22,27,32,37,42,47],3}],2,'omitnan'),mean([CON{[2,7,12,17,22,27,32,37,42,47],4}],2,'omitnan'),...
    mean([CON{[2,7,12,17,22,27,32,37,42,47],5}],2,'omitnan'),mean([CON{[2,7,12,17,22,27,32,37,42,47],6}],2,'omitnan'),...
    mean([CON{[2,7,12,17,22,27,32,37,42,47],7}],2,'omitnan'), mean([CON{[2,7,12,17,22,27,32,37,42,47],10}],2,'omitnan'),...
     mean([CON{[2,7,12,17,22,27,32,37,42,47],11}],2,'omitnan')];
CON_P2 = [mean([CON{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan'),mean([CON{[3,8,13,18,23,28,33,38,43,48],2}],2,'omitnan'),...
    mean([CON{[3,8,13,18,23,28,33,38,43,48],3}],2,'omitnan'),mean([CON{[3,8,13,18,23,28,33,38,43,48],4}],2,'omitnan'),...
    mean([CON{[3,8,13,18,23,28,33,38,43,48],5}],2,'omitnan'),mean([CON{[3,8,13,18,23,28,33,38,43,48],6}],2,'omitnan'),...
    mean([CON{[3,8,13,18,23,28,33,38,43,48],7}],2,'omitnan'),mean([CON{[3,8,13,18,23,28,33,38,43,48],10}],2,'omitnan'),...
    mean([CON{[3,8,13,18,23,28,33,38,43,48],11}],2,'omitnan')];
CON_P3 = [mean([CON{[4,9,14,19,24,29,34,39,44,49],1}],2,'omitnan'),mean([CON{[4,9,14,19,24,29,34,39,44,49],2}],2,'omitnan'),...
    mean([CON{[4,9,14,19,24,29,34,39,44,49],3}],2,'omitnan'),mean([CON{[4,9,14,19,24,29,34,39,44,49],4}],2,'omitnan'),...
    mean([CON{[4,9,14,19,24,29,34,39,44,49],5}],2,'omitnan'),mean([CON{[4,9,14,19,24,29,34,39,44,49],6}],2,'omitnan'),...
    mean([CON{[4,9,14,19,24,29,34,39,44,49],7}],2,'omitnan'),mean([CON{[4,9,14,19,24,29,34,39,44,49],10}],2,'omitnan'),...
    mean([CON{[4,9,14,19,24,29,34,39,44,49],11}],2,'omitnan')];
CON_P4 = [mean([CON{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan'),mean([CON{[5,10,15,20,25,30,35,40,45,50],2}],2,'omitnan'),...
    mean([CON{[5,10,15,20,25,30,35,40,45,50],3}],2,'omitnan'),mean([CON{[5,10,15,20,25,30,35,40,45,50],4}],2,'omitnan'),...
    mean([CON{[5,10,15,20,25,30,35,40,45,50],5}],2,'omitnan'),mean([CON{[5,10,15,20,25,30,35,40,45,50],6}],2,'omitnan'),...
    mean([CON{[5,10,15,20,25,30,35,40,45,50],7}],2,'omitnan'),mean([CON{[5,10,15,20,25,30,35,40,45,50],10}],2,'omitnan'),...
    mean([CON{[5,10,15,20,25,30,35,40,45,50],11}],2,'omitnan')];

CON_NW_std = [std([CON{[1,6,11,16,21,26,31,36,41,46],1}],0,2,'omitnan'),std([CON{[1,6,11,16,21,26,31,36,41,46],2}],0,2,'omitnan'),...
    std([CON{[1,6,11,16,21,26,31,36,41,46],3}],0,2,'omitnan'),std([CON{[1,6,11,16,21,26,31,36,41,46],4}],0,2,'omitnan'),...
    std([CON{[1,6,11,16,21,26,31,36,41,46],5}],0,2,'omitnan'),std([CON{[1,6,11,16,21,26,31,36,41,46],6}],0,2,'omitnan'),...
    std([CON{[1,6,11,16,21,26,31,36,41,46],7}],0,2,'omitnan'),std([CON{[1,6,11,16,21,26,31,36,41,46],10}],0,2,'omitnan'),...
    std([CON{[1,6,11,16,21,26,31,36,41,46],11}],0,2,'omitnan')];
CON_P1_std = [std([CON{[2,7,12,17,22,27,32,37,42,47],1}],0,2,'omitnan'),std([CON{[2,7,12,17,22,27,32,37,42,47],2}],0,2,'omitnan'),...
    std([CON{[2,7,12,17,22,27,32,37,42,47],3}],0,2,'omitnan'),std([CON{[2,7,12,17,22,27,32,37,42,47],4}],0,2,'omitnan'),...
    std([CON{[2,7,12,17,22,27,32,37,42,47],5}],0,2,'omitnan'),std([CON{[2,7,12,17,22,27,32,37,42,47],6}],0,2,'omitnan'),...
    std([CON{[2,7,12,17,22,27,32,37,42,47],7}],0,2,'omitnan'),std([CON{[2,7,12,17,22,27,32,37,42,47],10}],0,2,'omitnan'),...
    std([CON{[2,7,12,17,22,27,32,37,42,47],11}],0,2,'omitnan')];
CON_P2_std = [std([CON{[3,8,13,18,23,28,33,38,43,48],1}],0,2,'omitnan'),std([CON{[3,8,13,18,23,28,33,38,43,48],2}],0,2,'omitnan'),...
    std([CON{[3,8,13,18,23,28,33,38,43,48],3}],0,2,'omitnan'),std([CON{[3,8,13,18,23,28,33,38,43,48],4}],0,2,'omitnan'),...
    std([CON{[3,8,13,18,23,28,33,38,43,48],5}],0,2,'omitnan'),std([CON{[3,8,13,18,23,28,33,38,43,48],6}],0,2,'omitnan'),...
    std([CON{[3,8,13,18,23,28,33,38,43,48],7}],0,2,'omitnan'),std([CON{[3,8,13,18,23,28,33,38,43,48],10}],0,2,'omitnan'),...
    std([CON{[3,8,13,18,23,28,33,38,43,48],11}],0,2,'omitnan')];
CON_P3_std = [std([CON{[4,9,14,19,24,29,34,39,44,49],1}],0,2,'omitnan'),std([CON{[4,9,14,19,24,29,34,39,44,49],2}],0,2,'omitnan'),...
    std([CON{[4,9,14,19,24,29,34,39,44,49],3}],0,2,'omitnan'),std([CON{[4,9,14,19,24,29,34,39,44,49],4}],0,2,'omitnan'),...
    std([CON{[4,9,14,19,24,29,34,39,44,49],5}],0,2,'omitnan'),std([CON{[4,9,14,19,24,29,34,39,44,49],6}],0,2,'omitnan'),...
    std([CON{[4,9,14,19,24,29,34,39,44,49],7}],0,2,'omitnan'),std([CON{[4,9,14,19,24,29,34,39,44,49],10}],0,2,'omitnan'),...
    std([CON{[4,9,14,19,24,29,34,39,44,49],11}],0,2,'omitnan')];
CON_P4_std = [std([CON{[5,10,15,20,25,30,35,40,45,50],1}],0,2,'omitnan'),std([CON{[5,10,15,20,25,30,35,40,45,50],2}],0,2,'omitnan'),...
    std([CON{[5,10,15,20,25,30,35,40,45,50],3}],0,2,'omitnan'),std([CON{[5,10,15,20,25,30,35,40,45,50],4}],0,2,'omitnan'),...
    std([CON{[5,10,15,20,25,30,35,40,45,50],5}],0,2,'omitnan'),std([CON{[5,10,15,20,25,30,35,40,45,50],6}],0,2,'omitnan'),...
    std([CON{[5,10,15,20,25,30,35,40,45,50],7}],0,2,'omitnan'),std([CON{[5,10,15,20,25,30,35,40,45,50],10}],0,2,'omitnan'),...
    std([CON{[5,10,15,20,25,30,35,40,45,50],11}],0,2,'omitnan')];


B_angvel_NW = [mean([CON{[1,6,11,16,21,26,31,36,41,46],8}],2,'omitnan'),mean([CON{[1,6,11,16,21,26,31,36,41,46],9}],2,'omitnan')];
B_angvel_NW_std = [std([CON{[1,6,11,16,21,26,31,36,41,46],8}],0,2,'omitnan'), std([CON{[1,6,11,16,21,26,31,36,41,46],9}],0,2,'omitnan')];
B_angvel_P1 = [mean([CON{[2,7,12,17,22,27,32,37,42,47],8}],2,'omitnan'),mean([CON{[2,7,12,17,22,27,32,37,42,47],9}],2,'omitnan')]; 
B_angvel_P1_std = [std([CON{[2,7,12,17,22,27,32,37,42,47],8}],0,2,'omitnan'),std([CON{[2,7,12,17,22,27,32,37,42,47],9}],0,2,'omitnan')];
B_angvel_P2 = [mean([CON{[3,8,13,18,23,28,33,38,43,48],8}],2,'omitnan'),mean([CON{[3,8,13,18,23,28,33,38,43,48],9}],2,'omitnan')]; 
B_angvel_P2_std = [std([CON{[3,8,13,18,23,28,33,38,43,48],8}],0,2,'omitnan'),std([CON{[3,8,13,18,23,28,33,38,43,48],9}],0,2,'omitnan')];
B_angvel_P3 = [mean([CON{[4,9,14,19,24,29,34,39,44,49],8}],2,'omitnan'),mean([CON{[4,9,14,19,24,29,34,39,44,49],9}],2,'omitnan')]; 
B_angvel_P3_std = [std([CON{[4,9,14,19,24,29,34,39,44,49],8}],0,2,'omitnan'),std([CON{[4,9,14,19,24,29,34,39,44,49],9}],0,2,'omitnan')];  
B_angvel_P4 = [mean([CON{[5,10,15,20,25,30,35,40,45,50],8}],2,'omitnan'),mean([CON{[5,10,15,20,25,30,35,40,45,50],9}],2,'omitnan')]; 
B_angvel_P4_std = [std([CON{[5,10,15,20,25,30,35,40,45,50],8}],0,2,'omitnan'),std([CON{[5,10,15,20,25,30,35,40,45,50],9}],0,2,'omitnan')];

% % 
% figure(1)
% subplot(3,1,1)
% hold on
% plot(xxs,mean([CON{4,5}],2,'omitnan'))
% plot(xxs,mean([CON{9,5}],2,'omitnan'))
% plot(xxs,mean([CON{14,5}],2,'omitnan'))
% plot(xxs,mean([CON{19,5}],2,'omitnan'))
% plot(xxs,mean([CON{24,5}],2,'omitnan'))
% plot(xxs,mean([CON{29,5}],2,'omitnan'))
% plot(xxs,mean([CON{34,5}],2,'omitnan'))
% % figure(2)
% subplot(3,1,2)
% hold on
% plot(xxs,mean([CON{4,4}],2,'omitnan'))
% plot(xxs,mean([CON{9,4}],2,'omitnan'))
% plot(xxs,mean([CON{14,4}],2,'omitnan'))
% plot(xxs,mean([CON{19,4}],2,'omitnan'))
% plot(xxs,mean([CON{24,4}],2,'omitnan'))
% plot(xxs,mean([CON{29,4}],2,'omitnan'))
% plot(xxs,mean([CON{34,4}],2,'omitnan'))
% 
% 
% subplot(3,1,3)
% hold on
% 
% plot(xxs,mean([CON{4,8}],2,'omitnan'))
% plot(xxs,mean([CON{9,8}],2,'omitnan'))
% % fillLines(xxs,mean([CON{9,8}],2,'omitnan')-std([CON{9,8}],0,2,'omitnan'),mean([CON{9,8}],2,'omitnan')+std([CON{9,8}],0,2,'omitnan'),{[153/255 204/255 103/255]},0.25)
% plot(xxs,mean([CON{14,8}],2,'omitnan'))
% plot(xxs,mean([CON{19,8}],2,'omitnan'))
% plot(xxs,mean([CON{24,8}],2,'omitnan'))
% plot(xxs,mean([CON{29,8}],2,'omitnan'))
% plot(xxs,mean([CON{34,8}],2,'omitnan'))
% 
% figure(12334)
% subplot(3,2,1)
% hold on
% title('NW')
% ylim([-0.05 0.05])
% plot(xxs,mean([COM{1,1}],2,'omitnan'))
% plot(xxs,mean([COM{6,1}],2,'omitnan'))
% plot(xxs,mean([COM{11,1}],2,'omitnan'))
% plot(xxs,mean([COM{16,1}],2,'omitnan'))
% plot(xxs,mean([COM{21,1}],2,'omitnan'))
% plot(xxs,mean([COM{26,1}],2,'omitnan'))
% plot(xxs,mean([COM{31,1}],2,'omitnan'))
% % figure(2)
% subplot(3,2,2)
% hold on
% title('P1')
% ylim([-0.05 0.05])
% plot(xxs,mean([COM{2,1}],2,'omitnan'))
% plot(xxs,mean([COM{7,1}],2,'omitnan'))
% plot(xxs,mean([COM{12,1}],2,'omitnan'))
% plot(xxs,mean([COM{17,1}],2,'omitnan'))
% plot(xxs,mean([COM{22,1}],2,'omitnan'))
% plot(xxs,mean([COM{27,1}],2,'omitnan'))
% plot(xxs,mean([COM{32,1}],2,'omitnan'))
% 
% subplot(3,2,3)
% hold on
% title('P2')
% ylim([-0.05 0.05])
% plot(xxs,mean([COM{3,1}],2,'omitnan'))
% plot(xxs,mean([COM{8,1}],2,'omitnan'))
% % fillLines(xxs,mean([COM{9,8}],2,'omitnan')-std([CON{9,8}],0,2,'omitnan'),mean([CON{9,8}],2,'omitnan')+std([CON{9,8}],0,2,'omitnan'),{[153/255 204/255 103/255]},0.25)
% plot(xxs,mean([COM{13,1}],2,'omitnan'))
% plot(xxs,mean([COM{18,1}],2,'omitnan'))
% plot(xxs,mean([COM{23,1}],2,'omitnan'))
% plot(xxs,mean([COM{28,1}],2,'omitnan'))
% plot(xxs,mean([COM{33,1}],2,'omitnan'))
% 
% subplot(3,2,4)
% hold on
% title('P3')
% ylim([-0.05 0.05])
% plot(xxs,mean([COM{4,1}],2,'omitnan'))
% plot(xxs,mean([COM{9,1}],2,'omitnan'))
% plot(xxs,mean([COM{14,1}],2,'omitnan'))
% plot(xxs,mean([COM{19,1}],2,'omitnan'))
% plot(xxs,mean([COM{24,1}],2,'omitnan'))
% plot(xxs,mean([COM{29,1}],2,'omitnan'))
% plot(xxs,mean([COM{34,1}],2,'omitnan'))
% 
% subplot(3,2,5)
% hold on
% title('P4')
% ylim([-0.05 0.05])
% plot(xxs,mean([COM{5,1}],2,'omitnan'))
% plot(xxs,mean([COM{10,1}],2,'omitnan'))
% plot(xxs,mean([COM{15,1}],2,'omitnan'))
% plot(xxs,mean([COM{20,1}],2,'omitnan'))
% plot(xxs,mean([COM{25,1}],2,'omitnan'))
% plot(xxs,mean([COM{30,1}],2,'omitnan'))
% plot(xxs,mean([COM{35,1}],2,'omitnan'))

Hip_NW = [mean([HIP{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan'),mean([HIP{[1,6,11,16,21,26,31,36,41,46],2}],2,'omitnan'),...
    mean([HIP{[1,6,11,16,21,26,31,36,41,46],3}],2,'omitnan'),mean([HIP{[1,6,11,16,21,26,31,36,41,46],4}],2,'omitnan'),...
    mean([HIP{[1,6,11,16,21,26,31,36,41,46],5}],2,'omitnan'),mean([HIP{[1,6,11,16,21,26,31,36,41,46],6}],2,'omitnan')];
Hip_P1 = [mean([HIP{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan'),mean([HIP{[2,7,12,17,22,27,32,37,42,47],2}],2,'omitnan'),...
    mean([HIP{[2,7,12,17,22,27,32,37,42,47],3}],2,'omitnan'),mean([HIP{[2,7,12,17,22,27,32,37,42,47],4}],2,'omitnan'),...
    mean([HIP{[2,7,12,17,22,27,32,37,42,47],5}],2,'omitnan'),mean([HIP{[2,7,12,17,22,27,32,37,42,47],6}],2,'omitnan')];
Hip_P2 = [mean([HIP{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan'),mean([HIP{[3,8,13,18,23,28,33,38,43,48],2}],2,'omitnan'),...
    mean([HIP{[3,8,13,18,23,28,33,38,43,48],3}],2,'omitnan'),mean([HIP{[3,8,13,18,23,28,33,38,43,48],4}],2,'omitnan'),...
    mean([HIP{[3,8,13,18,23,28,33,38,43,48],5}],2,'omitnan'),mean([HIP{[3,8,13,18,23,28,33,38,43,48],6}],2,'omitnan')];
Hip_P3 = [mean([HIP{[4,9,14,19,24,29,34,39,43,49],1}],2,'omitnan'),mean([HIP{[4,9,14,19,24,29,34,39,43,49],2}],2,'omitnan'),...
    mean([HIP{[4,9,14,19,24,29,34,39,43,49],3}],2,'omitnan'),mean([HIP{[4,9,14,19,24,29,34,39,43,49],4}],2,'omitnan'),...
    mean([HIP{[4,9,14,19,24,29,34,39,43,49],5}],2,'omitnan'),mean([HIP{[4,9,14,19,24,29,34,39,43,49],6}],2,'omitnan')];
Hip_P4 = [mean([HIP{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan'),mean([HIP{[5,10,15,20,25,30,35,40,45,50],2}],2,'omitnan'),...
    mean([HIP{[5,10,15,20,25,30,35,40,45,50],3}],2,'omitnan'),mean([HIP{[5,10,15,20,25,30,35,40,45,50],4}],2,'omitnan'),...
    mean([HIP{[5,10,15,20,25,30,35,40,45,50],5}],2,'omitnan'),mean([HIP{[5,10,15,20,25,30,35,40,45,50],6}],2,'omitnan')];

Kne_NW = [mean([KNE{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan'),mean([KNE{[1,6,11,16,21,26,31,36,41,46],2}],2,'omitnan'),...
    mean([KNE{[1,6,11,16,21,26,31,36,41,46],3}],2,'omitnan')];
% Kne_P1 = [mean([KNE{2,1},KNE{7,1},KNE{12,1},KNE{17,1},KNE{22,1},KNE{27,1},KNE{32,1}],2,'omitnan'),...
%     mean([KNE{[2,7,12,17,22,27,32],2}],2,'omitnan'),mean([KNE{[2,7,12,17,22,27,32],3}],2,'omitnan')];
% Kne_P2 = [mean([KNE{3,1},KNE{8,1},KNE{13,1},KNE{18,1},KNE{23,1},KNE{28,1},KNE{33,1}],2,'omitnan'),...
%     mean([KNE{[3,8,13,18,23,28,33],2}],2,'omitnan'),mean([KNE{[3,8,13,18,23,28,33],3}],2,'omitnan')];
% Kne_P3 = [mean([KNE{4,1},KNE{9,1},KNE{14,1},KNE{19,1},KNE{24,1},KNE{29,1},KNE{34,1}],2,'omitnan'),...
%     mean([KNE{[4,9,14,19,24,29,34],2}],2,'omitnan'),mean([KNE{[4,9,14,19,24,29,34],3}],2,'omitnan')];
% Kne_P4 = [mean([KNE{5,1},KNE{10,1},KNE{15,1},KNE{20,1},KNE{25,1},KNE{30,1},KNE{35,1}],2,'omitnan'),...
%     mean([KNE{[5,10,15,20,25,30,35],2}],2,'omitnan'),mean([KNE{[5,10,15,20,25,30,35],3}],2,'omitnan')];
Kne_P1 = [mean([KNE{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan'),...
    mean([KNE{[2,7,12,17,22,27,32,37,42,47],2}],2,'omitnan'),mean([KNE{[2,7,12,17,22,27,32,37,42,47],3}],2,'omitnan')];
Kne_P2 = [mean([KNE{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan'),...
    mean([KNE{[3,8,13,18,23,28,33,38,43,48],2}],2,'omitnan'),mean([KNE{[3,8,13,18,23,28,33,38,43,48],3}],2,'omitnan')];
Kne_P3 = [mean([KNE{[4,9,14,19,24,29,34,39,44,49],1}],2,'omitnan'),...
    mean([KNE{[4,9,14,19,24,29,34,39,44,49],2}],2,'omitnan'),mean([KNE{[4,9,14,19,24,29,34,39,44,49],3}],2,'omitnan')];
Kne_P4 = [mean([KNE{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan'),...
    mean([KNE{[5,10,15,20,25,30,35,40,45,50],2}],2,'omitnan'),mean([KNE{[5,10,15,20,25,30,35,40,45,50],3}],2,'omitnan')];

RMET_NW = mean([RMET{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan');
RMET_P1 = mean([RMET{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan');
RMET_P2 = mean([RMET{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan');
RMET_P3 = mean([RMET{[4,9,14,19,24,29,34,39,44,49],1}],2,'omitnan');
RMET_P4 = mean([RMET{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan');


MOS_NW = [mean([MOS{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan'),mean([MOS{[1,6,11,16,21,26,31,36,41,46],2}],2,'omitnan')];
MOS_P1 = [mean([MOS{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan'),mean([MOS{[2,7,12,17,22,27,32,37,42,47],2}],2,'omitnan')];
MOS_P2 = [mean([MOS{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan'),mean([MOS{[3,8,13,18,23,28,33,38,43,48],2}],2,'omitnan')];
MOS_P3 = [mean([MOS{[4,9,14,19,24,29,34,39,44,49],1}],2,'omitnan'),mean([MOS{[4,9,14,19,24,29,34,39,44,49],2}],2,'omitnan')];
MOS_P4 = [mean([MOS{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan'),mean([MOS{[5,10,15,20,25,30,35,40,45,50],2}],2,'omitnan')];

MOS_min_NW = [mean([MOS{[1,6,11,16,21,26,31,36,41,46],3}],'omitnan')',std([MOS{[1,6,11,16,21,26,31,36,41,46],3}],'omitnan')',...
    mean([MOS{[1,6,11,16,21,26,31,36,41,46],4}],'omitnan')',std([MOS{[1,6,11,16,21,26,31,36,41,46],4}],'omitnan')'];
MOS_min_P1 = [mean([MOS{[2,7,12,17,22,27,32,37,42,47],3}],'omitnan')',std([MOS{[2,7,12,17,22,27,32,37,42,47],3}],'omitnan')',...
    mean([MOS{[2,7,12,17,22,27,32,37,42,47],4}],'omitnan')',std([MOS{[2,7,12,17,22,27,32,37,42,47],4}],'omitnan')'];
MOS_min_P2 = [mean([MOS{[3,8,13,18,23,28,33,38,43,48],3}],'omitnan')',std([MOS{[3,8,13,18,23,28,33,38,43,48],3}],'omitnan')',...
    mean([MOS{[2,7,12,17,22,27,32,38,43,48],4}],'omitnan')',std([MOS{[2,7,12,17,22,27,32,38,43,48],4}],'omitnan')'];
MOS_min_P3 = [mean([MOS{[4,9,14,19,24,29,34,39,44,49],3}],'omitnan')',std([MOS{[4,9,14,19,24,29,34,39,44,49],3}],'omitnan')',...
    mean([MOS{[2,7,12,17,22,27,32,39,44,49],4}],'omitnan')',std([MOS{[2,7,12,17,22,27,32,39,44,49],4}],'omitnan')'];
MOS_min_P4 = [mean([MOS{[5,10,15,20,25,30,35,40,45,50],3}],'omitnan')',std([MOS{[5,10,15,20,25,30,35,40,45,50],3}],'omitnan')',...
    mean([MOS{[2,7,12,17,22,27,32,40,45,50],4}],'omitnan')',std([MOS{[2,7,12,17,22,27,32,40,45,50],4}],'omitnan')'];

BX_min_NW = [(mean(MOS{1,6},2,'omitnan')),(mean(MOS{6,6},2,'omitnan')),(mean(MOS{11,6},2,'omitnan')),...
(mean(MOS{16,6},2,'omitnan')),(mean(MOS{21,6},2,'omitnan')),(mean(MOS{26,6},2,'omitnan')),(mean(MOS{31,6},2,'omitnan')),...
(mean(MOS{36,6},2,'omitnan')),(mean(MOS{41,6},2,'omitnan')),(mean(MOS{46,6},2,'omitnan'))];
BX_min_P1 = [(mean(MOS{2,6},2,'omitnan')),(mean(MOS{7,6},2,'omitnan')),(mean(MOS{12,6},2,'omitnan')),...
(mean(MOS{17,6},2,'omitnan')),(mean(MOS{22,6},2,'omitnan')),(mean(MOS{27,6},2,'omitnan')),(mean(MOS{32,6},2,'omitnan')),...
(mean(MOS{37,6},2,'omitnan')),(mean(MOS{42,6},2,'omitnan')),(mean(MOS{47,6},2,'omitnan'))];
BX_min_P2 =[(mean(MOS{3,6},2,'omitnan')),(mean(MOS{8,6},2,'omitnan')),(mean(MOS{13,6},2,'omitnan')),...
(mean(MOS{18,6},2,'omitnan')),(mean(MOS{23,6},2,'omitnan')),(mean(MOS{28,6},2,'omitnan')),(mean(MOS{33,6},2,'omitnan')),...
(mean(MOS{38,6},2,'omitnan')),(mean(MOS{43,6},2,'omitnan')),(mean(MOS{48,6},2,'omitnan'))];
BX_min_P3 = [(mean(MOS{4,6},2,'omitnan')),(mean(MOS{9,6},2,'omitnan')),(mean(MOS{14,6},2,'omitnan')),...
(mean(MOS{19,6},2,'omitnan')),(mean(MOS{24,6},2,'omitnan')),(mean(MOS{29,6},2,'omitnan')),(mean(MOS{34,6},2,'omitnan')),...
(mean(MOS{39,6},2,'omitnan')),(mean(MOS{44,6},2,'omitnan')),(mean(MOS{49,6},2,'omitnan'))];
BX_min_P4 = [(mean(MOS{5,6},2,'omitnan')),(mean(MOS{10,6},2,'omitnan')),(mean(MOS{15,6},2,'omitnan')),...
(mean(MOS{20,6},2,'omitnan')),(mean(MOS{25,6},2,'omitnan')),(mean(MOS{30,6},2,'omitnan')),(mean(MOS{35,6},2,'omitnan')),...
(mean(MOS{40,6},2,'omitnan')),(mean(MOS{45,6},2,'omitnan')),(mean(MOS{50,6},2,'omitnan'))];
BX_NW = mean(BX_min_NW)';
BX_P1 = mean(BX_min_P1)';
BX_P2 = mean(BX_min_P2)';
BX_P3 = mean(BX_min_P3)';
BX_P4 = mean(BX_min_P4)';
BX = [BX_NW,BX_P1,BX_P2,BX_P3,BX_P4];
% BX_min_P1 = [mean([MOS{[2,7,12,17,22,27,32],5}],2,'omitnan'),mean([MOS{[2,7,12,17,22,27,32],6}],2,'omitnan')];
% BX_min_P2 = [mean([MOS{[3,8,13,18,23,28,33],5}],2,'omitnan'),mean([MOS{[3,8,13,18,23,28,33],6}],2,'omitnan')];
% BX_min_P3 = [mean([MOS{[4,9,14,19,24,29,34],5}],2,'omitnan'),mean([MOS{[4,9,14,19,24,29,34],6}],2,'omitnan')];
% BX_min_P4 = [mean([MOS{[5,10,15,20,25,30,35],5}],2,'omitnan'),mean([MOS{[5,10,15,20,25,30,35],6}],2,'omitnan')];

% mean(BX_min_NW)

Time_NW_sts = [mean([Time{[1,6,11,16,21,26,31,36,41,46],1}],'omitnan')',std([Time{[1,6,11,16,21,26,31,36,41,46],1}],'omitnan')'];
Time_P1_sts = [mean([Time{[2,7,12,17,22,27,32,37,42,47],1}],'omitnan')',std([Time{[2,7,12,17,22,27,32,37,42,47],1}],'omitnan')'];
Time_P2_sts = [mean([Time{[3,8,13,18,23,28,33,38,43,48],1}],'omitnan')',std([Time{[3,8,13,18,23,28,33,38,43,48],1}],'omitnan')'];
Time_P3_sts = [mean([Time{[4,9,14,19,24,29,34,39,44,49],1}],'omitnan')',std([Time{[4,9,14,19,24,29,34,39,44,49],1}],'omitnan')'];
Time_P4_sts = [mean([Time{[5,10,15,20,25,30,35,40,45,50],1}],'omitnan')',std([Time{[5,10,15,20,25,30,35,40,45,50],1}],'omitnan')'];

Time_NW_sw = [mean([Time{[1,6,11,16,21,26,31,36,41,46],2}],'omitnan')',std([Time{[1,6,11,16,21,26,31,36,41,46],2}],'omitnan')'];
Time_P1_sw = [mean([Time{[2,7,12,17,22,27,32,37,42,47],2}],'omitnan')',std([Time{[2,7,12,17,22,27,32,37,42,47],2}],'omitnan')'];
Time_P2_sw = [mean([Time{[3,8,13,18,23,28,33,38,43,48],2}],'omitnan')',std([Time{[3,8,13,18,23,28,33,38,43,48],2}],'omitnan')'];
Time_P3_sw = [mean([Time{[4,9,14,19,24,29,34,39,44,49],2}],'omitnan')',std([Time{[4,9,14,19,24,29,34,39,44,49],2}],'omitnan')'];
Time_P4_sw = [mean([Time{[5,10,15,20,25,30,35,40,45,50],2}],'omitnan')',std([Time{[5,10,15,20,25,30,35,40,45,50],2}],'omitnan')'];

Time_DS.raw = [mean(Time{1,3},2,'omitnan'),mean(Time{6,3},2,'omitnan'),mean(Time{11,3},2,'omitnan'),...
    mean(Time{16,3},2,'omitnan'),mean(Time{21,3},2,'omitnan'),mean(Time{26,3},2,'omitnan'),mean(Time{31,3},2,'omitnan'),...
    mean(Time{36,3},2,'omitnan'),mean(Time{41,3},2,'omitnan'),mean(Time{46,3},2,'omitnan')];

Time_DS.NW.init.raw= [(mean(Time{1,4},2,'omitnan')),(mean(Time{6,4},2,'omitnan')),(mean(Time{11,4},2,'omitnan')),...
(mean(Time{16,4},2,'omitnan')),(mean(Time{21,4},2,'omitnan')),(mean(Time{26,4},2,'omitnan')),(mean(Time{31,4},2,'omitnan')),...
(mean(Time{36,4},2,'omitnan')),(mean(Time{41,4},2,'omitnan')),(mean(Time{46,4},2,'omitnan'));];
Time_DS.P1.init.raw= [(mean(Time{2,4},2,'omitnan')),(mean(Time{7,4},2,'omitnan')),(mean(Time{12,4},2,'omitnan')),...
(mean(Time{17,4},2,'omitnan')),(mean(Time{22,4},2,'omitnan')),(mean(Time{27,4},2,'omitnan')),(mean(Time{32,4},2,'omitnan')),...
(mean(Time{37,4},2,'omitnan')),(mean(Time{42,4},2,'omitnan')),(mean(Time{47,4},2,'omitnan'));];
Time_DS.P2.init.raw= [(mean(Time{3,4},2,'omitnan')),(mean(Time{8,4},2,'omitnan')),(mean(Time{13,4},2,'omitnan')),...
(mean(Time{18,4},2,'omitnan')),(mean(Time{23,4},2,'omitnan')),(mean(Time{28,4},2,'omitnan')),(mean(Time{33,4},2,'omitnan')),...
(mean(Time{38,4},2,'omitnan')),(mean(Time{43,4},2,'omitnan')),(mean(Time{48,4},2,'omitnan'));];
Time_DS.P3.init.raw= [(mean(Time{4,4},2,'omitnan')),(mean(Time{9,4},2,'omitnan')),(mean(Time{14,4},2,'omitnan')),...
(mean(Time{19,4},2,'omitnan')),(mean(Time{24,4},2,'omitnan')),(mean(Time{29,4},2,'omitnan')),(mean(Time{34,4},2,'omitnan')),...
(mean(Time{39,4},2,'omitnan')),(mean(Time{44,4},2,'omitnan')),(mean(Time{49,4},2,'omitnan'));];
Time_DS.P4.init.raw= [(mean(Time{5,4},2,'omitnan')),(mean(Time{10,4},2,'omitnan')),(mean(Time{15,4},2,'omitnan')),...
(mean(Time{20,4},2,'omitnan')),(mean(Time{25,4},2,'omitnan')),(mean(Time{30,4},2,'omitnan')),(mean(Time{35,4},2,'omitnan')),...
(mean(Time{40,4},2,'omitnan')),(mean(Time{45,4},2,'omitnan')),(mean(Time{50,4},2,'omitnan'));];

Time_DS.NW.init.mean= [mean(mean(Time{1,4},2,'omitnan')),mean(mean(Time{6,4},2,'omitnan')),mean(mean(Time{11,4},2,'omitnan')),...
mean(mean(Time{16,4},2,'omitnan')),mean(mean(Time{21,4},2,'omitnan')),mean(mean(Time{26,4},2,'omitnan')),mean(mean(Time{31,4},2,'omitnan')),...
mean(mean(Time{31,4},2,'omitnan')),mean(mean(Time{31,4},2,'omitnan')),mean(mean(Time{31,4},2,'omitnan'));]';
Time_DS.P1.init.mean= [mean(mean(Time{2,4},2,'omitnan')),mean(mean(Time{7,4},2,'omitnan')),mean(mean(Time{12,4},2,'omitnan')),...
mean(mean(Time{17,4},2,'omitnan')),mean(mean(Time{22,4},2,'omitnan')),mean(mean(Time{27,4},2,'omitnan')),mean(mean(Time{32,4},2,'omitnan')),...
mean(mean(Time{37,4},2,'omitnan')),mean(mean(Time{42,4},2,'omitnan')),mean(mean(Time{47,4},2,'omitnan'));]';
Time_DS.P2.init.mean= [mean(mean(Time{3,4},2,'omitnan')),mean(mean(Time{8,4},2,'omitnan')),mean(mean(Time{13,4},2,'omitnan')),...
mean(mean(Time{18,4},2,'omitnan')),mean(mean(Time{23,4},2,'omitnan')),mean(mean(Time{28,4},2,'omitnan')),mean(mean(Time{33,4},2,'omitnan')),...
mean(mean(Time{38,4},2,'omitnan')),mean(mean(Time{43,4},2,'omitnan')),mean(mean(Time{48,4},2,'omitnan'));]';
Time_DS.P3.init.mean= [mean(mean(Time{4,4},2,'omitnan')),mean(mean(Time{9,4},2,'omitnan')),mean(mean(Time{14,4},2,'omitnan')),...
mean(mean(Time{19,4},2,'omitnan')),mean(mean(Time{24,4},2,'omitnan')),mean(mean(Time{29,4},2,'omitnan')),mean(mean(Time{34,4},2,'omitnan')),...
mean(mean(Time{39,4},2,'omitnan')),mean(mean(Time{44,4},2,'omitnan')),mean(mean(Time{49,4},2,'omitnan'));]';
Time_DS.P4.init.mean= [mean(mean(Time{5,4},2,'omitnan')),mean(mean(Time{10,4},2,'omitnan')),mean(mean(Time{15,4},2,'omitnan')),...
mean(mean(Time{20,4},2,'omitnan')),mean(mean(Time{25,4},2,'omitnan')),mean(mean(Time{30,4},2,'omitnan')),mean(mean(Time{35,4},2,'omitnan')),...
mean(mean(Time{40,4},2,'omitnan')),mean(mean(Time{45,4},2,'omitnan')),mean(mean(Time{50,4},2,'omitnan'));]';



Time_DS.NW.term1.raw= [(mean(Time{1,5},2,'omitnan')),(mean(Time{6,5},2,'omitnan')),(mean(Time{11,5},2,'omitnan')),...
(mean(Time{16,5},2,'omitnan')),(mean(Time{21,5},2,'omitnan')),(mean(Time{26,5},2,'omitnan')),(mean(Time{31,5},2,'omitnan')),...
(mean(Time{36,5},2,'omitnan')),(mean(Time{41,5},2,'omitnan')),(mean(Time{46,5},2,'omitnan'));];
Time_DS.P1.term1.raw= [(mean(Time{2,5},2,'omitnan')),(mean(Time{7,5},2,'omitnan')),(mean(Time{12,5},2,'omitnan')),...
(mean(Time{17,5},2,'omitnan')),(mean(Time{22,5},2,'omitnan')),(mean(Time{27,5},2,'omitnan')),(mean(Time{32,5},2,'omitnan')),...
(mean(Time{37,5},2,'omitnan')),(mean(Time{42,5},2,'omitnan')),(mean(Time{47,5},2,'omitnan'));];
Time_DS.P2.term1.raw= [(mean(Time{3,5},2,'omitnan')),(mean(Time{8,5},2,'omitnan')),(mean(Time{13,5},2,'omitnan')),...
(mean(Time{18,5},2,'omitnan')),(mean(Time{23,5},2,'omitnan')),(mean(Time{28,5},2,'omitnan')),(mean(Time{33,5},2,'omitnan')),...
(mean(Time{38,5},2,'omitnan')),(mean(Time{43,5},2,'omitnan')),(mean(Time{48,5},2,'omitnan'));];
Time_DS.P3.term1.raw= [(mean(Time{4,5},2,'omitnan')),(mean(Time{9,5},2,'omitnan')),(mean(Time{14,5},2,'omitnan')),...
(mean(Time{19,5},2,'omitnan')),(mean(Time{24,5},2,'omitnan')),(mean(Time{29,5},2,'omitnan')),(mean(Time{34,5},2,'omitnan')),...
(mean(Time{39,5},2,'omitnan')),(mean(Time{44,5},2,'omitnan')),(mean(Time{49,5},2,'omitnan'));];
Time_DS.P4.term1.raw= [(mean(Time{5,5},2,'omitnan')),(mean(Time{10,5},2,'omitnan')),(mean(Time{15,5},2,'omitnan')),...
(mean(Time{20,5},2,'omitnan')),(mean(Time{25,5},2,'omitnan')),(mean(Time{30,5},2,'omitnan')),(mean(Time{35,5},2,'omitnan')),...
(mean(Time{40,5},2,'omitnan')),(mean(Time{45,5},2,'omitnan')),(mean(Time{50,5},2,'omitnan'));];

Time_DS.NW.term1.mean= [mean(mean(Time{1,5},2,'omitnan')),mean(mean(Time{6,5},2,'omitnan')),mean(mean(Time{11,5},2,'omitnan')),...
mean(mean(Time{16,5},2,'omitnan')),mean(mean(Time{21,5},2,'omitnan')),mean(mean(Time{26,5},2,'omitnan')),mean(mean(Time{31,5},2,'omitnan')),...
mean(mean(Time{36,5},2,'omitnan')),mean(mean(Time{41,5},2,'omitnan')),mean(mean(Time{46,5},2,'omitnan'));]';
Time_DS.P1.term1.mean= [mean(mean(Time{2,5},2,'omitnan')),mean(mean(Time{7,5},2,'omitnan')),mean(mean(Time{12,5},2,'omitnan')),...
mean(mean(Time{17,5},2,'omitnan')),mean(mean(Time{22,5},2,'omitnan')),mean(mean(Time{27,5},2,'omitnan')),mean(mean(Time{32,5},2,'omitnan')),...
mean(mean(Time{37,5},2,'omitnan')),mean(mean(Time{42,5},2,'omitnan')),mean(mean(Time{47,5},2,'omitnan'));]';
Time_DS.P2.term1.mean= [mean(mean(Time{3,5},2,'omitnan')),mean(mean(Time{8,5},2,'omitnan')),mean(mean(Time{13,5},2,'omitnan')),...
mean(mean(Time{18,5},2,'omitnan')),mean(mean(Time{23,5},2,'omitnan')),mean(mean(Time{28,5},2,'omitnan')),mean(mean(Time{33,5},2,'omitnan')),...
mean(mean(Time{38,5},2,'omitnan')),mean(mean(Time{43,5},2,'omitnan')),mean(mean(Time{48,5},2,'omitnan'));]';
Time_DS.P3.term1.mean= [mean(mean(Time{4,5},2,'omitnan')),mean(mean(Time{9,5},2,'omitnan')),mean(mean(Time{14,5},2,'omitnan')),...
mean(mean(Time{19,5},2,'omitnan')),mean(mean(Time{24,5},2,'omitnan')),mean(mean(Time{29,5},2,'omitnan')),mean(mean(Time{34,5},2,'omitnan')),...
mean(mean(Time{39,5},2,'omitnan')),mean(mean(Time{44,5},2,'omitnan')),mean(mean(Time{49,5},2,'omitnan'));]';
Time_DS.P4.term1.mean= [mean(mean(Time{5,5},2,'omitnan')),mean(mean(Time{10,5},2,'omitnan')),mean(mean(Time{15,5},2,'omitnan')),...
mean(mean(Time{20,5},2,'omitnan')),mean(mean(Time{25,5},2,'omitnan')),mean(mean(Time{30,5},2,'omitnan')),mean(mean(Time{35,5},2,'omitnan')),...
mean(mean(Time{40,5},2,'omitnan')),mean(mean(Time{45,5},2,'omitnan')),mean(mean(Time{50,5},2,'omitnan'));]';


Time_DS.NW.term2.raw= [(mean(Time{1,6},2,'omitnan')),(mean(Time{6,6},2,'omitnan')),(mean(Time{11,6},2,'omitnan')),...
(mean(Time{16,6},2,'omitnan')),(mean(Time{21,6},2,'omitnan')),(mean(Time{26,6},2,'omitnan')),(mean(Time{31,6},2,'omitnan')),...
(mean(Time{36,6},2,'omitnan')),(mean(Time{41,6},2,'omitnan')),(mean(Time{46,6},2,'omitnan'));];
Time_DS.P1.term2.raw= [(mean(Time{2,6},2,'omitnan')),(mean(Time{7,6},2,'omitnan')),(mean(Time{12,6},2,'omitnan')),...
(mean(Time{17,6},2,'omitnan')),(mean(Time{22,6},2,'omitnan')),(mean(Time{27,6},2,'omitnan')),(mean(Time{32,6},2,'omitnan')),...
(mean(Time{37,6},2,'omitnan')),(mean(Time{42,6},2,'omitnan')),(mean(Time{47,6},2,'omitnan'));];
Time_DS.P2.term2.raw= [(mean(Time{3,6},2,'omitnan')),(mean(Time{8,6},2,'omitnan')),(mean(Time{13,6},2,'omitnan')),...
(mean(Time{18,6},2,'omitnan')),(mean(Time{23,6},2,'omitnan')),(mean(Time{28,6},2,'omitnan')),(mean(Time{33,6},2,'omitnan')),...
(mean(Time{38,6},2,'omitnan')),(mean(Time{43,6},2,'omitnan')),(mean(Time{48,6},2,'omitnan'));];
Time_DS.P3.term2.raw= [(mean(Time{4,6},2,'omitnan')),(mean(Time{9,6},2,'omitnan')),(mean(Time{14,6},2,'omitnan')),...
(mean(Time{19,6},2,'omitnan')),(mean(Time{24,6},2,'omitnan')),(mean(Time{29,6},2,'omitnan')),(mean(Time{34,6},2,'omitnan')),...
(mean(Time{39,6},2,'omitnan')),(mean(Time{44,6},2,'omitnan')),(mean(Time{49,6},2,'omitnan'));];
Time_DS.P4.term2.raw= [(mean(Time{5,6},2,'omitnan')),(mean(Time{10,6},2,'omitnan')),(mean(Time{15,6},2,'omitnan')),...
(mean(Time{20,6},2,'omitnan')),(mean(Time{25,6},2,'omitnan')),(mean(Time{30,6},2,'omitnan')),(mean(Time{35,6},2,'omitnan')),...
(mean(Time{40,6},2,'omitnan')),(mean(Time{45,6},2,'omitnan')),(mean(Time{50,6},2,'omitnan'));];

Time_DS.NW.term2.mean= [mean(mean(Time{1,6},2,'omitnan')),mean(mean(Time{6,6},2,'omitnan')),mean(mean(Time{11,6},2,'omitnan')),...
mean(mean(Time{16,6},2,'omitnan')),mean(mean(Time{21,6},2,'omitnan')),mean(mean(Time{26,6},2,'omitnan')),mean(mean(Time{31,6},2,'omitnan')),...
mean(mean(Time{36,6},2,'omitnan')),mean(mean(Time{41,6},2,'omitnan')),mean(mean(Time{46,6},2,'omitnan'));]';
Time_DS.P1.term2.mean= [mean(mean(Time{2,6},2,'omitnan')),mean(mean(Time{7,6},2,'omitnan')),mean(mean(Time{12,6},2,'omitnan')),...
mean(mean(Time{17,6},2,'omitnan')),mean(mean(Time{22,6},2,'omitnan')),mean(mean(Time{27,6},2,'omitnan')),mean(mean(Time{32,6},2,'omitnan')),...
mean(mean(Time{37,6},2,'omitnan')),mean(mean(Time{42,6},2,'omitnan')),mean(mean(Time{47,6},2,'omitnan'));]';
Time_DS.P2.term2.mean= [mean(mean(Time{3,6},2,'omitnan')),mean(mean(Time{8,6},2,'omitnan')),mean(mean(Time{13,6},2,'omitnan')),...
mean(mean(Time{18,6},2,'omitnan')),mean(mean(Time{23,6},2,'omitnan')),mean(mean(Time{28,6},2,'omitnan')),mean(mean(Time{33,6},2,'omitnan')),...
mean(mean(Time{38,6},2,'omitnan')),mean(mean(Time{43,6},2,'omitnan')),mean(mean(Time{48,6},2,'omitnan'));]';
Time_DS.P3.term2.mean= [mean(mean(Time{4,6},2,'omitnan')),mean(mean(Time{9,6},2,'omitnan')),mean(mean(Time{14,6},2,'omitnan')),...
mean(mean(Time{19,6},2,'omitnan')),mean(mean(Time{24,6},2,'omitnan')),mean(mean(Time{29,6},2,'omitnan')),mean(mean(Time{34,6},2,'omitnan')),...
mean(mean(Time{39,6},2,'omitnan')),mean(mean(Time{44,6},2,'omitnan')),mean(mean(Time{49,6},2,'omitnan'));]';
Time_DS.P4.term2.mean= [mean(mean(Time{5,6},2,'omitnan')),mean(mean(Time{10,6},2,'omitnan')),mean(mean(Time{15,6},2,'omitnan')),...
mean(mean(Time{20,6},2,'omitnan')),mean(mean(Time{25,6},2,'omitnan')),mean(mean(Time{30,6},2,'omitnan')),mean(mean(Time{35,6},2,'omitnan')),...
mean(mean(Time{40,6},2,'omitnan')),mean(mean(Time{45,6},2,'omitnan')),mean(mean(Time{50,6},2,'omitnan'));]';


max_force = [max(mean(CON{1,3},2)),max(mean(CON{2,3},2)),max(mean(CON{3,3},2)),max(mean(CON{4,3},2)),max(mean(CON{5,3},2));...
    max(mean(CON{6,3},2)),max(mean(CON{7,3},2)),max(mean(CON{8,3},2)),max(mean(CON{9,3},2)),max(mean(CON{10,3},2));...
    max(mean(CON{11,3},2)),max(mean(CON{12,3},2)),max(mean(CON{13,3},2)),max(mean(CON{14,3},2)),max(mean(CON{15,3},2));...
    max(mean(CON{16,3},2)),max(mean(CON{17,3},2)),max(mean(CON{18,3},2)),max(mean(CON{19,3},2)),max(mean(CON{20,3},2));...
    max(mean(CON{21,3},2)),max(mean(CON{22,3},2)),max(mean(CON{23,3},2)),max(mean(CON{24,3},2)),max(mean(CON{25,3},2));...
    max(mean(CON{26,3},2)),max(mean(CON{27,3},2)),max(mean(CON{28,3},2)),max(mean(CON{29,3},2)),max(mean(CON{30,3},2));...
    max(mean(CON{31,3},2)),max(mean(CON{32,3},2)),max(mean(CON{33,3},2)),max(mean(CON{34,3},2)),max(mean(CON{35,3},2));...
    max(mean(CON{36,3},2)),max(mean(CON{37,3},2)),max(mean(CON{38,3},2)),max(mean(CON{39,3},2)),max(mean(CON{40,3},2));...
    max(mean(CON{41,3},2)),max(mean(CON{42,3},2)),max(mean(CON{43,3},2)),max(mean(CON{44,3},2)),max(mean(CON{45,3},2));...
    max(mean(CON{46,3},2)),max(mean(CON{47,3},2)),max(mean(CON{48,3},2)),max(mean(CON{49,3},2)),max(mean(CON{50,3},2))
];



COM_Normalwalking = [mean([COM{[1,6,11,16,21,26,31,36,41,46],1}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],2}],2,'omitnan'),...
    mean([COM{[1,6,11,16,21,26,31,36,41,46],3}],2,'omitnan')];
COM_profile1 = [mean([COM{[2,7,12,17,22,27,32,37,42,47],1}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],2}],2,'omitnan'),...
    mean([COM{[2,7,12,17,22,27,32,37,42,47],3}],2,'omitnan')];
COM_profile2 = [mean([COM{[3,8,13,18,23,28,33,38,43,48],1}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],2}],2,'omitnan'),...
    mean([COM{[3,8,13,18,23,28,33,38,43,48],3}],2,'omitnan')];
COM_profile3 = [mean([COM{[4,9,14,19,24,29,34,39,44,49],1}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],2}],2,'omitnan'),...
    mean([COM{[4,9,14,19,24,29,34,39,44,49],3}],2,'omitnan')];
COM_profile4 = [mean([COM{[5,10,15,20,25,30,35,40,45,50],1}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],2}],2,'omitnan'),...
    mean([COM{[5,10,15,20,25,30,35,40,45,50],3}],2,'omitnan')];

max_com = [max(mean(COM{1,1},2)),max(mean(COM{2,1},2)),max(mean(COM{3,1},2)),max(mean(COM{4,1},2)),max(mean(COM{5,1},2));...
    max(mean(COM{6,1},2)),max(mean(COM{7,1},2)),max(mean(COM{8,1},2)),max(mean(COM{9,1},2)),max(mean(COM{10,1},2));...
    max(mean(COM{11,1},2)),max(mean(COM{12,1},2)),max(mean(COM{13,1},2)),max(mean(COM{14,1},2)),max(mean(COM{15,1},2));...
    max(mean(COM{16,1},2)),max(mean(COM{17,1},2)),max(mean(COM{18,1},2)),max(mean(COM{19,1},2)),max(mean(COM{20,1},2));...
    max(mean(COM{21,1},2)),max(mean(COM{22,1},2)),max(mean(COM{23,1},2)),max(mean(COM{24,1},2)),max(mean(COM{25,1},2));...
    max(mean(COM{26,1},2)),max(mean(COM{27,1},2)),max(mean(COM{28,1},2)),max(mean(COM{29,1},2)),max(mean(COM{30,1},2));...
    max(mean(COM{31,1},2)),max(mean(COM{32,1},2)),max(mean(COM{33,1},2)),max(mean(COM{34,1},2)),max(mean(COM{35,1},2));...
    max(mean(COM{36,1},2)),max(mean(COM{37,1},2)),max(mean(COM{38,1},2)),max(mean(COM{39,1},2)),max(mean(COM{40,1},2));...
    max(mean(COM{41,1},2)),max(mean(COM{42,1},2)),max(mean(COM{43,1},2)),max(mean(COM{44,1},2)),max(mean(COM{45,1},2));...
    max(mean(COM{46,1},2)),max(mean(COM{47,1},2)),max(mean(COM{48,1},2)),max(mean(COM{49,1},2)),max(mean(COM{50,1},2))
];
    

COM_Normalwalking_std = [std([COM{[1,6,11,16,21,26,31,36,41,46],1}],0,2,'omitnan'),std([COM{[1,6,11,16,21,26,31,36,41,46],2}],0,2,'omitnan'),...
    std([COM{[1,6,11,16,21,26,31,36,41,46],3}],0,2,'omitnan')];
COM_profile1_std = [std([COM{[2,7,12,17,22,27,32,37,42,47],1}],0,2,'omitnan'),std([COM{[2,7,12,17,22,27,32,37,42,47],2}],0,2,'omitnan'),...
    std([COM{[2,7,12,17,22,27,32,37,42,47],3}],0,2,'omitnan')];
COM_profile2_std = [std([COM{[3,8,13,18,23,28,33,38,43,48],1}],0,2,'omitnan'),std([COM{[3,8,13,18,23,28,33,38,43,48],2}],0,2,'omitnan'),...
    std([COM{[3,8,13,18,23,28,33,38,43,48],3}],0,2,'omitnan')];
COM_profile3_std = [std([COM{[4,9,14,19,24,29,34,39,44,49],1}],0,2,'omitnan'),std([COM{[4,9,14,19,24,29,34,39,44,49],2}],0,2,'omitnan'),...
    std([COM{[4,9,14,19,24,29,34,39,44,49],3}],0,2,'omitnan')];
COM_profile4_std = [std([COM{[5,10,15,20,25,30,35,40,45,50],1}],0,2,'omitnan'),std([COM{[5,10,15,20,25,30,35,40,45,50],2}],0,2,'omitnan'),...
    std([COM{[5,10,15,20,25,30,35,40,45,50],3}],0,2,'omitnan')];

COM_vel_Normalwalking = [mean([COM{[1,6,11,16,21,26,31,36,41,46],4}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],5}],2,'omitnan'),...
    mean([COM{[1,6,11,16,21,26,31,36,41,46],6}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],7}],2,'omitnan'),...
    mean([COM{[1,6,11,16,21,26,31,36,41,46],12}],2,'omitnan')];
COM_vel_profile1 = [mean([COM{[2,7,12,17,22,27,32,37,42,47],4}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],5}],2,'omitnan'),...
    mean([COM{[2,7,12,17,22,27,32,37,42,47],6}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],7}],2,'omitnan'),...
    mean([COM{[2,7,12,17,22,27,32,37,42,47],12}],2,'omitnan')];
COM_vel_profile2 = [mean([COM{[3,8,13,18,23,28,33,38,43,48],4}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],5}],2,'omitnan'),...
    mean([COM{[3,8,13,18,23,28,33,38,43,48],6}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],7}],2,'omitnan'),...
    mean([COM{[3,8,13,18,23,28,33,38,43,48],12}],2,'omitnan')];
COM_vel_profile3 = [mean([COM{[4,9,14,19,24,29,34,39,44,49],4}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],5}],2,'omitnan'),...
    mean([COM{[4,9,14,19,24,29,34,39,44,49],6}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],7}],2,'omitnan'),...
    mean([COM{[4,9,14,19,24,29,34,39,44,49],12}],2,'omitnan')];
COM_vel_profile4 = [mean([COM{[5,10,15,20,25,30,35,40,45,50],4}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],5}],2,'omitnan'),...
    mean([COM{[5,10,15,20,25,30,35,40,45,50],6}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],7}],2,'omitnan'),...
    mean([COM{[5,10,15,20,25,30,35,40,45,50],12}],2,'omitnan')];

COM_vel_Normalwalking_std = [std([COM{[1,6,11,16,21,26,31,36,41,46],4}],0,2,'omitnan'),std([COM{[1,6,11,16,21,26,31,36,41,46],5}],0,2,'omitnan'),...
    std([COM{[1,6,11,16,21,26,31,36,41,46],6}],0,2,'omitnan'),std([COM{[1,6,11,16,21,26,31,36,41,46],7}],0,2,'omitnan'),...
    std([COM{[1,6,11,16,21,26,31,36,41,46],12}],0,2,'omitnan')];
COM_vel_profile1_std = [std([COM{[2,7,12,17,22,27,32,37,42,47],4}],0,2,'omitnan'),std([COM{[2,7,12,17,22,27,32,37,42,47],5}],0,2,'omitnan'),...
    std([COM{[2,7,12,17,22,27,32,37,42,47],6}],0,2,'omitnan'),std([COM{[2,7,12,17,22,27,32,37,42,47],7}],0,2,'omitnan'),...
    std([COM{[2,7,12,17,22,27,32,37,42,47],12}],0,2,'omitnan')];
COM_vel_profile2_std = [std([COM{[3,8,13,18,23,28,33,38,43,48],4}],0,2,'omitnan'),std([COM{[3,8,13,18,23,28,33,38,43,48],5}],0,2,'omitnan'),...
    std([COM{[3,8,13,18,23,28,33,38,43,48],6}],0,2,'omitnan'),std([COM{[3,8,13,18,23,28,33,38,43,48],7}],0,2,'omitnan'),...
    std([COM{[3,8,13,18,23,28,33,38,43,48],12}],0,2,'omitnan')];
COM_vel_profile3_std = [std([COM{[4,9,14,19,24,29,34,39,44,49],4}],0,2,'omitnan'),std([COM{[4,9,14,19,24,29,34,39,44,49],5}],0,2,'omitnan'),...
    std([COM{[4,9,14,19,24,29,34,39,44,49],6}],0,2,'omitnan'),std([COM{[4,9,14,19,24,29,34,39,44,49],7}],0,2,'omitnan'),...
    std([COM{[4,9,14,19,24,29,34,39,44,49],12}],0,2,'omitnan')];
COM_vel_profile4_std = [std([COM{[5,10,15,20,25,30,35,40,45,50],4}],0,2,'omitnan'),std([COM{[5,10,15,20,25,30,35,40,45,50],5}],0,2,'omitnan'),...
    std([COM{[5,10,15,20,25,30,35,40,45,50],6}],0,2,'omitnan'),std([COM{[5,10,15,20,25,30,35,40,45,50],7}],0,2,'omitnan'),...
    std([COM{[5,10,15,20,25,30,35,40,45,50],12}],0,2,'omitnan')];

COM_acc_Normalwalking = [mean([COM{[1,6,11,16,21,26,31,36,41,46],13}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],14}],2,'omitnan'),...
    mean([COM{[1,6,11,16,21,26,31,36,41,46],15}],2,'omitnan')];
COM_acc_profile1 = [mean([COM{[2,7,12,17,22,27,32,37,42,47],13}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],14}],2,'omitnan'),...
    mean([COM{[2,7,12,17,22,27,32,37,42,47],15}],2,'omitnan')];
COM_acc_profile2 = [mean([COM{[3,8,13,18,23,28,33,38,43,48],13}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],14}],2,'omitnan'),...
    mean([COM{[3,8,13,18,23,28,33,38,43,48],15}],2,'omitnan')];
COM_acc_profile3 = [mean([COM{[4,9,14,19,24,29,34,39,44,49],13}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],14}],2,'omitnan'),...
    mean([COM{[4,9,14,19,24,29,34,39,44,49],15}],2,'omitnan')];
COM_acc_profile4 = [mean([COM{[5,10,15,20,25,30,35,40,45,50],13}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],14}],2,'omitnan'),...
    mean([COM{[5,10,15,20,25,30,35,40,45,50],15}],2,'omitnan')];

XCOM_Normalwalking = [mean([COM{[1,6,11,16,21,26,31,36,41,46],8}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],9}],2,'omitnan'),...
    mean([COM{[1,6,11,16,21,26,31,36,41,46],10}],2,'omitnan'),mean([COM{[1,6,11,16,21,26,31,36,41,46],11}],2,'omitnan')];
XCOM_profile1 = [mean([COM{[2,7,12,17,22,27,32,37,42,47],8}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],9}],2,'omitnan'),...
    mean([COM{[2,7,12,17,22,27,32,37,42,47],10}],2,'omitnan'),mean([COM{[2,7,12,17,22,27,32,37,42,47],11}],2,'omitnan')];
XCOM_profile2 = [mean([COM{[3,8,13,18,23,28,33,38,43,48],8}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],9}],2,'omitnan'),...
    mean([COM{[3,8,13,18,23,28,33,38,43,48],10}],2,'omitnan'),mean([COM{[3,8,13,18,23,28,33,38,43,48],11}],2,'omitnan')];
XCOM_profile3 = [mean([COM{[4,9,14,19,24,29,34,39,44,49],8}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],9}],2,'omitnan'),...
    mean([COM{[4,9,14,19,24,29,34,39,44,49],10}],2,'omitnan'),mean([COM{[4,9,14,19,24,29,34,39,44,49],11}],2,'omitnan')];
XCOM_profile4 = [mean([COM{[5,10,15,20,25,30,35,40,45,50],8}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],9}],2,'omitnan'),...
    mean([COM{[5,10,15,20,25,30,35,40,45,50],10}],2,'omitnan'),mean([COM{[5,10,15,20,25,30,35,40,45,50],11}],2,'omitnan')];

XCOM_Normalwalking_std = [std([COM{[1,6,11,16,21,26,31,36,41,46],8}],0,2,'omitnan'),std([COM{[1,6,11,16,21,26,31,36,41,46],9}],0,2,'omitnan'),...
    std([COM{[1,6,11,16,21,26,31,36,41,46],10}],0,2,'omitnan'),std([COM{[1,6,11,16,21,26,31,36,41,46],11}],0,2,'omitnan')];
XCOM_profile1_std = [std([COM{[2,7,12,17,22,27,32,37,42,47],8}],0,2,'omitnan'),std([COM{[2,7,12,17,22,27,32,37,42,47],9}],0,2,'omitnan'),...
    std([COM{[2,7,12,17,22,27,32,37,42,47],10}],0,2,'omitnan'),std([COM{[2,7,12,17,22,27,32,37,42,47],11}],0,2,'omitnan')];
XCOM_profile2_std = [std([COM{[3,8,13,18,23,28,33,38,43,48],8}],0,2,'omitnan'),std([COM{[3,8,13,18,23,28,33,38,43,48],9}],0,2,'omitnan'),...
    std([COM{[3,8,13,18,23,28,33,38,43,48],10}],0,2,'omitnan'),std([COM{[3,8,13,18,23,28,33,38,43,48],11}],0,2,'omitnan')];
XCOM_profile3_std = [std([COM{[4,9,14,19,24,29,34,39,44,49],8}],0,2,'omitnan'),std([COM{[4,9,14,19,24,29,34,39,44,49],9}],0,2,'omitnan'),...
    std([COM{[4,9,14,19,24,29,34,39,44,49],10}],0,2,'omitnan'),std([COM{[4,9,14,19,24,29,34,39,44,49],11}],0,2,'omitnan')];
XCOM_profile4_std = [std([COM{[5,10,15,20,25,30,35,40,45,50],8}],0,2,'omitnan'),std([COM{[5,10,15,20,25,30,35,40,45,50],9}],0,2,'omitnan'),...
    std([COM{[5,10,15,20,25,30,35,40,45,50],10}],0,2,'omitnan'),std([COM{[5,10,15,20,25,30,35,40,45,50],11}],0,2,'omitnan')];

for ee = 1:10
Disp(ee,1) =norm(COM_Normalwalking(round((Time_DS.NW.term2.mean(ee,1))),:) - COM_Normalwalking(round(Time_DS.NW.term1.mean(ee,1)),:));
Disp(ee,2) =norm(COM_profile1(round((Time_DS.P1.term2.mean(ee,1))),:) - COM_profile1(round(Time_DS.P1.term1.mean(ee,1)),:));
Disp(ee,3) =norm(COM_profile2(round((Time_DS.P2.term2.mean(ee,1))),:) - COM_profile2(round(Time_DS.P2.term1.mean(ee,1)),:));
Disp(ee,4) =norm(COM_profile3(round((Time_DS.P3.term2.mean(ee,1))),:) - COM_profile3(round(Time_DS.P3.term1.mean(ee,1)),:));
Disp(ee,5) =norm(COM_profile4(round((Time_DS.P4.term2.mean(ee,1))),:) - COM_profile4(round(Time_DS.P4.term1.mean(ee,1)),:));
end


W_NW = [mean([W{[1,6,11,16,21,26,31,36,41,46],3}],'omitnan')',mean([W{[1,6,11,16,21,26,31,36,41,46],6}],'omitnan')'];
W_P1 = [mean([W{[2,7,12,17,22,27,32,37,42,47],3}],'omitnan')',mean([W{[2,7,12,17,22,27,32,37,42,47],6}],'omitnan')'];
W_P2 = [mean([W{[3,8,13,18,23,28,33,38,43,48],3}],'omitnan')',mean([W{[3,8,13,18,23,28,33,38,43,48],6}],'omitnan')'];
W_P3 = [mean([W{[4,9,14,19,24,29,34,39,44,49],3}],'omitnan')',mean([W{[4,9,14,19,24,29,34,39,44,49],6}],'omitnan')'];
W_P4 = [mean([W{[5,10,15,20,25,30,35,40,45,50],3}],'omitnan')',mean([W{[5,10,15,20,25,30,35,40,45,50],6}],'omitnan')'];

W_del_NW = [mean([W{[1,6,11,16,21,26,31,36,41,46],2}],'omitnan')'];
W_del_P1 = [mean([W{[2,7,12,17,22,27,32,37,42,47],2}],'omitnan')'];
W_del_P2 = [mean([W{[3,8,13,18,23,28,33,38,43,48],2}],'omitnan')'];
W_del_P3 = [mean([W{[4,9,14,19,24,29,34,39,44,49],2}],'omitnan')'];
W_del_P4 = [mean([W{[5,10,15,20,25,30,35,40,45,50],2}],'omitnan')'];

W_del_NW_2 = [W_del_NW(1:2:19,1) , W_del_NW(2:2:20,1)];
W_del_P1_2 = [W_del_P1(1:2:19,1) , W_del_P1(2:2:20,1)];
W_del_P2_2 = [W_del_P2(1:2:19,1) , W_del_P2(2:2:20,1)];
W_del_P3_2 = [W_del_P3(1:2:19,1) , W_del_P3(2:2:20,1)];
W_del_P4_2 = [W_del_P4(1:2:19,1) , W_del_P4(2:2:20,1)];

W_pre_NW = [mean([W{[1,6,11,16,21,26,31,36,41,46],1}],'omitnan')'];
W_pre_P1 = [mean([W{[2,7,12,17,22,27,32,37,42,47],1}],'omitnan')'];
W_pre_P2 = [mean([W{[3,8,13,18,23,28,33,38,43,48],1}],'omitnan')'];
W_pre_P3 = [mean([W{[4,9,14,19,24,29,34,39,44,49],1}],'omitnan')'];
W_pre_P4 = [mean([W{[5,10,15,20,25,30,35,40,45,50],1}],'omitnan')'];

W_pre_NW_2 = [W_pre_NW(1:2:19,1) , W_pre_NW(2:2:20,1)];
W_pre_P1_2 = [W_pre_P1(1:2:19,1) , W_pre_P1(2:2:20,1)];
W_pre_P2_2 = [W_pre_P2(1:2:19,1) , W_pre_P2(2:2:20,1)];
W_pre_P3_2 = [W_pre_P3(1:2:19,1) , W_pre_P3(2:2:20,1)];
W_pre_P4_2 = [W_pre_P4(1:2:19,1) , W_pre_P4(2:2:20,1)];


W_total = [W_NW(:,1),W_P1(:,1),W_P2(:,1),W_P3(:,1),W_P4(:,1)];
W_total2 = [W_NW(:,2),W_P1(:,2),W_P2(:,2),W_P3(:,2),W_P4(:,2)];
W_del_y = [W_del_NW_2(:,1),W_del_P1_2(:,1),W_del_P2_2(:,1),W_del_P3_2(:,1),W_del_P4_2(:,1)];
W_del_z = [W_del_NW_2(:,2),W_del_P1_2(:,2),W_del_P2_2(:,2),W_del_P3_2(:,2),W_del_P4_2(:,2)];
W_pre_y = [W_pre_NW_2(:,1),W_pre_P1_2(:,1),W_pre_P2_2(:,1),W_pre_P3_2(:,1),W_pre_P4_2(:,1)];
W_pre_z = [W_pre_NW_2(:,2),W_pre_P1_2(:,2),W_pre_P2_2(:,2),W_pre_P3_2(:,2),W_pre_P4_2(:,2)];

w1 = [W_NW;W_P1;W_P2;W_P3;W_P4];
w2 = [4.128;3.966;4.237;3.841;4.452;4.793;3.292;4.697;4.681;4.719;4.580;3.837;4.763;3.403;4.697;4.520;3.006;3.992;...
3.539;4.363;3.296;3.877;4.277;3.753;3.513;3.026;3.927;2.824;4.108;4.556;4.121;3.171;3.637;3.946;2.818];






figure(331)
xlim([45 65])
ylim([-0.5 2])
hold on
plot(xxs,Time_NW_DS(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
% fillLines(xxs,Time_NW_DS(:,1)-Time_NW_DS(:,2),Time_NW_DS(:,1)+Time_NW_DS(:,2),{[100/255 100/255 102/255]},0.25)
plot(xxs,Time_P1_DS(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
% fillLines(xxs,Time_P1_DS(:,1)-Time_P1_DS(:,2),Time_P1_DS(:,1)+Time_P1_DS(:,2),{[194/255 81/255 79/255]},0.25)
plot(xxs,Time_P2_DS(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
% fillLines(xxs,Time_P2_DS(:,1)-Time_P2_DS(:,2),Time_P2_DS(:,1)+Time_P2_DS(:,2),{[248/255 153/255 55/255]},0.25)
plot(xxs,Time_P3_DS(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
% fillLines(xxs,Time_P3_DS(:,1)-Time_P3_DS(:,2),Time_P3_DS(:,1)+Time_P3_DS(:,2),{[153/255 204/255 103/255]},0.25)
plot(xxs,Time_P4_DS(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% fillLines(xxs,Time_P4_DS(:,1)-Time_P4_DS(:,2),Time_P4_DS(:,1)+Time_P4_DS(:,2),{[67/255 172/255 198/255]},0.25)
% 

figure(331)
hold on
plot(xxs,COM_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
fillLines(xxs,COM_Normalwalking(:,3)-COM_Normalwalking_std(:,3),COM_Normalwalking(:,3)+COM_Normalwalking_std(:,3),{[100/255 100/255 102/255]},0.25)
plot(xxs,COM_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
fillLines(xxs,COM_profile1(:,3)-COM_profile1_std(:,3),COM_profile1(:,3)+COM_profile1_std(:,3),{[194/255 81/255 79/255]},0.25)
plot(xxs,COM_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
fillLines(xxs,COM_profile2(:,3)-COM_profile2_std(:,3),COM_profile2(:,3)+COM_profile2_std(:,3),{[248/255 153/255 55/255]},0.25)
plot(xxs,COM_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
fillLines(xxs,COM_profile3(:,3)-COM_profile3_std(:,3),COM_profile3(:,3)+COM_profile3_std(:,3),{[153/255 204/255 103/255]},0.25)
plot(xxs,COM_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
fillLines(xxs,COM_profile4(:,3)-COM_profile4_std(:,3),COM_profile4(:,3)+COM_profile4_std(:,3),{[67/255 172/255 198/255]},0.25)


figure(442)
% xlim([45 65])
% ylim([-0.5 2])
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
yyaxis left
hold on
ax = gca ;
ax.FontSize = 30 ;
xticks([0:10:100])
% yyaxis left
hold on
plot(xxs,MOS_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,MOS_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,MOS_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,MOS_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,MOS_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
ylim([0.1 0.4])
yticks([0.1 0.2 0.3 0.4])
yyaxis right
ylim([0 300])
yticks([0 100 200 300])
% % plot(xxs,CON_NW(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
% a11 = plot(xxs,CON_P1(:,9),'--','linewidth',4,'color',[194/255 81/255 79/255]); a11.Color(4) = 0.5;
% a22 = plot(xxs,CON_P2(:,9),'--','linewidth',4,'color',[248/255 153/255 55/255]);  a22.Color(4) = 0.5;
% a33 = plot(xxs,CON_P3(:,9),'--','linewidth',4,'color',[153/255 204/255 103/255]); a33.Color(4) = 0.5;
a44 = plot(xxs,CON_P4(:,9),'--','linewidth',4,'color',[67/255 172/255 198/255]) ; a44.Color(4) = 0.5;
% 
% a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
% a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
% a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;

figure(4422)
% xlim([45 65])
% ylim([-0.5 2])
yyaxis left
hold on
plot(xxs,BX_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,MOS_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,MOS_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,MOS_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,MOS_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
title('MoS - ML')






figure(4421)
% xlim([45 65])
% ylim([-0.5 2])
hold on
plot(xxs,MOS_NW(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,MOS_P1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,MOS_P2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,MOS_P3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,MOS_P4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
figure(331)
hold on
plot(COM_Normalwalking(1:6000,1),COM_Normalwalking(1:6000,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_profile1(1:6000,1),COM_profile1(1:6000,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_profile2(1:6000,1),COM_profile2(1:6000,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_profile3(1:6000,1),COM_profile3(1:6000,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_profile4(1:6000,1),COM_profile4(1:6000,2),'-','linewidth',4,'color',[67/255 172/255 198/255])

figure(4431)
% subplot(2,3,1)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
% yyaxis left
hold on
ax = gca ;
ax.FontSize = 10 ;
% xlabel('Gait cycle (%)')
hold on
% xlim([0 1.1])
xticks([0 20 40 60 80 100])
% ylim([-0.03 0.03])
plot(xxs,CON_P1(:,3),'-','linewidth',3,'color',[194/255 81/255 79/255])
plot(xxs,CON_P2(:,3),'-','linewidth',3,'color',[248/255 153/255 55/255])
plot(xxs,CON_P3(:,3),'-','linewidth',3,'color',[153/255 204/255 103/255])
plot(xxs,CON_P4(:,3),'-','linewidth',3,'color',[67/255 172/255 198/255])
set(gcf,'Position',[0 0 1000 450],'Renderer', 'painters')


figure(4431)
% subplot(2,3,1)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
ax.OuterPosition =[1,1,9.0,5.3];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
% ax.Position =[1.926166666666667,1.222375,15.206249999999997,7.144187499999999];
xticks([0 10 20 30 40 50 60 70 80 90 100])
ylim([-0.03 0.03])
yticks([-0.03 -0.02 -0.01 0 0.01 0.02 0.03])
% xlabel('Gait cycle (%)')
hold on
% xlim([0 1.1])
% xticks([0 20 40 60 80 100])
% ylim([-0.03 0.03])
plot(xxs,COM_Normalwalking(:,1),'-','linewidth',2,'color',[100/255 100/255 102/255])
plot(xxs,COM_profile1(:,1),'-','linewidth',2,'color',[194/255 81/255 79/255])
plot(xxs,COM_profile2(:,1),'-','linewidth',2,'color',[248/255 153/255 55/255])
plot(xxs,COM_profile3(:,1),'-','linewidth',2,'color',[153/255 204/255 103/255])
plot(xxs,COM_profile4(:,1),'-','linewidth',2,'color',[67/255 172/255 198/255])
set(gcf,'Position',[0 0 1000 450],'Renderer', 'painters')
% yyaxis right
% a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
% a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
% a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
% a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;
plot(linspace(0,mean(Time_NW_sts(:,1)+Time_NW_sw(:,1)),10001),COM_Normalwalking(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(linspace(0,mean(Time_P1_sts(:,1)+Time_P1_sw(:,1)),10001),COM_profile1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(linspace(0,mean(Time_P2_sts(:,1)+Time_P2_sw(:,1)),10001),COM_profile2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(linspace(0,mean(Time_P3_sts(:,1)+Time_P3_sw(:,1)),10001),COM_profile3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(linspace(0,mean(Time_P4_sts(:,1)+Time_P4_sw(:,1)),10001),COM_profile4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM X - frontal (Real-time)')

figure(444)
% subplot(2,3,2)
hold on
% plot(xxs,COM_Normalwalking(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(xxs,COM_profile1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(xxs,COM_profile2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(xxs,COM_profile3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(xxs,COM_profile4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
plot(linspace(0,mean(Time_NW_sts(:,1)+Time_NW_sw(:,1)),10001),COM_Normalwalking(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(linspace(0,mean(Time_P1_sts(:,1)+Time_P1_sw(:,1)),10001),COM_profile1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(linspace(0,mean(Time_P2_sts(:,1)+Time_P2_sw(:,1)),10001),COM_profile2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(linspace(0,mean(Time_P3_sts(:,1)+Time_P3_sw(:,1)),10001),COM_profile3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(linspace(0,mean(Time_P4_sts(:,1)+Time_P4_sw(:,1)),10001),COM_profile4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Y - sagittal')
figure(445)
% subplot(2,3,3)
hold on
% plot(xxs,COM_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(xxs,COM_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(xxs,COM_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(xxs,COM_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(xxs,COM_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
plot(linspace(0,mean(Time_NW_sts(:,1)+Time_NW_sw(:,1)),10001),COM_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(linspace(0,mean(Time_P1_sts(:,1)+Time_P1_sw(:,1)),10001),COM_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(linspace(0,mean(Time_P2_sts(:,1)+Time_P2_sw(:,1)),10001),COM_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(linspace(0,mean(Time_P3_sts(:,1)+Time_P3_sw(:,1)),10001),COM_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(linspace(0,mean(Time_P4_sts(:,1)+Time_P4_sw(:,1)),10001),COM_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Z - transverse')
b1 = linspace(0,mean(Time_NW_sts(:,1)+Time_NW_sw(:,1)),10001)'; b2 = linspace(0,mean(Time_P1_sts(:,1)+Time_P1_sw(:,1)),10001)';
b3 = linspace(0,mean(Time_P2_sts(:,1)+Time_P2_sw(:,1)),10001)'; b4 = linspace(0,mean(Time_P3_sts(:,1)+Time_P3_sw(:,1)),10001)';
b5 = linspace(0,mean(Time_P4_sts(:,1)+Time_P4_sw(:,1)),10001)';
subplot(2,3,4)
title('COM x at terminal double stance')
hold on
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),1),...
        COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),1),...
        '-','linewidth',4,'color',[100/255 100/255 102/255])
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),1),...
        'o','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
    plot(b1(round(mean((Time_DS.NW.term2.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),1),...
       'v','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')

    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),1),...
        COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),1),...
        '-','linewidth',4,'color',[194/255 81/255 79/255])
    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),1),...
        'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
    plot(b2(round(mean((Time_DS.P1.term2.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),1),...
       'v','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')

    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),1),...
        COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),1),...
        '-','linewidth',4,'color',[248/255 153/255 55/255])
    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))),1),...
        'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
    plot(b3(round(mean((Time_DS.P2.term2.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term2.mean(:,1)))),1),...
       'v','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')

    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),1),...
        COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),1),...
        '-','linewidth',4,'color',[153/255 204/255 103/255])
    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),1),...
        'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
    plot(b4(round(mean((Time_DS.P3.term2.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),1),...
       'v','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')

    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),1),...
        COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),1),...
        '-','linewidth',4,'color',[67/255 172/255 198/255])
    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))),1),...
        'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
    plot(b5(round(mean((Time_DS.P4.term2.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term2.mean(:,1)))),1),...
       'v','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
subplot(2,3,5)
title('COM y at terminal double stance')
hold on
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),1),...
        COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),2),...
        '-','linewidth',4,'color',[100/255 100/255 102/255])
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),2),...
        'o','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
    plot(b1(round(mean((Time_DS.NW.term2.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),2),...
       'v','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')

    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),1),...
        COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),2),...
        '-','linewidth',4,'color',[194/255 81/255 79/255])
    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),2),...
        'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
    plot(b2(round(mean((Time_DS.P1.term2.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),2),...
       'v','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')

    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),1),...
        COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),2),...
        '-','linewidth',4,'color',[248/255 153/255 55/255])
    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))),2),...
        'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
    plot(b3(round(mean((Time_DS.P2.term2.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term2.mean(:,1)))),2),...
       'v','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')

    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),1),...
        COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),2),...
        '-','linewidth',4,'color',[153/255 204/255 103/255])
    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),2),...
        'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
    plot(b4(round(mean((Time_DS.P3.term2.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),2),...
       'v','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')

    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),1),...
        COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),2),...
        '-','linewidth',4,'color',[67/255 172/255 198/255])
    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))),2),...
        'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
    plot(b5(round(mean((Time_DS.P4.term2.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term2.mean(:,1)))),2),...
       'v','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
subplot(2,3,6)
title('COM z at terminal double stance')
hold on
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),1),...
        COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),3),...
        '-','linewidth',4,'color',[100/255 100/255 102/255])
    plot(b1(round(mean((Time_DS.NW.term1.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
    plot(b1(round(mean((Time_DS.NW.term2.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')

    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),1),...
        COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
        '-','linewidth',4,'color',[194/255 81/255 79/255])
    plot(b2(round(mean((Time_DS.P1.term1.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
    plot(b2(round(mean((Time_DS.P1.term2.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')

    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),1),...
        COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
        '-','linewidth',4,'color',[248/255 153/255 55/255])
    plot(b3(round(mean((Time_DS.P2.term1.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
    plot(b3(round(mean((Time_DS.P2.term2.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')

    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),1),...
        COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
        '-','linewidth',4,'color',[153/255 204/255 103/255])
    plot(b4(round(mean((Time_DS.P3.term1.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
    plot(b4(round(mean((Time_DS.P3.term2.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')

    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),1),...
        COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
        '-','linewidth',4,'color',[67/255 172/255 198/255])
    plot(b5(round(mean((Time_DS.P4.term1.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
    plot(b5(round(mean((Time_DS.P4.term2.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')


% 
figure(446)
% subplot(2,3,4)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
ax.OuterPosition =[1,1,9.0,5.3];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

ylim([-0.4 0.4])
xticks([0 10 20 30 40 50 60 70 80 90 100])
% xlabel('Gait cycle (%)')

plot(xxs,COM_vel_Normalwalking(:,1),'-','linewidth',2,'color',[100/255 100/255 102/255])
plot(xxs,COM_vel_profile1(:,1),'-','linewidth',2,'color',[194/255 81/255 79/255])
plot(xxs,COM_vel_profile2(:,1),'-','linewidth',2,'color',[248/255 153/255 55/255])
plot(xxs,COM_vel_profile3(:,1),'-','linewidth',2,'color',[153/255 204/255 103/255])
plot(xxs,COM_vel_profile4(:,1),'-','linewidth',2,'color',[67/255 172/255 198/255])
set(gcf,'Position',[0 0 1000 450],'Renderer', 'painters')
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Arial','fontsize',28)
% legend('NW','p1','p2','p3','p4')
% ylabel('COM Vel frontal (m/s)')
yyaxis right
a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;
ylabel('Force (N)')
% % figure(447)
% subplot(2,3,5)
% hold on
% plot(xxs,COM_vel_Normalwalking(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(xxs,COM_vel_profile1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(xxs,COM_vel_profile2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(xxs,COM_vel_profile3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(xxs,COM_vel_profile4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
% % legend('NW','p1','p2','p3','p4')
% title('COM Vel sagittal')
% % figure(448)
% subplot(2,3,6)
% hold on
% plot(xxs,COM_vel_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(xxs,COM_vel_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(xxs,COM_vel_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(xxs,COM_vel_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(xxs,COM_vel_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
% % legend('NW','p1','p2','p3','p4')
% title('COM Vel transverse')

figure(4437)
% subplot(2,3,1)

set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
ax.OuterPosition =[1,1,9.0,5.3];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

ylim([-0.4 0.4])
xticks([0 10 20 30 40 50 60 70 80 90 100])
% xlabel('Gait cycle (%)')
hold on
grid on
% xticks([0 10 20 30 40 50 60 70 80 90 100])
% xticks([0 20 40 60 80 100])
ylim([-3 3])
plot(xxs,COM_acc_Normalwalking(:,1),'-','linewidth',2,'color',[100/255 100/255 102/255]) ;
ac2 = plot(xxs,COM_acc_profile1(:,1),'-','linewidth',2,'color',[194/255 81/255 79/255]) ; ac2.Color(4) = 0.25;
ac3 = plot(xxs,COM_acc_profile2(:,1),'-','linewidth',2,'color',[248/255 153/255 55/255]) ; ac3.Color(4) = 0.25;
ac4 = plot(xxs,COM_acc_profile3(:,1),'-','linewidth',2,'color',[153/255 204/255 103/255]) ; ac4.Color(4) = 0.25;
ac5 = plot(xxs,COM_acc_profile4(:,1),'-','linewidth',2,'color',[67/255 172/255 198/255]) ; ac5.Color(4) = 0.25;
plot(xxs(25:272),COM_acc_profile1(25:272,1),'-','linewidth',2,'color',[194/255 81/255 79/255])
plot(xxs(105:351),COM_acc_profile2(105:351,1),'-','linewidth',2,'color',[248/255 153/255 55/255])
plot(xxs(306:554),COM_acc_profile3(306:554,1),'-','linewidth',2,'color',[153/255 204/255 103/255])
plot(xxs(376:634),COM_acc_profile4(376:634,1),'-','linewidth',2,'color',[67/255 172/255 198/255])
set(gcf,'Position',[0 0 1000 450],'Renderer', 'painters')
yyaxis right
a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;
ylabel('Force (N)')


figure(4438)
% subplot(2,3,1)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
% yyaxis left
hold on
ax = gca ;
ax.FontSize = 30 ;
% xlabel('Gait cycle (%)')
hold on
xticks([0 10 20 30 40 50 60 70 80 90 100])
% ylim([-0.03 0.03])
plot(xxs,COM_acc_Normalwalking(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255]) ;
ac2 = plot(xxs,COM_acc_profile1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255]) ; ac2.Color(4) = 0.25;
ac3 = plot(xxs,COM_acc_profile2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255]) ; ac3.Color(4) = 0.25;
ac4 = plot(xxs,COM_acc_profile3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255]) ; ac4.Color(4) = 0.25;
ac5 = plot(xxs,COM_acc_profile4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; ac5.Color(4) = 0.25;
plot(xxs(248:2723),COM_acc_profile1(248:2723,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs(1048:3511),COM_acc_profile2(1048:3511,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs(3063:5543),COM_acc_profile3(3063:5543,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs(3761:6339),COM_acc_profile4(3761:6339,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
yyaxis right
a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;

figure(4439)
% subplot(2,3,1)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
% yyaxis left
hold on
ax = gca ;
ax.FontSize = 30 ;
% xlabel('Gait cycle (%)')
hold on
xticks([0 10 20 30 40 50 60 70 80 90 100])
% ylim([-0.03 0.03])
plot(xxs,COM_acc_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255]) ;
ac2 = plot(xxs,COM_acc_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]) ; ac2.Color(4) = 0.25;
ac3 = plot(xxs,COM_acc_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]) ; ac3.Color(4) = 0.25;
ac4 = plot(xxs,COM_acc_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]) ; ac4.Color(4) = 0.25;
ac5 = plot(xxs,COM_acc_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; ac5.Color(4) = 0.25;
plot(xxs(248:2723),COM_acc_profile1(248:2723,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs(1048:3511),COM_acc_profile2(1048:3511,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs(3063:5543),COM_acc_profile3(3063:5543,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs(3761:6339),COM_acc_profile4(3761:6339,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
yyaxis right
a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;
ylabel('Force (N)')
view(2)
figure(449)

set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on

ax = gca ;
ax.FontSize = 20 ;
% plot(COM_Normalwalking(round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))),1),...
%     COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(COM_profile1(round(mean(((Time_DS.P1.term1.mean(:,1))))):round(mean(((Time_DS.P1.term2.mean(:,1))))),1),...
%     COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(COM_profile2(round(mean(((Time_DS.P2.term1.mean(:,1))))):round(mean(((Time_DS.P2.term2.mean(:,1))))),1),...
%     COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(COM_profile3(round(mean(((Time_DS.P3.term1.mean(:,1))))):round(mean(((Time_DS.P3.term2.mean(:,1))))),1),...
%     COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(COM_profile4(round(mean(((Time_DS.P4.term1.mean(:,1))))):round(mean(((Time_DS.P4.term2.mean(:,1))))),1),...
%     COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM XZ at terminal double stance')
    plot(COM_Normalwalking(4000:6000,1),COM_Normalwalking(4000:6000,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
    plot(COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
    plot(COM_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),1),COM_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')

    plot(COM_profile1(4000:6000,1),COM_profile1(4000:6000,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
    plot(COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
    plot(COM_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),1),COM_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')

    plot(COM_profile2(4000:6000,1),COM_profile2(4000:6000,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
    plot(COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
    plot(COM_profile2(round(mean((Time_DS.P2.term2.mean(:,1)))),1),COM_profile2(round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')

    plot(COM_profile3(4000:6000,1),COM_profile3(4000:6000,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
    plot(COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
    plot(COM_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),1),COM_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')

    plot(COM_profile4(4000:6000,1),COM_profile4(4000:6000,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
    plot(COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))),3),...
        'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
    plot(COM_profile4(round(mean((Time_DS.P4.term2.mean(:,1)))),1),COM_profile4(round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
       'v','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')


view(2)
figure(450)
subplot(2,3,1)
hold on
plot(COM_Normalwalking(:,2),COM_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_profile1(:,2),COM_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_profile2(:,2),COM_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_profile3(:,2),COM_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_profile4(:,2),COM_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM YZ')

% figure(4501)
subplot(2,3,3)
hold on
plot(xxs,COM_vel_Normalwalking(:,5),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,COM_vel_profile1(:,5),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,COM_vel_profile2(:,5),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,COM_vel_profile3(:,5),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,COM_vel_profile4(:,5),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Vel yz Resultant')

% figure(4511)
subplot(2,3,6)
hold on
plot((round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))))*0.01,...
    COM_vel_Normalwalking(round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))),5),...
    '-','linewidth',4,'color',[100/255 100/255 102/255])
plot((round(mean(((Time_DS.P1.term1.mean(:,1))))):round(mean(((Time_DS.P1.term2.mean(:,1))))))*0.01,...
    COM_vel_profile1(round(mean(((Time_DS.P1.term1.mean(:,1))))):round(mean(((Time_DS.P1.term2.mean(:,1))))),5),...
    '-','linewidth',4,'color',[194/255 81/255 79/255])
plot((round(mean(((Time_DS.P2.term1.mean(:,1))))):round(mean(((Time_DS.P2.term2.mean(:,1))))))*0.01,...
    COM_vel_profile2(round(mean(((Time_DS.P2.term1.mean(:,1))))):round(mean(((Time_DS.P2.term2.mean(:,1))))),5),...
    '-','linewidth',4,'color',[248/255 153/255 55/255])
plot((round(mean(((Time_DS.P3.term1.mean(:,1))))):round(mean(((Time_DS.P3.term2.mean(:,1))))))*0.01,...
    COM_vel_profile3(round(mean(((Time_DS.P3.term1.mean(:,1))))):round(mean(((Time_DS.P3.term2.mean(:,1))))),5),...
    '-','linewidth',4,'color',[153/255 204/255 103/255])
plot((round(mean(((Time_DS.P4.term1.mean(:,1))))):round(mean(((Time_DS.P4.term2.mean(:,1))))))*0.01,...
    COM_vel_profile4(round(mean(((Time_DS.P4.term1.mean(:,1))))):round(mean(((Time_DS.P4.term2.mean(:,1))))),5),...
    '-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Vel Resultant')




% figure(4502)
subplot(2,3,4)
hold on
plot(COM_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),2),...
    COM_Normalwalking(round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))),3),...
    '-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),2),...
    COM_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
    '-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),2),...
    COM_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
    '-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),2),...
    COM_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
    '-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),2),...
    COM_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
    '-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM YZ - terminal double stance')

% figure(4503)
xxs2 = 40:0.01:60;
subplot(2,3,5)
hold on
plot(COM_vel_Normalwalking(4000:6000,2),COM_vel_Normalwalking(4000:6000,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_vel_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),2),COM_vel_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))),3),...
    'o','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
plot(COM_vel_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),2),COM_vel_Normalwalking(round(mean((Time_DS.NW.term2.mean(:,1)))),3),...
    'v','MarkerSize',10,'MarkerFaceColor',[100/255 100/255 102/255],'MarkerEdgeColor','k')
plot(COM_vel_profile1(4000:6000,2),COM_vel_profile1(4000:6000,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_vel_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),2),COM_vel_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))),3),...
    'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
plot(COM_vel_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),2),COM_vel_profile1(round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
    'v','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
plot(COM_vel_profile2(4000:6000,2),COM_vel_profile2(4000:6000,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_vel_profile2(round(mean((Time_DS.P1.term1.mean(:,1)))),2),COM_vel_profile2(round(mean((Time_DS.P1.term1.mean(:,1)))),3),...
    'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
plot(COM_vel_profile2(round(mean((Time_DS.P1.term2.mean(:,1)))),2),COM_vel_profile2(round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
    'v','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
plot(COM_vel_profile3(4000:6000,2),COM_vel_profile3(4000:6000,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_vel_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),2),COM_vel_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))),3),...
    'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
plot(COM_vel_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),2),COM_vel_profile3(round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
    'v','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
plot(COM_vel_profile4(4000:6000,2), COM_vel_profile4(4000:6000,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
plot(COM_vel_profile4(round(mean((Time_DS.P3.term1.mean(:,1)))),2),COM_vel_profile4(round(mean((Time_DS.P3.term1.mean(:,1)))),3),...
    'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
plot(COM_vel_profile4(round(mean((Time_DS.P3.term2.mean(:,1)))),2),COM_vel_profile4(round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
    'v','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
% legend('NW','p1','p2','p3','p4')
title('COM vel YZ - terminal double stance')

% subplot(2,3,5)
% hold on
% plot(COM_vel_Normalwalking(round(mean((Time_DS.NW.term1.mean(:,1)))):round(mean((Time_DS.NW.term2.mean(:,1)))),2),...
%     COM_vel_Normalwalking(round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))),3),...
%     '-','linewidth',4,'color',[100/255 100/255 102/255])
% plot(COM_vel_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),2),...
%     COM_vel_profile1(round(mean((Time_DS.P1.term1.mean(:,1)))):round(mean((Time_DS.P1.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[194/255 81/255 79/255])
% plot(COM_vel_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),2),...
%     COM_vel_profile2(round(mean((Time_DS.P2.term1.mean(:,1)))):round(mean((Time_DS.P2.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[248/255 153/255 55/255])
% plot(COM_vel_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),2),...
%     COM_vel_profile3(round(mean((Time_DS.P3.term1.mean(:,1)))):round(mean((Time_DS.P3.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[153/255 204/255 103/255])
% plot(COM_vel_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),2),...
%     COM_vel_profile4(round(mean((Time_DS.P4.term1.mean(:,1)))):round(mean((Time_DS.P4.term2.mean(:,1)))),3),...
%     '-','linewidth',4,'color',[67/255 172/255 198/255])
% % legend('NW','p1','p2','p3','p4')
% title('COM vel YZ - terminal double stance')

% figure(4504)
subplot(2,3,2)
hold on
plot(COM_vel_Normalwalking(:,2),COM_vel_Normalwalking(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_vel_profile1(:,2),COM_vel_profile1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_vel_profile2(:,2),COM_vel_profile2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_vel_profile3(:,2),COM_vel_profile3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_vel_profile4(:,2),COM_vel_profile4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Vel yz')

figure(451)
% subplot(2,3,3)
hold on
plot(xxs,COM_vel_Normalwalking(:,4),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,COM_vel_profile1(:,4),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,COM_vel_profile2(:,4),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,COM_vel_profile3(:,4),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,COM_vel_profile4(:,4),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Vel Resultant')


figure(4511)
% subplot(2,3,6)
hold on
plot((round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))))*0.01,...
    COM_vel_Normalwalking(round(mean(((Time_DS.NW.term1.mean(:,1))))):round(mean(((Time_DS.NW.term2.mean(:,1))))),4),...
    '-','linewidth',4,'color',[100/255 100/255 102/255])
plot((round(mean(((Time_DS.P1.term1.mean(:,1))))):round(mean(((Time_DS.P1.term2.mean(:,1))))))*0.01,...
    COM_vel_profile1(round(mean(((Time_DS.P1.term1.mean(:,1))))):round(mean(((Time_DS.P1.term2.mean(:,1))))),4),...
    '-','linewidth',4,'color',[194/255 81/255 79/255])
plot((round(mean(((Time_DS.P2.term1.mean(:,1))))):round(mean(((Time_DS.P2.term2.mean(:,1))))))*0.01,...
    COM_vel_profile2(round(mean(((Time_DS.P2.term1.mean(:,1))))):round(mean(((Time_DS.P2.term2.mean(:,1))))),4),...
    '-','linewidth',4,'color',[248/255 153/255 55/255])
plot((round(mean(((Time_DS.P3.term1.mean(:,1))))):round(mean(((Time_DS.P3.term2.mean(:,1))))))*0.01,...
    COM_vel_profile3(round(mean(((Time_DS.P3.term1.mean(:,1))))):round(mean(((Time_DS.P3.term2.mean(:,1))))),4),...
    '-','linewidth',4,'color',[153/255 204/255 103/255])
plot((round(mean(((Time_DS.P4.term1.mean(:,1))))):round(mean(((Time_DS.P4.term2.mean(:,1))))))*0.01,...
    COM_vel_profile4(round(mean(((Time_DS.P4.term1.mean(:,1))))):round(mean(((Time_DS.P4.term2.mean(:,1))))),4),...
    '-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('COM Vel Resultant')




figure(452)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
ax = gca ;
ax.FontSize = 20 ;
xticks([0 10 20 30 40 50 60 70 80 90 100])
yyaxis left
ylim([-0.2 0.2])
yticks([-0.2 -0.1 0 0.1 0.2])
hold on
plot(xxs,XCOM_Normalwalking(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,XCOM_profile1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,XCOM_profile2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,XCOM_profile3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,XCOM_profile4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
ylabel('XCOM-ML')
yyaxis right
ylabel('Force')
hold on
ylim([0 300])
yticks([0 100 200 300])
a33 = plot(xxs,CON_P3(:,9),'--','linewidth',4,'color',[153/255 204/255 103/255]); a33.Color(4) = 0.7;
a44 = plot(xxs,CON_P4(:,9),'--','linewidth',4,'color',[67/255 172/255 198/255]) ; a44.Color(4) = 0.7;
% legend('NW','p1','p2','p3','p4')
% title('XCOM mediolateral')

figure(4521)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
% ylim([0.4 0.5])
% yticks([0.4 0.45 0.5])
xticks([0 10 20 30 40 50 60 70 80 90 100])
ax = gca ;
ax.FontSize = 20 ;
hold on
plot(xxs,RMET_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,RMET_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,RMET_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,RMET_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,RMET_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('BoS')


figure(4522)

set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
% ax.OuterPosition =[1,1,16.1,6];
ax.OuterPosition =[1,1,8.89,6.5];
% ax.OuterPosition =[1,1,11.5,4.94];
% ax.Position = [1,1,12.7,4.94];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xticks([0 10 20 30 40 50 60 70 80 90 100])
% ax.Position =[1,1,11,5];
ylim([0.1 0.4])
yticks([0.1 0.2 0.3 0.4])
hold on
plot(xxs,MOS_NW(:,1),'-','linewidth',2,'color',[100/255 100/255 102/255])
plot(xxs,MOS_P1(:,1),'-','linewidth',2,'color',[194/255 81/255 79/255])
plot(xxs,MOS_P2(:,1),'-','linewidth',2,'color',[248/255 153/255 55/255])
plot(xxs,MOS_P3(:,1),'-','linewidth',2,'color',[153/255 204/255 103/255])
plot(xxs,MOS_P4(:,1),'-','linewidth',2,'color',[67/255 172/255 198/255])
ylabel('MoS - ML')

yyaxis right
ylabel('Force')
ylim([0 300])
yticks([0 100 200 300])
% plot(xxs,CON_NW(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
% a11 = plot(xxs,CON_P1(:,9),'--','linewidth',4,'color',[194/255 81/255 79/255]); a11.Color(4) = 0.5;
% a22 = plot(xxs,CON_P2(:,9),'--','linewidth',4,'color',[248/255 153/255 55/255]);  a22.Color(4) = 0.5;
% a33 = plot(xxs,CON_P3(:,9),'--','linewidth',4,'color',[153/255 204/255 103/255]); a33.Color(4) = 0.7;
a44 = plot(xxs,CON_P4(:,9),'--','linewidth',2,'color',[67/255 172/255 198/255]) ; a44.Color(4) = 0.5;

% a111 = plot(xxs,CON_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
% a222 = plot(xxs,CON_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
% a333 = plot(xxs,CON_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,3),'-','linewidth',2,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;



figure(453)
view(3)
hold all
plot3(XCOM_Normalwalking(:,1)-XCOM_Normalwalking(1,1),XCOM_Normalwalking(:,2)-XCOM_Normalwalking(1,2),...
    XCOM_Normalwalking(:,3)-XCOM_Normalwalking(1,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot3(XCOM_profile1(:,1)-XCOM_profile1(1,1),XCOM_profile1(:,2)-XCOM_profile1(1,2),...
    XCOM_profile1(:,3)-XCOM_profile1(1,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot3(XCOM_profile2(:,1)-XCOM_profile2(1,1),XCOM_profile2(:,2)-XCOM_profile2(1,2),...
    XCOM_profile2(:,3)-XCOM_profile2(1,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot3(XCOM_profile3(:,1)-XCOM_profile3(1,1),XCOM_profile3(:,2)-XCOM_profile3(1,2),...
    XCOM_profile3(:,3)-XCOM_profile3(1,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot3(XCOM_profile4(:,1)-XCOM_profile4(1,1),XCOM_profile4(:,2)-XCOM_profile4(1,2),...
    XCOM_profile4(:,3)-XCOM_profile4(1,3),'-','linewidth',4,'color',[67/255 172/255 198/255]); 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
% legend('NW','p1','p2','p3','p4')

figure(455)
hold on
plot(xxs,Hip_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,Hip_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,Hip_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,Hip_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,Hip_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Hip joint angle - sagittal')

figure(456)
hold on
plot(xxs,Hip_NW(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,Hip_P1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,Hip_P2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,Hip_P3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,Hip_P4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Hip joint angle - frontal')

figure(457)
hold on
plot(xxs,Hip_NW(:,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,Hip_P1(:,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,Hip_P2(:,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,Hip_P3(:,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,Hip_P4(:,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Hip joint angle - transverse')

figure(4571)
hold on
plot(xxs,Kne_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,Kne_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,Kne_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,Kne_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,Kne_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Knee joint angle - sagittal')

% CON : R_power ; R_force; L_force; R_moment; R_power_mo; B_angvel_f
% B_angvel_s; R_HIP_angvel_y_rad; R_HIP_angvel_x_rad

figure(45771)

set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
ax.OuterPosition =[1,1,16.1,6];
% ax.OuterPosition =[1,1,8.89,4.94];
% ax.OuterPosition =[1,1,11.5,4.94];
% ax.Position = [1,1,12.7,4.94];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xticks([0 10 20 30 40 50 60 70 80 90 100])
ylim([0 300])
yticks([0 100 200 300])
hold on
grid on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,CON_P1(:,3),'-','linewidth',2.5,'color',[194/255 81/255 79/255])
plot(xxs,CON_P2(:,3),'-','linewidth',2.5,'color',[248/255 153/255 55/255])
plot(xxs,CON_P3(:,3),'-','linewidth',2.5,'color',[153/255 204/255 103/255])
plot(xxs,CON_P4(:,3),'-','linewidth',2.5,'color',[67/255 172/255 198/255])
fillLines(xxs,CON_P1(:,3)-CON_P1_std(:,3),CON_P1(:,3)+CON_P1_std(:,3),{[194/255 81/255 79/255]},0.25)
fillLines(xxs,CON_P2(:,3)-CON_P2_std(:,3),CON_P2(:,3)+CON_P2_std(:,3),{[248/255 153/255 55/255]},0.25)
fillLines(xxs,CON_P3(:,3)-CON_P3_std(:,3),CON_P3(:,3)+CON_P3_std(:,3),{[153/255 204/255 103/255]},0.25)
fillLines(xxs,CON_P4(:,3)-CON_P4_std(:,3),CON_P4(:,3)+CON_P4_std(:,3),{[67/255 172/255 198/255]},0.25)
% legend('NW','p1','p2','p3','p4')
% ylabel('Delivered mechanical power (W/kg)')
xlabel('Gait cycle (%)')


figure(45772)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
% ax.OuterPosition =[1,1,11.5,4.94];
ax.OuterPosition =[1,1,16.1,6];
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
xticks([0 10 20 30 40 50 60 70 80 90 100])
ylim([-0.2 0.6])
yticks([-0.2 0 0.2 0.4 0.6])

% ax.Position = [1,1,12.7,4.94];
% ax.OuterPosition =[1,1,18.54,7.2];
hold on
grid on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,CON_P1(:,1),'-','linewidth',2.5,'color',[194/255 81/255 79/255])
plot(xxs,CON_P2(:,1),'-','linewidth',2.5,'color',[248/255 153/255 55/255])
plot(xxs,CON_P3(:,1),'-','linewidth',2.5,'color',[153/255 204/255 103/255])
plot(xxs,CON_P4(:,1),'-','linewidth',2.5,'color',[67/255 172/255 198/255])
fillLines(xxs,CON_P1(:,1)-CON_P1_std(:,1),CON_P1(:,1)+CON_P1_std(:,1),{[194/255 81/255 79/255]},0.25)
fillLines(xxs,CON_P2(:,1)-CON_P2_std(:,1),CON_P2(:,1)+CON_P2_std(:,1),{[248/255 153/255 55/255]},0.25)
fillLines(xxs,CON_P3(:,1)-CON_P3_std(:,1),CON_P3(:,1)+CON_P3_std(:,1),{[153/255 204/255 103/255]},0.25)
fillLines(xxs,CON_P4(:,1)-CON_P4_std(:,1),CON_P4(:,1)+CON_P4_std(:,1),{[67/255 172/255 198/255]},0.25)


figure(458)
hold on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,-CON_P1(:,5),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,-CON_P2(:,5),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,-CON_P3(:,5),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,-CON_P4(:,5),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Delivered power mo frontal')

figure(4582)
hold on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,CON_P1(:,8),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,CON_P2(:,8),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,CON_P3(:,8),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,CON_P4(:,8),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Delivered power mo sagittal')



figure(4588)
hold on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,CON_P1(:,4),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,CON_P2(:,4),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,CON_P3(:,4),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,CON_P4(:,4),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('Moment')

figure(459)
hold on
% plot(xxs,-B_angvel_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,-CON_P1(:,6),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,-CON_P2(:,6),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,-CON_P3(:,6),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,-CON_P4(:,6),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('B angular velocity - IMU - Frontal')

figure(460)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
grid on

ax = gca ;
ax.FontSize = 20 ;
% yyaxis left

hold on
plot(xxs,B_angvel_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,B_angvel_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,B_angvel_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,B_angvel_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,B_angvel_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% fillLines(xxs,B_angvel_P4(:,1)-B_angvel_P4_std(:,1),B_angvel_P4(:,1)+B_angvel_P4_std(:,1),{[67/255 172/255 198/255]},0.25)
% legend('NW','p1','p2','p3','p4')
% title('Hip angular velocity - frontal')
ylabel('Hip angular velocity - frontal (m/s)')
yyaxis right
ylabel('Moment (Nm/kg)')
hold on
% plot(xxs,CON_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
a1 = plot(xxs,CON_P1(:,4),'-','linewidth',4,'color',[194/255 81/255 79/255]); a1.Color(4) = 0.5;
a2 = plot(xxs,CON_P2(:,4),'-','linewidth',4,'color',[248/255 153/255 55/255]); a2.Color(4) = 0.5;
a_p3 = plot(xxs,CON_P3(:,4),'-','linewidth',4,'color',[153/255 204/255 103/255]) ; a_p3.Color(4) = 0.5;
a4 = plot(xxs,CON_P4(:,4),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a4.Color(4) = 0.5;


% legend('NW','p1','p2','p3','p4')
% title('Moment')



figure(461)
hold on
% plot(xxs,-B_angvel_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,(CON_P1(:,7)),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,(CON_P2(:,7)),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,(CON_P3(:,7)),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,(CON_P4(:,7)),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('B angular velocity - IMU - sagittal')

figure(4613)
hold on
% plot(xxs,-B_angvel_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,-CON_P1(:,6),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,-CON_P2(:,6),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,-CON_P3(:,6),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,-CON_P4(:,6),'-','linewidth',4,'color',[67/255 172/255 198/255])
% legend('NW','p1','p2','p3','p4')
title('B angular velocity - IMU - frontal')



figure(462)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
title('Hip angular velocity - sagittal','FontName','Arial','Fontsize',20)
xlabel('Gait cycle (%)','FontName','Arial','Fontsize',20)
hold on
% grid on
yyaxis left
hold on
plot(xxs,B_angvel_NW(:,2),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,B_angvel_P1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,B_angvel_P2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,B_angvel_P3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,B_angvel_P4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255])
ylabel('Angular velocity (deg/s)','FontName','Arial','Fontsize',20)
% fillLines(xxs,B_angvel_P4(:,1)-B_angvel_P4_std(:,1),B_angvel_P4(:,1)+B_angvel_P4_std(:,1),{[67/255 172/255 198/255]},0.25)
% legend('NW','p1','p2','p3','p4')

yyaxis right
a111 = plot(xxs,CON_P1(:,2),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,CON_P2(:,2),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,CON_P3(:,2),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
% a3332 = plot(xxs,CON_P3(:,9),'--','linewidth',4,'color',[153/255 204/255 103/255]); a3332.Color(4) = 0.5;
a444 = plot(xxs,CON_P4(:,2),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;
ylabel('Force (N)','FontName','Arial','Fontsize',20)




% 
% figure(458)
figure(4621)
yyaxis left
hold on
plot(xxs,B_angvel_NW(:,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,B_angvel_P1(:,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,B_angvel_P2(:,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,B_angvel_P3(:,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,B_angvel_P4(:,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
% fillLines(xxs,B_angvel_P4(:,1)-B_angvel_P4_std(:,1),B_angvel_P4(:,1)+B_angvel_P4_std(:,1),{[67/255 172/255 198/255]},0.25)
% legend('NW','p1','p2','p3','p4')
ylim([-150 150])
title('Hip angular velocity - frontal')
yyaxis right
a111 = plot(xxs,-CON_P1(:,5),'-','linewidth',4,'color',[194/255 81/255 79/255]); a111.Color(4) = 0.5;
a222 = plot(xxs,-CON_P2(:,5),'-','linewidth',4,'color',[248/255 153/255 55/255]);  a222.Color(4) = 0.5;
a333 = plot(xxs,-CON_P3(:,5),'-','linewidth',4,'color',[153/255 204/255 103/255]); a333.Color(4) = 0.5;
a444 = plot(xxs,-CON_P4(:,5),'-','linewidth',4,'color',[67/255 172/255 198/255]) ; a444.Color(4) = 0.5;






%% time normalizing
PSD_R_ANK_angle_x = [];
PSD_R_ANK_angle_y = [];
PSD_R_ANK_angle_z = [];
PSD_R_KNE_angle_x = [];
PSD_R_KNE_angle_y = [];
PSD_R_KNE_angle_z = [];
PSD_R_HIP_angle_x = [];
PSD_R_HIP_angle_y = [];
PSD_R_HIP_angle_z = [];

PSD_Pelvis_tilt_x = [];
PSD_Pelvis_tilt_y = [];
PSD_Pelvis_tilt_z = [];

PSD_COM_x = [];
PSD_COM_y = [];
PSD_COM_z = [];


for i = length(RHS)-119:length(RHS)
    xx = (0:100/(RHS(i)-1-RHS(i-1,1)):100)';
    yy_R_ANK_angle_x = R_ANK_angle_x(RHS(i-1):RHS(i)-1,1);
    yy_R_ANK_angle_y = R_ANK_angle_y(RHS(i-1):RHS(i)-1,1);
    yy_R_ANK_angle_z = R_ANK_angle_z(RHS(i-1):RHS(i)-1,1);
    yy_R_KNE_angle_x = R_KNE_angle_x(RHS(i-1):RHS(i)-1,1);
    yy_R_KNE_angle_y = R_KNE_angle_y(RHS(i-1):RHS(i)-1,1);
    yy_R_KNE_angle_z = R_KNE_angle_z(RHS(i-1):RHS(i)-1,1);
    yy_R_HIP_angle_x = R_HIP_angle_s(RHS(i-1):RHS(i)-1,1);
    yy_R_HIP_angle_y = R_HIP_angle_f(RHS(i-1):RHS(i)-1,1);
    yy_R_HIP_angle_z = R_HIP_angle_z(RHS(i-1):RHS(i)-1,1);

    yy_Pelvis_tilt_x = Pelvis_tilt_x(RHS(i-1):RHS(i)-1,1);
    yy_Pelvis_tilt_y = Pelvis_tilt_y(RHS(i-1):RHS(i)-1,1);
    yy_Pelvis_tilt_z = Pelvis_tilt_z(RHS(i-1):RHS(i)-1,1);
    
    yy_COM_x = COM_x(RHS(i-1):RHS(i)-1,1);
    yy_COM_y = COM_y(RHS(i-1):RHS(i)-1,1);
    yy_COM_z = COM_z(RHS(i-1):RHS(i)-1,1);
    xspa = 0:0.01:100 ;

    temp28 = interp1(xx,yy_R_ANK_angle_x,xspa);
    temp29 = interp1(xx,yy_R_ANK_angle_y,xspa);
    temp30 = interp1(xx,yy_R_ANK_angle_z,xspa);
    temp31 = interp1(xx,yy_R_KNE_angle_x,xspa);
    temp32 = interp1(xx,yy_R_KNE_angle_y,xspa);
    temp33 = interp1(xx,yy_R_KNE_angle_z,xspa);
    temp34 = interp1(xx,yy_R_HIP_angle_x,xspa);
    temp35 = interp1(xx,yy_R_HIP_angle_y,xspa);
    temp36 = interp1(xx,yy_R_HIP_angle_z,xspa);
    
    temp43 = interp1(xx,yy_Pelvis_tilt_x,xspa);
    temp44 = interp1(xx,yy_Pelvis_tilt_y,xspa);
    temp45 = interp1(xx,yy_Pelvis_tilt_z,xspa);
    
    temp46 = interp1(xx,yy_COM_x,xspa);
    temp47 = interp1(xx,yy_COM_y,xspa);
    temp48 = interp1(xx,yy_COM_z,xspa);
    

    PSD_R_ANK_angle_x = [PSD_R_ANK_angle_x , temp28'];
    PSD_R_ANK_angle_y = [PSD_R_ANK_angle_y , temp29'];
    PSD_R_ANK_angle_z = [PSD_R_ANK_angle_z , temp30'];
    PSD_R_KNE_angle_x = [PSD_R_KNE_angle_x , temp31'];
    PSD_R_KNE_angle_y = [PSD_R_KNE_angle_y , temp32'];
    PSD_R_KNE_angle_z = [PSD_R_KNE_angle_z , temp33'];
    PSD_R_HIP_angle_x = [PSD_R_HIP_angle_x , temp34'];
    PSD_R_HIP_angle_y = [PSD_R_HIP_angle_y , temp35'];
    PSD_R_HIP_angle_z = [PSD_R_HIP_angle_z , temp36'];
    
    PSD_Pelvis_tilt_x = [PSD_Pelvis_tilt_x , temp43'];
    PSD_Pelvis_tilt_y = [PSD_Pelvis_tilt_y , temp44'];
    PSD_Pelvis_tilt_z = [PSD_Pelvis_tilt_z , temp45'];
    
    PSD_COM_x = [PSD_COM_x , temp46'];
    PSD_COM_y = [PSD_COM_y , temp47'];
    PSD_COM_z = [PSD_COM_z , temp48'];
end

PSD_L_ANK_angle_x = [];
PSD_L_ANK_angle_y = [];
PSD_L_ANK_angle_z = [];
PSD_L_KNE_angle_x = [];
PSD_L_KNE_angle_y = [];
PSD_L_KNE_angle_z = [];
PSD_L_HIP_angle_x = [];
PSD_L_HIP_angle_y = [];
PSD_L_HIP_angle_z = [];

aa = (RHS(end,1)-LHS);
aaa = find(aa>0,1,'last');

for i = aaa-118:aaa
    xx = (0:100/(LHS(i)-1-LHS(i-1,1)):100)';
    
    yy_L_ANK_angle_x = L_ANK_angle_x(LHS(i-1):LHS(i)-1,1);
    yy_L_ANK_angle_y = L_ANK_angle_y(LHS(i-1):LHS(i)-1,1);
    yy_L_ANK_angle_z = L_ANK_angle_z(LHS(i-1):LHS(i)-1,1);
    yy_L_KNE_angle_x = L_KNE_angle_x(LHS(i-1):LHS(i)-1,1);
    yy_L_KNE_angle_y = L_KNE_angle_y(LHS(i-1):LHS(i)-1,1);
    yy_L_KNE_angle_z = L_KNE_angle_z(LHS(i-1):LHS(i)-1,1);
    yy_L_HIP_angle_x = L_HIP_angle_x(LHS(i-1):LHS(i)-1,1);
    yy_L_HIP_angle_y = L_HIP_angle_y(LHS(i-1):LHS(i)-1,1);
    yy_L_HIP_angle_z = L_HIP_angle_z(LHS(i-1):LHS(i)-1,1);
    
    xspa = 0:0.01:100 ;
    
    temp10 = interp1(xx,yy_L_ANK_angle_x,xspa);
    temp11 = interp1(xx,yy_L_ANK_angle_y,xspa);
    temp12 = interp1(xx,yy_L_ANK_angle_z,xspa);
    temp13 = interp1(xx,yy_L_KNE_angle_x,xspa);
    temp14 = interp1(xx,yy_L_KNE_angle_y,xspa);
    temp15 = interp1(xx,yy_L_KNE_angle_z,xspa);
    temp16 = interp1(xx,yy_L_HIP_angle_x,xspa);
    temp17 = interp1(xx,yy_L_HIP_angle_y,xspa);
    temp18 = interp1(xx,yy_L_HIP_angle_z,xspa);

    PSD_L_ANK_angle_x = [PSD_L_ANK_angle_x , temp10'];
    PSD_L_ANK_angle_y = [PSD_L_ANK_angle_y , temp11'];
    PSD_L_ANK_angle_z = [PSD_L_ANK_angle_z , temp12'];
    PSD_L_KNE_angle_x = [PSD_L_KNE_angle_x , temp13'];
    PSD_L_KNE_angle_y = [PSD_L_KNE_angle_y , temp14'];
    PSD_L_KNE_angle_z = [PSD_L_KNE_angle_z , temp15'];
    PSD_L_HIP_angle_x = [PSD_L_HIP_angle_x , temp16'];
    PSD_L_HIP_angle_y = [PSD_L_HIP_angle_y , temp17'];
    PSD_L_HIP_angle_z = [PSD_L_HIP_angle_z , temp18'];
end

A_HIP{j,1} = PSD_R_HIP_angle_x;
A_HIP{j,2} = PSD_R_HIP_angle_y;
A_HIP{j,3} = PSD_R_HIP_angle_z;
A_HIP{j,4} = PSD_L_HIP_angle_x;
A_HIP{j,5} = PSD_L_HIP_angle_y;
A_HIP{j,6} = PSD_L_HIP_angle_z;

A_ANK{j,1} = PSD_R_ANK_angle_x;
A_ANK{j,2} = PSD_R_ANK_angle_y;
A_ANK{j,3} = PSD_R_ANK_angle_z;
A_ANK{j,4} = PSD_L_ANK_angle_x;
A_ANK{j,5} = PSD_L_ANK_angle_y;
A_ANK{j,6} = PSD_L_ANK_angle_z;

A_KNE{j,1} = PSD_R_KNE_angle_x;
A_KNE{j,2} = PSD_R_KNE_angle_y;
A_KNE{j,3} = PSD_R_KNE_angle_z;
A_KNE{j,4} = PSD_L_KNE_angle_x;
A_KNE{j,5} = PSD_L_KNE_angle_y;
A_KNE{j,6} = PSD_L_KNE_angle_z;

A_PEL{j,1} = PSD_Pelvis_tilt_x;
A_PEL{j,2} = PSD_Pelvis_tilt_y;
A_PEL{j,3} = PSD_Pelvis_tilt_z;

COM{j,1} = PSD_COM_x;
COM{j,2} = PSD_COM_y;
COM{j,3} = PSD_COM_z;

COM_raw{j,1} = COM_x(RHS(end-119,1):RHS(end,1),1);
COM_raw{j,2} = COM_y(RHS(end-119,1):RHS(end,1),1);
COM_raw{j,3} = COM_z(RHS(end-119,1):RHS(end,1),1);

end


COM_Normalwalking = [mean([COM{1,1},COM{6,1},COM{11,1},COM{16,1},COM{21,1},COM{26,1},COM{31,1}],2,'omitnan'),...
    mean([COM{1,2},COM{6,2},COM{11,2},COM{16,2},COM{21,2},COM{26,2},COM{31,2}],2,'omitnan'),...
    mean([COM{1,3},COM{6,3},COM{11,3},COM{16,3},COM{21,3},COM{26,3},COM{31,3}],2,'omitnan')];

COM_profile1 = [mean([COM{2,1},COM{7,1},COM{12,1},COM{17,1},COM{22,1},COM{27,1},COM{32,1}],2,'omitnan'),...
    mean([COM{2,2},COM{7,2},COM{12,2},COM{17,2},COM{22,2},COM{27,2},COM{32,2}],2,'omitnan'),...
    mean([COM{2,3},COM{7,3},COM{12,3},COM{17,3},COM{22,3},COM{27,3},COM{32,3}],2,'omitnan')];
COM_profile2 = [mean([COM{3,1},COM{8,1},COM{13,1},COM{18,1},COM{23,1},COM{28,1},COM{33,1}],2,'omitnan'),...
    mean([COM{3,2},COM{8,2},COM{13,2},COM{18,2},COM{23,2},COM{28,2},COM{33,2}],2,'omitnan'),...
    mean([COM{3,3},COM{8,3},COM{13,3},COM{18,3},COM{23,3},COM{28,3},COM{33,3}],2,'omitnan')];
COM_profile3 = [mean([COM{4,1},COM{9,1},COM{14,1},COM{19,1},COM{24,1},COM{29,1},COM{34,1}],2,'omitnan'),...
    mean([COM{4,2},COM{9,2},COM{14,2},COM{19,2},COM{24,2},COM{29,2},COM{34,2}],2,'omitnan'),...
    mean([COM{4,3},COM{9,3},COM{14,3},COM{19,3},COM{24,3},COM{29,3},COM{34,3}],2,'omitnan')];
COM_profile4 = [mean([COM{5,1},COM{10,1},COM{15,1},COM{20,1},COM{25,1},COM{30,1},COM{35,1}],2,'omitnan'),...
    mean([COM{5,2},COM{10,2},COM{15,2},COM{20,2},COM{25,2},COM{30,2},COM{35,2}],2,'omitnan'),...
    mean([COM{5,3},COM{10,3},COM{15,3},COM{20,3},COM{25,3},COM{30,3},COM{35,3}],2,'omitnan')];



view(3)
hold all
plot3(COM_Normalwalking(:,1)-COM_Normalwalking(1,1),COM_Normalwalking(:,2)-COM_Normalwalking(1,2),...
    COM_Normalwalking(:,3)-COM_Normalwalking(1,3),'-','linewidth',4,'color',[100/255 100/255 102/255]),...
plot3(COM_profile1(:,1)-COM_profile1(1,1),COM_profile1(:,2)-COM_profile1(1,1),...
COM_profile1(:,3)-COM_profile1(1,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot3(COM_profile2(:,1)-COM_profile2(1,1),COM_profile2(:,2)-COM_profile2(1,2),...
    COM_profile2(:,3)-COM_profile2(1,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot3(COM_profile3(:,1)-COM_profile3(1,1),COM_profile3(:,2)-COM_profile3(1,2),...
    COM_profile3(:,3)-COM_profile3(1,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot3(COM_profile4(:,1)-COM_profile4(1,1),COM_profile4(:,2)-COM_profile4(1,2),...
    COM_profile4(:,3)-COM_profile4(1,3),'-','linewidth',4,'color',[67/255 172/255 198/255]); 
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
legend('NW','p1','p2','p3','p4')

view(2)
hold on
plot(COM_Normalwalking(:,1)-COM_Normalwalking(1,1),...
    COM_Normalwalking(:,3)-COM_Normalwalking(1,3),'-','linewidth',4,'color',[100/255 100/255 102/255]),...
plot(COM_profile1(:,1)-COM_profile1(1,1),...
COM_profile1(:,3)-COM_profile1(1,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_profile2(:,1)-COM_profile2(1,1),...
    COM_profile2(:,3)-COM_profile2(1,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_profile3(:,1)-COM_profile3(1,1),...
    COM_profile3(:,3)-COM_profile3(1,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_profile4(:,1)-COM_profile4(1,1),...
    COM_profile4(:,3)-COM_profile4(1,3),'-','linewidth',4,'color',[67/255 172/255 198/255]); 
xlabel('x [m]')
ylabel('z [m]')
% zlabel('z [m]')
legend('NW','p1','p2','p3','p4')


xxs = (0:0.01:60)' ; 
figure(334)
hold on
scatter3(COM_Normalwalking(:,1),COM_Normalwalking(:,2),COM_Normalwalking(:,3))
scatter3(COM_profile1(:,1),COM_profile1(:,2),COM_profile1(:,3))
scatter3(COM_profile2(:,1),COM_profile2(:,2),COM_profile2(:,3))
scatter3(COM_profile3(:,1),COM_profile3(:,2),COM_profile3(:,3))
scatter3(COM_profile4(:,1),COM_profile4(:,2),COM_profile4(:,3))


figure(443)
hold on
plot(COM_Normalwalking(:,1)-COM_Normalwalking(1,1),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(COM_profile1(:,1)-COM_profile1(1,1),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(COM_profile2(:,1)-COM_profile2(1,1),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(COM_profile3(:,1)-COM_profile3(1,1),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(COM_profile4(:,1)-COM_profile4(1,1),'-','linewidth',4,'color',[67/255 172/255 198/255])
legend('NW','p1','p2','p3','p4')

figure(4434)
hold on
plot(xxs,COM_Normalwalking(1:6001,3)-COM_Normalwalking(1,3),'-','linewidth',4,'color',[100/255 100/255 102/255])
plot(xxs,COM_profile1(1:6001,3)-COM_profile1(1,3),'-','linewidth',4,'color',[194/255 81/255 79/255])
plot(xxs,COM_profile2(1:6001,3)-COM_profile2(1,3),'-','linewidth',4,'color',[248/255 153/255 55/255])
plot(xxs,COM_profile3(1:6001,3)-COM_profile3(1,3),'-','linewidth',4,'color',[153/255 204/255 103/255])
plot(xxs,COM_profile4(1:6001,3)-COM_profile4(1,3),'-','linewidth',4,'color',[67/255 172/255 198/255])
xlabel('Gait cycle percentage from Heel-strike [%]')
ylabel('z [m]')
legend('NW','p1','p2','p3','p4')

hdiff(1,:) = [max((COM_Normalwalking(1001:5001,3)-COM_Normalwalking(1,3))), min((COM_Normalwalking(1001:5001,3)-COM_Normalwalking(1,3)))];
hdiff(2,:) = [max((COM_profile1(1001:5001,3)-COM_profile1(1,3))), min((COM_profile1(1001:5001,3)-COM_profile1(1,3)))];
hdiff(3,:) = [max((COM_profile2(1001:5001,3)-COM_profile2(1,3))), min((COM_profile2(1001:5001,3)-COM_profile2(1,3)))];
hdiff(4,:) = [max((COM_profile3(1001:5001,3)-COM_profile3(1,3))), min((COM_profile3(1001:5001,3)-COM_profile3(1,3)))];
hdiff(5,:) = [max((COM_profile4(1001:5001,3)-COM_profile4(1,3))), min((COM_profile4(1001:5001,3)-COM_profile4(1,3)))];

hdiff(:,3) = hdiff(:,1)-hdiff(:,2);

MR = [0.569318024;0.568870211;-0.250501941;-0.020136303;0.715473903;0.553591787;0.311016317;0.59020393;...
0.481116686;-1.231036884;-0.484797051;-0.116867692;0.739046853;0.150622755;-0.328348905;-0.669830257;...
-0.614396037;-0.912664224;-1.425754553;-0.814452837;-0.030090196;-0.429952193;-0.866300842;-0.847443801;...
0.110711263;0.003337765;-0.468473814;-0.474382603];

MRS = [4.101	4.383	3.916	3.600	3.765];
MRSTD = [0.441	0.499	0.602	0.479	0.557];


figure1 = figure('Color',[1 1 1]);
% ylabel = ('Metabolic rate [W kg^-1]');
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(sizes,MRS,0.6,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
hold on
er = errorbar(sizes,MRS,-MRSTD,MRSTD);    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 1;
yticks([2 3 4 5 6])
% ylim([2 5])
% a = get(gca,'XTickLabel');
% set(gca,'XTickLabel',a,'FontName','Arial','fontsize',10)
a = get(gca,'YTickLabel');
set(gca,'YTickLabel',a,'FontName','Arial','fontsize',10)
%%
HIP_ABD.sub01.cha.bw = 75;
HIP_ABD.sub02.cha.bw = 73;
HIP_ABD.sub03.cha.bw = 70;
HIP_ABD.sub04.cha.bw = 67;
HIP_ABD.sub05.cha.bw = 65;
HIP_ABD.sub06.cha.bw = 86;
HIP_ABD.sub07.cha.bw = 94;
HIP_ABD.sub01.cha.height = 1.83;
HIP_ABD.sub02.cha.height = 1.75;
HIP_ABD.sub03.cha.height = 1.63;
HIP_ABD.sub05.cha.height = 1.68;
HIP_ABD.sub06.cha.height = 1.80;
HIP_ABD.sub07.cha.height = 1.77;

%% ctl param save
sub01.NW.ctl_param = [];
sub02.NW.ctl_param = [];
sub03.NW.ctl_param = [];
sub04.NW.ctl_param = [];
sub05.NW.ctl_param = [];
sub06.NW.ctl_param = [];
sub07.NW.ctl_param = [];
%% power

% sub01.MFMP.ctl_param.power.raw = ctl{2,6};
sub01.MFMP.ctl_param.power.mean = mean(ctl{2,6},2,'omitnan');
sub01.MFMP.ctl_param.power.std = std(ctl{2,6},0,2,'omitnan');
sub01.MFMP.ctl_param.power.unit = 'W/kg';
% sub01.PFMP.ctl_param.power.raw = ctl{3,6};
sub01.PFMP.ctl_param.power.mean = mean(ctl{3,6},2,'omitnan');
sub01.PFMP.ctl_param.power.std = std(ctl{3,6},0,2,'omitnan');
sub01.PFMP.ctl_param.power.unit = 'W/kg';
% sub01.MSMP.ctl_param.power.raw = ctl{4,6};
sub01.MSMP.ctl_param.power.mean = mean(ctl{4,6},2,'omitnan');
sub01.MSMP.ctl_param.power.std = std(ctl{4,6},0,2,'omitnan');
sub01.MSMP.ctl_param.power.unit = 'W/kg';
% sub01.PSMP.ctl_param.power.raw = ctl{5,6};
sub01.PSMP.ctl_param.power.mean = mean(ctl{5,6},2,'omitnan');
sub01.PSMP.ctl_param.power.std = std(ctl{5,6},0,2,'omitnan');
sub01.PSMP.ctl_param.power.unit = 'W/kg';

% sub02.MFMP.ctl_param.power.raw = ctl{7,6};
sub02.MFMP.ctl_param.power.mean = mean(ctl{7,6},2,'omitnan');
sub02.MFMP.ctl_param.power.std = std(ctl{7,6},0,2,'omitnan');
% sub02.PFMP.ctl_param.power.raw = ctl{8,6};
sub02.PFMP.ctl_param.power.mean = mean(ctl{8,6},2,'omitnan');
sub02.PFMP.ctl_param.power.std = std(ctl{8,6},0,2,'omitnan');
% sub02.MSMP.ctl_param.power.raw = ctl{9,6};
sub02.MSMP.ctl_param.power.mean = mean(ctl{9,6},2,'omitnan');
sub02.MSMP.ctl_param.power.std = std(ctl{9,6},0,2,'omitnan');
% sub02.PSMP.ctl_param.power.raw = ctl{10,6};
sub02.PSMP.ctl_param.power.mean = mean(ctl{10,6},2,'omitnan');
sub02.PSMP.ctl_param.power.std = std(ctl{10,6},0,2,'omitnan');
sub02.MFMP.ctl_param.power.unit = 'W/kg';
sub02.PFMP.ctl_param.power.unit = 'W/kg';
sub02.MSMP.ctl_param.power.unit = 'W/kg';
sub02.PSMP.ctl_param.power.unit = 'W/kg';

% sub03.MFMP.ctl_param.power.raw = ctl{12,6};
sub03.MFMP.ctl_param.power.mean = mean(ctl{12,6},2,'omitnan');
sub03.MFMP.ctl_param.power.std = std(ctl{12,6},0,2,'omitnan');
% sub03.PFMP.ctl_param.power.raw = ctl{13,6};
sub03.PFMP.ctl_param.power.mean = mean(ctl{13,6},2,'omitnan');
sub03.PFMP.ctl_param.power.std = std(ctl{13,6},0,2,'omitnan');
% sub03.MSMP.ctl_param.power.raw = ctl{14,6};
sub03.MSMP.ctl_param.power.mean = mean(ctl{14,6},2,'omitnan');
sub03.MSMP.ctl_param.power.std = std(ctl{14,6},0,2,'omitnan');
% sub03.PSMP.ctl_param.power.raw = ctl{15,6};
sub03.PSMP.ctl_param.power.mean = mean(ctl{15,6},2,'omitnan');
sub03.PSMP.ctl_param.power.std = std(ctl{15,6},0,2,'omitnan');
sub03.MFMP.ctl_param.power.unit = 'W/kg';
sub03.PFMP.ctl_param.power.unit = 'W/kg';
sub03.MSMP.ctl_param.power.unit = 'W/kg';
sub03.PSMP.ctl_param.power.unit = 'W/kg';

% sub04.MFMP.ctl_param.power.raw = ctl{17,6};
sub04.MFMP.ctl_param.power.mean = mean(ctl{17,6},2,'omitnan');
sub04.MFMP.ctl_param.power.std = std(ctl{17,6},0,2,'omitnan');
% sub04.PFMP.ctl_param.power.raw = ctl{18,6};
sub04.PFMP.ctl_param.power.mean = mean(ctl{18,6},2,'omitnan');
sub04.PFMP.ctl_param.power.std = std(ctl{18,6},0,2,'omitnan');
% sub04.MSMP.ctl_param.power.raw = ctl{19,6};
sub04.MSMP.ctl_param.power.mean = mean(ctl{19,6},2,'omitnan');
sub04.MSMP.ctl_param.power.std = std(ctl{19,6},0,2,'omitnan');
% sub04.PSMP.ctl_param.power.raw = ctl{20,6};
sub04.PSMP.ctl_param.power.mean = mean(ctl{20,6},2,'omitnan');
sub04.PSMP.ctl_param.power.std = std(ctl{20,6},0,2,'omitnan');
sub04.MFMP.ctl_param.power.unit = 'W/kg';
sub04.PFMP.ctl_param.power.unit = 'W/kg';
sub04.MSMP.ctl_param.power.unit = 'W/kg';
sub04.PSMP.ctl_param.power.unit = 'W/kg';

% sub05.MFMP.ctl_param.power.raw = ctl{22,6};
sub05.MFMP.ctl_param.power.mean = mean(ctl{22,6},2,'omitnan');
sub05.MFMP.ctl_param.power.std = std(ctl{22,6},0,2,'omitnan');
% sub05.PFMP.ctl_param.power.raw = ctl{23,6};
sub05.PFMP.ctl_param.power.mean = mean(ctl{23,6},2,'omitnan');
sub05.PFMP.ctl_param.power.std = std(ctl{23,6},0,2,'omitnan');
% sub05.MSMP.ctl_param.power.raw = ctl{24,6};
sub05.MSMP.ctl_param.power.mean = mean(ctl{24,6},2,'omitnan');
sub05.MSMP.ctl_param.power.std = std(ctl{24,6},0,2,'omitnan');
% sub05.PSMP.ctl_param.power.raw = ctl{25,6};
sub05.PSMP.ctl_param.power.mean = mean(ctl{25,6},2,'omitnan');
sub05.PSMP.ctl_param.power.std = std(ctl{25,6},0,2,'omitnan');
sub05.MFMP.ctl_param.power.unit = 'W/kg';
sub05.PFMP.ctl_param.power.unit = 'W/kg';
sub05.MSMP.ctl_param.power.unit = 'W/kg';
sub05.PSMP.ctl_param.power.unit = 'W/kg';


% sub06.MFMP.ctl_param.power.raw = ctl{27,6};
sub06.MFMP.ctl_param.power.mean = mean(ctl{27,6},2,'omitnan');
sub06.MFMP.ctl_param.power.std = std(ctl{27,6},0,2,'omitnan');
% sub06.PFMP.ctl_param.power.raw = ctl{28,6};
sub06.PFMP.ctl_param.power.mean = mean(ctl{28,6},2,'omitnan');
sub06.PFMP.ctl_param.power.std = std(ctl{28,6},0,2,'omitnan');
% sub06.MSMP.ctl_param.power.raw = ctl{29,6};
sub06.MSMP.ctl_param.power.mean = mean(ctl{29,6},2,'omitnan');
sub06.MSMP.ctl_param.power.std = std(ctl{29,6},0,2,'omitnan');
% sub06.PSMP.ctl_param.power.raw = ctl{30,6};
sub06.PSMP.ctl_param.power.mean = mean(ctl{30,6},2,'omitnan');
sub06.PSMP.ctl_param.power.std = std(ctl{30,6},0,2,'omitnan');
sub06.MFMP.ctl_param.power.unit = 'W/kg';
sub06.PFMP.ctl_param.power.unit = 'W/kg';
sub06.MSMP.ctl_param.power.unit = 'W/kg';
sub06.PSMP.ctl_param.power.unit = 'W/kg';

% sub07.MFMP.ctl_param.power.raw = ctl{32,6};
sub07.MFMP.ctl_param.power.mean = mean(ctl{32,6},2,'omitnan');
sub07.MFMP.ctl_param.power.std = std(ctl{32,6},0,2,'omitnan');
% sub07.PFMP.ctl_param.power.raw = ctl{33,6};
sub07.PFMP.ctl_param.power.mean = mean(ctl{33,6},2,'omitnan');
sub07.PFMP.ctl_param.power.std = std(ctl{33,6},0,2,'omitnan');
% sub07.MSMP.ctl_param.power.raw = ctl{34,6};
sub07.MSMP.ctl_param.power.mean = mean(ctl{34,6},2,'omitnan');
sub07.MSMP.ctl_param.power.std = std(ctl{34,6},0,2,'omitnan');
% sub07.PSMP.ctl_param.power.raw = ctl{35,6};
sub07.PSMP.ctl_param.power.mean = mean(ctl{35,6},2,'omitnan');
sub07.PSMP.ctl_param.power.std = std(ctl{35,6},0,2,'omitnan');
sub07.MFMP.ctl_param.power.unit = 'W/kg';
sub07.PFMP.ctl_param.power.unit = 'W/kg';
sub07.MSMP.ctl_param.power.unit = 'W/kg';
sub07.PSMP.ctl_param.power.unit = 'W/kg';
%% ref force

% sub01.MFMP.ctl_param.ref_force_R.raw = ctl{2,1};
sub01.MFMP.ctl_param.ref_force_R.mean = mean(ctl{2,1},2,'omitnan');
sub01.MFMP.ctl_param.ref_force_R.std = std(ctl{2,1},0,2,'omitnan');
% sub01.PFMP.ctl_param.ref_force_R.raw = ctl{3,1};
sub01.PFMP.ctl_param.force.mean = mean(ctl{3,1},2,'omitnan');
sub01.PFMP.ctl_param.ref_force_R.std = std(ctl{3,1},0,2,'omitnan');
% sub01.MSMP.ctl_param.ref_force_R.raw = ctl{4,1};
sub01.MSMP.ctl_param.ref_force_R.mean = mean(ctl{4,1},2,'omitnan');
sub01.MSMP.ctl_param.ref_force_R.std = std(ctl{4,1},0,2,'omitnan');
% sub01.PSMP.ctl_param.ref_force_R.raw = ctl{5,1};
sub01.PSMP.ctl_param.ref_force_R.mean = mean(ctl{5,1},2,'omitnan');
sub01.PSMP.ctl_param.ref_force_R.std = std(ctl{5,1},0,2,'omitnan');

% sub02.MFMP.ctl_param.ref_force_R.raw = ctl{7,1};
sub02.MFMP.ctl_param.ref_force_R.mean = mean(ctl{7,1},2,'omitnan');
sub02.MFMP.ctl_param.ref_force_R.std = std(ctl{7,1},0,2,'omitnan');
% sub02.PFMP.ctl_param.ref_force_R.raw = ctl{8,1};
sub02.PFMP.ctl_param.ref_force_R.mean = mean(ctl{8,1},2,'omitnan');
sub02.PFMP.ctl_param.ref_force_R.std = std(ctl{8,1},0,2,'omitnan');
% sub02.MSMP.ctl_param.ref_force_R.raw = ctl{9,1};
sub02.MSMP.ctl_param.ref_force_R.mean = mean(ctl{9,1},2,'omitnan');
sub02.MSMP.ctl_param.ref_force_R.std = std(ctl{9,1},0,2,'omitnan');
% sub02.PSMP.ctl_param.ref_force_R.raw = ctl{10,1};
sub02.PSMP.ctl_param.ref_force_R.mean = mean(ctl{10,1},2,'omitnan');
sub02.PSMP.ctl_param.ref_force_R.std = std(ctl{10,1},0,2,'omitnan');

% sub03.MFMP.ctl_param.ref_force_R.raw = ctl{12,1};
sub03.MFMP.ctl_param.ref_force_R.mean = mean(ctl{12,1},2,'omitnan');
sub03.MFMP.ctl_param.ref_force_R.std = std(ctl{12,1},0,2,'omitnan');
% sub03.PFMP.ctl_param.ref_force_R.raw = ctl{13,1};
sub03.PFMP.ctl_param.ref_force_R.mean = mean(ctl{13,1},2,'omitnan');
sub03.PFMP.ctl_param.ref_force_R.std = std(ctl{13,1},0,2,'omitnan');
% sub03.MSMP.ctl_param.ref_force_R.raw = ctl{14,1};
sub03.MSMP.ctl_param.ref_force_R.mean = mean(ctl{14,1},2,'omitnan');
sub03.MSMP.ctl_param.ref_force_R.std = std(ctl{14,1},0,2,'omitnan');
% sub03.PSMP.ctl_param.ref_force_R.raw = ctl{15,1};
sub03.PSMP.ctl_param.ref_force_R.mean = mean(ctl{15,1},2,'omitnan');
sub03.PSMP.ctl_param.ref_force_R.std = std(ctl{15,1},0,2,'omitnan');

% sub04.MFMP.ctl_param.ref_force_R.raw = ctl{17,1};
sub04.MFMP.ctl_param.ref_force_R.mean = mean(ctl{17,1},2,'omitnan');
sub04.MFMP.ctl_param.ref_force_R.std = std(ctl{17,1},0,2,'omitnan');
% sub04.PFMP.ctl_param.ref_force_R.raw = ctl{18,1};
sub04.PFMP.ctl_param.ref_force_R.mean = mean(ctl{18,1},2,'omitnan');
sub04.PFMP.ctl_param.ref_force_R.std = std(ctl{18,1},0,2,'omitnan');
% sub04.MSMP.ctl_param.ref_force_R.raw = ctl{19,1};
sub04.MSMP.ctl_param.ref_force_R.mean = mean(ctl{19,1},2,'omitnan');
sub04.MSMP.ctl_param.ref_force_R.std = std(ctl{19,1},0,2,'omitnan');
% sub04.PSMP.ctl_param.ref_force_R.raw = ctl{20,1};
sub04.PSMP.ctl_param.ref_force_R.mean = mean(ctl{20,1},2,'omitnan');
sub04.PSMP.ctl_param.ref_force_R.std = std(ctl{20,1},0,2,'omitnan');

% sub05.MFMP.ctl_param.ref_force_R.raw = ctl{22,1};
sub05.MFMP.ctl_param.ref_force_R.mean = mean(ctl{22,1},2,'omitnan');
sub05.MFMP.ctl_param.ref_force_R.std = std(ctl{22,1},0,2,'omitnan');
% sub05.PFMP.ctl_param.ref_force_R.raw = ctl{23,1};
sub05.PFMP.ctl_param.ref_force_R.mean = mean(ctl{23,1},2,'omitnan');
sub05.PFMP.ctl_param.ref_force_R.std = std(ctl{23,1},0,2,'omitnan');
% sub05.MSMP.ctl_param.ref_force_R.raw = ctl{24,1};
sub05.MSMP.ctl_param.ref_force_R.mean = mean(ctl{24,1},2,'omitnan');
sub05.MSMP.ctl_param.ref_force_R.std = std(ctl{24,1},0,2,'omitnan');
% sub05.PSMP.ctl_param.ref_force_R.raw = ctl{25,1};
sub05.PSMP.ctl_param.ref_force_R.mean = mean(ctl{25,1},2,'omitnan');
sub05.PSMP.ctl_param.ref_force_R.std = std(ctl{25,1},0,2,'omitnan');

% sub06.MFMP.ctl_param.ref_force_R.raw = ctl{27,1};
sub06.MFMP.ctl_param.ref_force_R.mean = mean(ctl{27,1},2,'omitnan');
sub06.MFMP.ctl_param.ref_force_R.std = std(ctl{27,1},0,2,'omitnan');
% sub06.PFMP.ctl_param.ref_force_R.raw = ctl{28,1};
sub06.PFMP.ctl_param.ref_force_R.mean = mean(ctl{28,1},2,'omitnan');
sub06.PFMP.ctl_param.ref_force_R.std = std(ctl{28,1},0,2,'omitnan');
% sub06.MSMP.ctl_param.ref_force_R.raw = ctl{29,1};
sub06.MSMP.ctl_param.ref_force_R.mean = mean(ctl{29,1},2,'omitnan');
sub06.MSMP.ctl_param.ref_force_R.std = std(ctl{29,1},0,2,'omitnan');
% sub06.PSMP.ctl_param.ref_force_R.raw = ctl{30,1};
sub06.PSMP.ctl_param.ref_force_R.mean = mean(ctl{30,1},2,'omitnan');
sub06.PSMP.ctl_param.ref_force_R.std = std(ctl{30,1},0,2,'omitnan');

% sub07.MFMP.ctl_param.ref_force_R.raw = ctl{32,1};
sub07.MFMP.ctl_param.ref_force_R.mean = mean(ctl{32,1},2,'omitnan');
sub07.MFMP.ctl_param.ref_force_R.std = std(ctl{32,1},0,2,'omitnan');
% sub07.PFMP.ctl_param.ref_force_R.raw = ctl{33,1};
sub07.PFMP.ctl_param.ref_force_R.mean = mean(ctl{33,1},2,'omitnan');
sub07.PFMP.ctl_param.ref_force_R.std = std(ctl{33,1},0,2,'omitnan');
% sub07.MSMP.ctl_param.ref_force_R.raw = ctl{34,1};
sub07.MSMP.ctl_param.ref_force_R.mean = mean(ctl{34,1},2,'omitnan');
sub07.MSMP.ctl_param.ref_force_R.std = std(ctl{34,1},0,2,'omitnan');
% sub07.PSMP.ctl_param.ref_force_R.raw = ctl{35,1};
sub07.PSMP.ctl_param.ref_force_R.mean = mean(ctl{35,1},2,'omitnan');
sub07.PSMP.ctl_param.ref_force_R.std = std(ctl{35,1},0,2,'omitnan');

sub01.MFMP.ctl_param.ref_force_R.unit = 'N';
sub01.PFMP.ctl_param.ref_force_R.unit = 'N';
sub01.MSMP.ctl_param.ref_force_R.unit = 'N';
sub01.PSMP.ctl_param.ref_force_R.unit = 'N';
sub02.MFMP.ctl_param.ref_force_R.unit = 'N';
sub02.PFMP.ctl_param.ref_force_R.unit = 'N';
sub02.MSMP.ctl_param.ref_force_R.unit = 'N';
sub02.PSMP.ctl_param.ref_force_R.unit = 'N';
sub03.MFMP.ctl_param.ref_force_R.unit = 'N';
sub03.PFMP.ctl_param.ref_force_R.unit = 'N';
sub03.MSMP.ctl_param.ref_force_R.unit = 'N';
sub03.PSMP.ctl_param.ref_force_R.unit = 'N';
sub04.MFMP.ctl_param.ref_force_R.unit = 'N';
sub04.PFMP.ctl_param.ref_force_R.unit = 'N';
sub04.MSMP.ctl_param.ref_force_R.unit = 'N';
sub04.PSMP.ctl_param.ref_force_R.unit = 'N';
sub05.MFMP.ctl_param.ref_force_R.unit = 'N';
sub05.PFMP.ctl_param.ref_force_R.unit = 'N';
sub05.MSMP.ctl_param.ref_force_R.unit = 'N';
sub05.PSMP.ctl_param.ref_force_R.unit = 'N';
sub06.MFMP.ctl_param.ref_force_R.unit = 'N';
sub06.PFMP.ctl_param.ref_force_R.unit = 'N';
sub06.MSMP.ctl_param.ref_force_R.unit = 'N';
sub06.PSMP.ctl_param.ref_force_R.unit = 'N';
sub07.MFMP.ctl_param.ref_force_R.unit = 'N';
sub07.PFMP.ctl_param.ref_force_R.unit = 'N';
sub07.MSMP.ctl_param.ref_force_R.unit = 'N';
sub07.PSMP.ctl_param.ref_force_R.unit = 'N';


% sub01.MFMP.ctl_param.ref_force_L.raw = ctl{2,3};
sub01.MFMP.ctl_param.ref_force_L.mean = mean(ctl{2,3},2,'omitnan');
sub01.MFMP.ctl_param.ref_force_L.std = std(ctl{2,3},0,2,'omitnan');
% sub01.PFMP.ctl_param.ref_force_L.raw = ctl{3,3};
sub01.PFMP.ctl_param.ref_force_L.mean = mean(ctl{3,3},2,'omitnan');
sub01.PFMP.ctl_param.ref_force_L.std = std(ctl{3,3},0,2,'omitnan');
% sub01.MSMP.ctl_param.ref_force_L.raw = ctl{4,3};
sub01.MSMP.ctl_param.ref_force_L.mean = mean(ctl{4,3},2,'omitnan');
sub01.MSMP.ctl_param.ref_force_L.std = std(ctl{4,3},0,2,'omitnan');
% sub01.PSMP.ctl_param.ref_force_L.raw = ctl{5,3};
sub01.PSMP.ctl_param.ref_force_L.mean = mean(ctl{5,3},2,'omitnan');
sub01.PSMP.ctl_param.ref_force_L.std = std(ctl{5,3},0,2,'omitnan');

% sub02.MFMP.ctl_param.ref_force_L.raw = ctl{7,3};
sub02.MFMP.ctl_param.ref_force_L.mean = mean(ctl{7,3},2,'omitnan');
sub02.MFMP.ctl_param.ref_force_L.std = std(ctl{7,3},0,2,'omitnan');
% sub02.PFMP.ctl_param.ref_force_L.raw = ctl{8,3};
sub02.PFMP.ctl_param.ref_force_L.mean = mean(ctl{8,3},2,'omitnan');
sub02.PFMP.ctl_param.ref_force_L.std = std(ctl{8,3},0,2,'omitnan');
% sub02.MSMP.ctl_param.ref_force_L.raw = ctl{9,3};
sub02.MSMP.ctl_param.ref_force_L.mean = mean(ctl{9,3},2,'omitnan');
sub02.MSMP.ctl_param.ref_force_L.std = std(ctl{9,3},0,2,'omitnan');
% sub02.PSMP.ctl_param.ref_force_L.raw = ctl{10,3};
sub02.PSMP.ctl_param.ref_force_L.mean = mean(ctl{10,3},2,'omitnan');
sub02.PSMP.ctl_param.ref_force_L.std = std(ctl{10,3},0,2,'omitnan');

% sub03.MFMP.ctl_param.ref_force_L.raw = ctl{12,3};
sub03.MFMP.ctl_param.ref_force_L.mean = mean(ctl{12,3},2,'omitnan');
sub03.MFMP.ctl_param.ref_force_L.std = std(ctl{12,3},0,2,'omitnan');
% sub03.PFMP.ctl_param.ref_force_L.raw = ctl{13,3};
sub03.PFMP.ctl_param.ref_force_L.mean = mean(ctl{13,3},2,'omitnan');
sub03.PFMP.ctl_param.ref_force_L.std = std(ctl{13,3},0,2,'omitnan');
% sub03.MSMP.ctl_param.ref_force_L.raw = ctl{14,3};
sub03.MSMP.ctl_param.ref_force_L.mean = mean(ctl{14,3},2,'omitnan');
sub03.MSMP.ctl_param.ref_force_L.std = std(ctl{14,3},0,2,'omitnan');
% sub03.PSMP.ctl_param.ref_force_L.raw = ctl{15,3};
sub03.PSMP.ctl_param.ref_force_L.mean = mean(ctl{15,3},2,'omitnan');
sub03.PSMP.ctl_param.ref_force_L.std = std(ctl{15,3},0,2,'omitnan');

% sub04.MFMP.ctl_param.ref_force_L.raw = ctl{17,3};
sub04.MFMP.ctl_param.ref_force_L.mean = mean(ctl{17,3},2,'omitnan');
sub04.MFMP.ctl_param.ref_force_L.std = std(ctl{17,3},0,2,'omitnan');
% sub04.PFMP.ctl_param.ref_force_L.raw = ctl{18,3};
sub04.PFMP.ctl_param.ref_force_L.mean = mean(ctl{18,3},2,'omitnan');
sub04.PFMP.ctl_param.ref_force_L.std = std(ctl{18,3},0,2,'omitnan');
% sub04.MSMP.ctl_param.ref_force_L.raw = ctl{19,3};
sub04.MSMP.ctl_param.ref_force_L.mean = mean(ctl{19,3},2,'omitnan');
sub04.MSMP.ctl_param.ref_force_L.std = std(ctl{19,3},0,2,'omitnan');
% sub04.PSMP.ctl_param.ref_force_L.raw = ctl{20,3};
sub04.PSMP.ctl_param.ref_force_L.mean = mean(ctl{20,3},2,'omitnan');
sub04.PSMP.ctl_param.ref_force_L.std = std(ctl{20,3},0,2,'omitnan');

% sub05.MFMP.ctl_param.ref_force_L.raw = ctl{22,3};
sub05.MFMP.ctl_param.ref_force_L.mean = mean(ctl{22,3},2,'omitnan');
sub05.MFMP.ctl_param.ref_force_L.std = std(ctl{22,3},0,2,'omitnan');
% sub05.PFMP.ctl_param.ref_force_L.raw = ctl{23,3};
sub05.PFMP.ctl_param.ref_force_L.mean = mean(ctl{23,3},2,'omitnan');
sub05.PFMP.ctl_param.ref_force_L.std = std(ctl{23,3},0,2,'omitnan');
% sub05.MSMP.ctl_param.ref_force_L.raw = ctl{24,3};
sub05.MSMP.ctl_param.ref_force_L.mean = mean(ctl{24,3},2,'omitnan');
sub05.MSMP.ctl_param.ref_force_L.std = std(ctl{24,3},0,2,'omitnan');
% sub05.PSMP.ctl_param.ref_force_L.raw = ctl{25,3};
sub05.PSMP.ctl_param.ref_force_L.mean = mean(ctl{25,3},2,'omitnan');
sub05.PSMP.ctl_param.ref_force_L.std = std(ctl{25,3},0,2,'omitnan');

% sub06.MFMP.ctl_param.ref_force_L.raw = ctl{27,3};
sub06.MFMP.ctl_param.ref_force_L.mean = mean(ctl{27,3},2,'omitnan');
sub06.MFMP.ctl_param.ref_force_L.std = std(ctl{27,3},0,2,'omitnan');
% sub06.PFMP.ctl_param.ref_force_L.raw = ctl{28,3};
sub06.PFMP.ctl_param.ref_force_L.mean = mean(ctl{28,3},2,'omitnan');
sub06.PFMP.ctl_param.ref_force_L.std = std(ctl{28,3},0,2,'omitnan');
% sub06.MSMP.ctl_param.ref_force_L.raw = ctl{29,3};
sub06.MSMP.ctl_param.ref_force_L.mean = mean(ctl{29,3},2,'omitnan');
sub06.MSMP.ctl_param.ref_force_L.std = std(ctl{29,3},0,2,'omitnan');
% sub06.PSMP.ctl_param.ref_force_L.raw = ctl{30,3};
sub06.PSMP.ctl_param.ref_force_L.mean = mean(ctl{30,3},2,'omitnan');
sub06.PSMP.ctl_param.ref_force_L.std = std(ctl{30,3},0,2,'omitnan');

% sub07.MFMP.ctl_param.ref_force_L.raw = ctl{32,3};
sub07.MFMP.ctl_param.ref_force_L.mean = mean(ctl{32,3},2,'omitnan');
sub07.MFMP.ctl_param.ref_force_L.std = std(ctl{32,3},0,2,'omitnan');
% sub07.PFMP.ctl_param.ref_force_L.raw = ctl{33,3};
sub07.PFMP.ctl_param.ref_force_L.mean = mean(ctl{33,3},2,'omitnan');
sub07.PFMP.ctl_param.ref_force_L.std = std(ctl{33,3},0,2,'omitnan');
% sub07.MSMP.ctl_param.ref_force_L.raw = ctl{34,3};
sub07.MSMP.ctl_param.ref_force_L.mean = mean(ctl{34,3},2,'omitnan');
sub07.MSMP.ctl_param.ref_force_L.std = std(ctl{34,3},0,2,'omitnan');
% sub07.PSMP.ctl_param.ref_force_L.raw = ctl{35,3};
sub07.PSMP.ctl_param.ref_force_L.mean = mean(ctl{35,3},2,'omitnan');
sub07.PSMP.ctl_param.ref_force_L.std = std(ctl{35,3},0,2,'omitnan');

sub01.MFMP.ctl_param.ref_force_L.unit = 'N';
sub01.PFMP.ctl_param.ref_force_L.unit = 'N';
sub01.MSMP.ctl_param.ref_force_L.unit = 'N';
sub01.PSMP.ctl_param.ref_force_L.unit = 'N';
sub02.MFMP.ctl_param.ref_force_L.unit = 'N';
sub02.PFMP.ctl_param.ref_force_L.unit = 'N';
sub02.MSMP.ctl_param.ref_force_L.unit = 'N';
sub02.PSMP.ctl_param.ref_force_L.unit = 'N';
sub03.MFMP.ctl_param.ref_force_L.unit = 'N';
sub03.PFMP.ctl_param.ref_force_L.unit = 'N';
sub03.MSMP.ctl_param.ref_force_L.unit = 'N';
sub03.PSMP.ctl_param.ref_force_L.unit = 'N';
sub04.MFMP.ctl_param.ref_force_L.unit = 'N';
sub04.PFMP.ctl_param.ref_force_L.unit = 'N';
sub04.MSMP.ctl_param.ref_force_L.unit = 'N';
sub04.PSMP.ctl_param.ref_force_L.unit = 'N';
sub05.MFMP.ctl_param.ref_force_L.unit = 'N';
sub05.PFMP.ctl_param.ref_force_L.unit = 'N';
sub05.MSMP.ctl_param.ref_force_L.unit = 'N';
sub05.PSMP.ctl_param.ref_force_L.unit = 'N';
sub06.MFMP.ctl_param.ref_force_L.unit = 'N';
sub06.PFMP.ctl_param.ref_force_L.unit = 'N';
sub06.MSMP.ctl_param.ref_force_L.unit = 'N';
sub06.PSMP.ctl_param.ref_force_L.unit = 'N';
sub07.MFMP.ctl_param.ref_force_L.unit = 'N';
sub07.PFMP.ctl_param.ref_force_L.unit = 'N';
sub07.MSMP.ctl_param.ref_force_L.unit = 'N';
sub07.PSMP.ctl_param.ref_force_L.unit = 'N';

%% force

% sub01.MFMP.ctl_param.force_R.raw = ctl{2,2};
sub01.MFMP.ctl_param.force_R.mean = mean(ctl{2,2},2,'omitnan');
sub01.MFMP.ctl_param.force_R.std = std(ctl{2,2},0,2,'omitnan');
% sub01.PFMP.ctl_param.force_R.raw = ctl{3,2};
sub01.PFMP.ctl_param.force_R.mean = mean(ctl{3,2},2,'omitnan');
sub01.PFMP.ctl_param.force_R.std = std(ctl{3,2},0,2,'omitnan');
% sub01.MSMP.ctl_param.force_R.raw = ctl{4,2};
sub01.MSMP.ctl_param.force_R.mean = mean(ctl{4,2},2,'omitnan');
sub01.MSMP.ctl_param.force_R.std = std(ctl{4,2},0,2,'omitnan');
% sub01.PSMP.ctl_param.force_R.raw = ctl{5,2};
sub01.PSMP.ctl_param.force_R.mean = mean(ctl{5,2},2,'omitnan');
sub01.PSMP.ctl_param.force_R.std = std(ctl{5,2},0,2,'omitnan');

% sub02.MFMP.ctl_param.force_R.raw = ctl{7,2};
sub02.MFMP.ctl_param.force_R.mean = mean(ctl{7,2},2,'omitnan');
sub02.MFMP.ctl_param.force_R.std = std(ctl{7,2},0,2,'omitnan');
% sub02.PFMP.ctl_param.force_R.raw = ctl{8,2};
sub02.PFMP.ctl_param.force_R.mean = mean(ctl{8,2},2,'omitnan');
sub02.PFMP.ctl_param.force_R.std = std(ctl{8,2},0,2,'omitnan');
% sub02.MSMP.ctl_param.force_R.raw = ctl{9,2};
sub02.MSMP.ctl_param.force_R.mean = mean(ctl{9,2},2,'omitnan');
sub02.MSMP.ctl_param.force_R.std = std(ctl{9,2},0,2,'omitnan');
% sub02.PSMP.ctl_param.force_R.raw = ctl{10,2};
sub02.PSMP.ctl_param.force_R.mean = mean(ctl{10,2},2,'omitnan');
sub02.PSMP.ctl_param.force_R.std = std(ctl{10,2},0,2,'omitnan');

% sub03.MFMP.ctl_param.force_R.raw = ctl{12,2};
sub03.MFMP.ctl_param.force_R.mean = mean(ctl{12,2},2,'omitnan');
sub03.MFMP.ctl_param.force_R.std = std(ctl{12,2},0,2,'omitnan');
% sub03.PFMP.ctl_param.force_R.raw = ctl{13,2};
sub03.PFMP.ctl_param.force_R.mean = mean(ctl{13,2},2,'omitnan');
sub03.PFMP.ctl_param.force_R.std = std(ctl{13,2},0,2,'omitnan');
% sub03.MSMP.ctl_param.force_R.raw = ctl{14,2};
sub03.MSMP.ctl_param.force_R.mean = mean(ctl{14,2},2,'omitnan');
sub03.MSMP.ctl_param.force_R.std = std(ctl{14,2},0,2,'omitnan');
% sub03.PSMP.ctl_param.force_R.raw = ctl{15,2};
sub03.PSMP.ctl_param.force_R.mean = mean(ctl{15,2},2,'omitnan');
sub03.PSMP.ctl_param.force_R.std = std(ctl{15,2},0,2,'omitnan');

% sub04.MFMP.ctl_param.force_R.raw = ctl{17,2};
sub04.MFMP.ctl_param.force_R.mean = mean(ctl{17,2},2,'omitnan');
sub04.MFMP.ctl_param.force_R.std = std(ctl{17,2},0,2,'omitnan');
% sub04.PFMP.ctl_param.force_R.raw = ctl{18,2};
sub04.PFMP.ctl_param.force_R.mean = mean(ctl{18,2},2,'omitnan');
sub04.PFMP.ctl_param.force_R.std = std(ctl{18,2},0,2,'omitnan');
% sub04.MSMP.ctl_param.force_R.raw = ctl{19,2};
sub04.MSMP.ctl_param.force_R.mean = mean(ctl{19,2},2,'omitnan');
sub04.MSMP.ctl_param.force_R.std = std(ctl{19,2},0,2,'omitnan');
% sub04.PSMP.ctl_param.force_R.raw = ctl{20,2};
sub04.PSMP.ctl_param.force_R.mean = mean(ctl{20,2},2,'omitnan');
sub04.PSMP.ctl_param.force_R.std = std(ctl{20,2},0,2,'omitnan');

% sub05.MFMP.ctl_param.force_R.raw = ctl{22,2};
sub05.MFMP.ctl_param.force_R.mean = mean(ctl{22,2},2,'omitnan');
sub05.MFMP.ctl_param.force_R.std = std(ctl{22,2},0,2,'omitnan');
% sub05.PFMP.ctl_param.force_R.raw = ctl{23,2};
sub05.PFMP.ctl_param.force_R.mean = mean(ctl{23,2},2,'omitnan');
sub05.PFMP.ctl_param.force_R.std = std(ctl{23,2},0,2,'omitnan');
% sub05.MSMP.ctl_param.force_R.raw = ctl{24,2};
sub05.MSMP.ctl_param.force_R.mean = mean(ctl{24,2},2,'omitnan');
sub05.MSMP.ctl_param.force_R.std = std(ctl{24,2},0,2,'omitnan');
% sub05.PSMP.ctl_param.force_R.raw = ctl{25,2};
sub05.PSMP.ctl_param.force_R.mean = mean(ctl{25,2},2,'omitnan');
sub05.PSMP.ctl_param.force_R.std = std(ctl{25,2},0,2,'omitnan');

% sub06.MFMP.ctl_param.force_R.raw = ctl{27,2};
sub06.MFMP.ctl_param.force_R.mean = mean(ctl{27,2},2,'omitnan');
sub06.MFMP.ctl_param.force_R.std = std(ctl{27,2},0,2,'omitnan');
% sub06.PFMP.ctl_param.force_R.raw = ctl{28,2};
sub06.PFMP.ctl_param.force_R.mean = mean(ctl{28,2},2,'omitnan');
sub06.PFMP.ctl_param.force_R.std = std(ctl{28,2},0,2,'omitnan');
% sub06.MSMP.ctl_param.force_R.raw = ctl{29,2};
sub06.MSMP.ctl_param.force_R.mean = mean(ctl{29,2},2,'omitnan');
sub06.MSMP.ctl_param.force_R.std = std(ctl{29,2},0,2,'omitnan');
% sub06.PSMP.ctl_param.force_R.raw = ctl{30,2};
sub06.PSMP.ctl_param.force_R.mean = mean(ctl{30,2},2,'omitnan');
sub06.PSMP.ctl_param.force_R.std = std(ctl{30,2},0,2,'omitnan');

% sub07.MFMP.ctl_param.force_R.raw = ctl{32,2};
sub07.MFMP.ctl_param.force_R.mean = mean(ctl{32,2},2,'omitnan');
sub07.MFMP.ctl_param.force_R.std = std(ctl{32,2},0,2,'omitnan');
% sub07.PFMP.ctl_param.force_R.raw = ctl{33,2};
sub07.PFMP.ctl_param.force_R.mean = mean(ctl{33,2},2,'omitnan');
sub07.PFMP.ctl_param.force_R.std = std(ctl{33,2},0,2,'omitnan');
% sub07.MSMP.ctl_param.force_R.raw = ctl{34,2};
sub07.MSMP.ctl_param.force_R.mean = mean(ctl{34,2},2,'omitnan');
sub07.MSMP.ctl_param.force_R.std = std(ctl{34,2},0,2,'omitnan');
% sub07.PSMP.ctl_param.force_R.raw = ctl{35,2};
sub07.PSMP.ctl_param.force_R.mean = mean(ctl{35,2},2,'omitnan');
sub07.PSMP.ctl_param.force_R.std = std(ctl{35,2},0,2,'omitnan');

sub01.MFMP.ctl_param.force_R.unit = 'N';
sub01.PFMP.ctl_param.force_R.unit = 'N';
sub01.MSMP.ctl_param.force_R.unit = 'N';
sub01.PSMP.ctl_param.force_R.unit = 'N';
sub02.MFMP.ctl_param.force_R.unit = 'N';
sub02.PFMP.ctl_param.force_R.unit = 'N';
sub02.MSMP.ctl_param.force_R.unit = 'N';
sub02.PSMP.ctl_param.force_R.unit = 'N';
sub03.MFMP.ctl_param.force_R.unit = 'N';
sub03.PFMP.ctl_param.force_R.unit = 'N';
sub03.MSMP.ctl_param.force_R.unit = 'N';
sub03.PSMP.ctl_param.force_R.unit = 'N';
sub04.MFMP.ctl_param.force_R.unit = 'N';
sub04.PFMP.ctl_param.force_R.unit = 'N';
sub04.MSMP.ctl_param.force_R.unit = 'N';
sub04.PSMP.ctl_param.force_R.unit = 'N';
sub05.MFMP.ctl_param.force_R.unit = 'N';
sub05.PFMP.ctl_param.force_R.unit = 'N';
sub05.MSMP.ctl_param.force_R.unit = 'N';
sub05.PSMP.ctl_param.force_R.unit = 'N';
sub06.MFMP.ctl_param.force_R.unit = 'N';
sub06.PFMP.ctl_param.force_R.unit = 'N';
sub06.MSMP.ctl_param.force_R.unit = 'N';
sub06.PSMP.ctl_param.force_R.unit = 'N';
sub07.MFMP.ctl_param.force_R.unit = 'N';
sub07.PFMP.ctl_param.force_R.unit = 'N';
sub07.MSMP.ctl_param.force_R.unit = 'N';
sub07.PSMP.ctl_param.force_R.unit = 'N';

% sub01.MFMP.ctl_param.force_L.raw = ctl{2,4};
sub01.MFMP.ctl_param.force_L.mean = mean(ctl{2,4},2,'omitnan');
sub01.MFMP.ctl_param.force_L.std = std(ctl{2,4},0,2,'omitnan');
% sub01.PFMP.ctl_param.force_L.raw = ctl{3,4};
sub01.PFMP.ctl_param.force_L.mean = mean(ctl{3,4},2,'omitnan');
sub01.PFMP.ctl_param.force_L.std = std(ctl{3,4},0,2,'omitnan');
% sub01.MSMP.ctl_param.force_L.raw = ctl{4,4};
sub01.MSMP.ctl_param.force_L.mean = mean(ctl{4,4},2,'omitnan');
sub01.MSMP.ctl_param.force_L.std = std(ctl{4,4},0,2,'omitnan');
% sub01.PSMP.ctl_param.force_L.raw = ctl{5,4};
sub01.PSMP.ctl_param.force_L.mean = mean(ctl{5,4},2,'omitnan');
sub01.PSMP.ctl_param.force_L.std = std(ctl{5,4},0,2,'omitnan');

% sub02.MFMP.ctl_param.force_L.raw = ctl{7,4};
sub02.MFMP.ctl_param.force_L.mean = mean(ctl{7,4},2,'omitnan');
sub02.MFMP.ctl_param.force_L.std = std(ctl{7,4},0,2,'omitnan');
% sub02.PFMP.ctl_param.force_L.raw = ctl{8,4};
sub02.PFMP.ctl_param.force_L.mean = mean(ctl{8,4},2,'omitnan');
sub02.PFMP.ctl_param.force_L.std = std(ctl{8,4},0,2,'omitnan');
% sub02.MSMP.ctl_param.force_L.raw = ctl{9,4};
sub02.MSMP.ctl_param.force_L.mean = mean(ctl{9,4},2,'omitnan');
sub02.MSMP.ctl_param.force_L.std = std(ctl{9,4},0,2,'omitnan');
% sub02.PSMP.ctl_param.force_L.raw = ctl{10,4};
sub02.PSMP.ctl_param.force_L.mean = mean(ctl{10,4},2,'omitnan');
sub02.PSMP.ctl_param.force_L.std = std(ctl{10,4},0,2,'omitnan');

% sub03.MFMP.ctl_param.force_L.raw = ctl{12,4};
sub03.MFMP.ctl_param.force_L.mean = mean(ctl{12,4},2,'omitnan');
sub03.MFMP.ctl_param.force_L.std = std(ctl{12,4},0,2,'omitnan');
% sub03.PFMP.ctl_param.force_L.raw = ctl{13,4};
sub03.PFMP.ctl_param.force_L.mean = mean(ctl{13,4},2,'omitnan');
sub03.PFMP.ctl_param.force_L.std = std(ctl{13,4},0,2,'omitnan');
% sub03.MSMP.ctl_param.force_L.raw = ctl{14,4};
sub03.MSMP.ctl_param.force_L.mean = mean(ctl{14,4},2,'omitnan');
sub03.MSMP.ctl_param.force_L.std = std(ctl{14,4},0,2,'omitnan');
% sub03.PSMP.ctl_param.force_L.raw = ctl{15,4};
sub03.PSMP.ctl_param.force_L.mean = mean(ctl{15,4},2,'omitnan');
sub03.PSMP.ctl_param.force_L.std = std(ctl{15,4},0,2,'omitnan');

% sub04.MFMP.ctl_param.force_L.raw = ctl{17,4};
sub04.MFMP.ctl_param.force_L.mean = mean(ctl{17,4},2,'omitnan');
sub04.MFMP.ctl_param.force_L.std = std(ctl{17,4},0,2,'omitnan');
% sub04.PFMP.ctl_param.force_L.raw = ctl{18,4};
sub04.PFMP.ctl_param.force_L.mean = mean(ctl{18,4},2,'omitnan');
sub04.PFMP.ctl_param.force_L.std = std(ctl{18,4},0,2,'omitnan');
% sub04.MSMP.ctl_param.force_L.raw = ctl{19,4};
sub04.MSMP.ctl_param.force_L.mean = mean(ctl{19,4},2,'omitnan');
sub04.MSMP.ctl_param.force_L.std = std(ctl{19,4},0,2,'omitnan');
% sub04.PSMP.ctl_param.force_L.raw = ctl{20,4};
sub04.PSMP.ctl_param.force_L.mean = mean(ctl{20,4},2,'omitnan');
sub04.PSMP.ctl_param.force_L.std = std(ctl{20,4},0,2,'omitnan');

% sub05.MFMP.ctl_param.force_L.raw = ctl{22,4};
sub05.MFMP.ctl_param.force_L.mean = mean(ctl{22,4},2,'omitnan');
sub05.MFMP.ctl_param.force_L.std = std(ctl{22,4},0,2,'omitnan');
% sub05.PFMP.ctl_param.force_L.raw = ctl{23,4};
sub05.PFMP.ctl_param.force_L.mean = mean(ctl{23,4},2,'omitnan');
sub05.PFMP.ctl_param.force_L.std = std(ctl{23,4},0,2,'omitnan');
% sub05.MSMP.ctl_param.force_L.raw = ctl{24,4};
sub05.MSMP.ctl_param.force_L.mean = mean(ctl{24,4},2,'omitnan');
sub05.MSMP.ctl_param.force_L.std = std(ctl{24,4},0,2,'omitnan');
% sub05.PSMP.ctl_param.force_L.raw = ctl{25,4};
sub05.PSMP.ctl_param.force_L.mean = mean(ctl{25,4},2,'omitnan');
sub05.PSMP.ctl_param.force_L.std = std(ctl{25,4},0,2,'omitnan');

% sub06.MFMP.ctl_param.force_L.raw = ctl{27,4};
sub06.MFMP.ctl_param.force_L.mean = mean(ctl{27,4},2,'omitnan');
sub06.MFMP.ctl_param.force_L.std = std(ctl{27,4},0,2,'omitnan');
% sub06.PFMP.ctl_param.force_L.raw = ctl{28,4};
sub06.PFMP.ctl_param.force_L.mean = mean(ctl{28,4},2,'omitnan');
sub06.PFMP.ctl_param.force_L.std = std(ctl{28,4},0,2,'omitnan');
% sub06.MSMP.ctl_param.force_L.raw = ctl{29,4};
sub06.MSMP.ctl_param.force_L.mean = mean(ctl{29,4},2,'omitnan');
sub06.MSMP.ctl_param.force_L.std = std(ctl{29,4},0,2,'omitnan');
% sub06.PSMP.ctl_param.force_L.raw = ctl{30,4};
sub06.PSMP.ctl_param.force_L.mean = mean(ctl{30,4},2,'omitnan');
sub06.PSMP.ctl_param.force_L.std = std(ctl{30,4},0,2,'omitnan');

% sub07.MFMP.ctl_param.force_L.raw = ctl{32,4};
sub07.MFMP.ctl_param.force_L.mean = mean(ctl{32,4},2,'omitnan');
sub07.MFMP.ctl_param.force_L.std = std(ctl{32,4},0,2,'omitnan');
% sub07.PFMP.ctl_param.force_L.raw = ctl{33,4};
sub07.PFMP.ctl_param.force_L.mean = mean(ctl{33,4},2,'omitnan');
sub07.PFMP.ctl_param.force_L.std = std(ctl{33,4},0,2,'omitnan');
% sub07.MSMP.ctl_param.force_L.raw = ctl{34,4};
sub07.MSMP.ctl_param.force_L.mean = mean(ctl{34,4},2,'omitnan');
sub07.MSMP.ctl_param.force_L.std = std(ctl{34,4},0,2,'omitnan');
% sub07.PSMP.ctl_param.force_L.raw = ctl{35,4};
sub07.PSMP.ctl_param.force_L.mean = mean(ctl{35,4},2,'omitnan');
sub07.PSMP.ctl_param.force_L.std = std(ctl{35,4},0,2,'omitnan');

sub01.MFMP.ctl_param.force_L.unit = 'N';
sub01.PFMP.ctl_param.force_L.unit = 'N';
sub01.MSMP.ctl_param.force_L.unit = 'N';
sub01.PSMP.ctl_param.force_L.unit = 'N';
sub02.MFMP.ctl_param.force_L.unit = 'N';
sub02.PFMP.ctl_param.force_L.unit = 'N';
sub02.MSMP.ctl_param.force_L.unit = 'N';
sub02.PSMP.ctl_param.force_L.unit = 'N';
sub03.MFMP.ctl_param.force_L.unit = 'N';
sub03.PFMP.ctl_param.force_L.unit = 'N';
sub03.MSMP.ctl_param.force_L.unit = 'N';
sub03.PSMP.ctl_param.force_L.unit = 'N';
sub04.MFMP.ctl_param.force_L.unit = 'N';
sub04.PFMP.ctl_param.force_L.unit = 'N';
sub04.MSMP.ctl_param.force_L.unit = 'N';
sub04.PSMP.ctl_param.force_L.unit = 'N';
sub05.MFMP.ctl_param.force_L.unit = 'N';
sub05.PFMP.ctl_param.force_L.unit = 'N';
sub05.MSMP.ctl_param.force_L.unit = 'N';
sub05.PSMP.ctl_param.force_L.unit = 'N';
sub06.MFMP.ctl_param.force_L.unit = 'N';
sub06.PFMP.ctl_param.force_L.unit = 'N';
sub06.MSMP.ctl_param.force_L.unit = 'N';
sub06.PSMP.ctl_param.force_L.unit = 'N';
sub07.MFMP.ctl_param.force_L.unit = 'N';
sub07.PFMP.ctl_param.force_L.unit = 'N';
sub07.MSMP.ctl_param.force_L.unit = 'N';
sub07.PSMP.ctl_param.force_L.unit = 'N';

%% moment
% sub01.MFMP.ctl_param.moment_r.raw = ctl{2,5};
sub01.MFMP.ctl_param.moment_r.mean = mean(ctl{2,5},2,'omitnan');
sub01.MFMP.ctl_param.moment_r.std = std(ctl{2,5},0,2,'omitnan');
% sub01.PFMP.ctl_param.moment_r.raw = ctl{3,5};
sub01.PFMP.ctl_param.moment_r.mean = mean(ctl{3,5},2,'omitnan');
sub01.PFMP.ctl_param.moment_r.std = std(ctl{3,5},0,2,'omitnan');
% sub01.MSMP.ctl_param.moment_r.raw = ctl{4,5};
sub01.MSMP.ctl_param.moment_r.mean = mean(ctl{4,5},2,'omitnan');
sub01.MSMP.ctl_param.moment_r.std = std(ctl{4,5},0,2,'omitnan');
% sub01.PSMP.ctl_param.moment_r.raw = ctl{5,5};
sub01.PSMP.ctl_param.moment_r.mean = mean(ctl{5,5},2,'omitnan');
sub01.PSMP.ctl_param.moment_r.std = std(ctl{5,5},0,2,'omitnan');

% sub02.MFMP.ctl_param.moment_r.raw = ctl{7,5};
sub02.MFMP.ctl_param.moment_r.mean = mean(ctl{7,5},2,'omitnan');
sub02.MFMP.ctl_param.moment_r.std = std(ctl{7,5},0,2,'omitnan');
% sub02.PFMP.ctl_param.moment_r.raw = ctl{8,5};
sub02.PFMP.ctl_param.moment_r.mean = mean(ctl{8,5},2,'omitnan');
sub02.PFMP.ctl_param.moment_r.std = std(ctl{8,5},0,2,'omitnan');
% sub02.MSMP.ctl_param.moment_r.raw = ctl{9,5};
sub02.MSMP.ctl_param.moment_r.mean = mean(ctl{9,5},2,'omitnan');
sub02.MSMP.ctl_param.moment_r.std = std(ctl{9,5},0,2,'omitnan');
% sub02.PSMP.ctl_param.moment_r.raw = ctl{10,5};
sub02.PSMP.ctl_param.moment_r.mean = mean(ctl{10,5},2,'omitnan');
sub02.PSMP.ctl_param.moment_r.std = std(ctl{10,5},0,2,'omitnan');

% sub03.MFMP.ctl_param.moment_r.raw = ctl{12,5};
sub03.MFMP.ctl_param.moment_r.mean = mean(ctl{12,5},2,'omitnan');
sub03.MFMP.ctl_param.moment_r.std = std(ctl{12,5},0,2,'omitnan');
% sub03.PFMP.ctl_param.moment_r.raw = ctl{13,5};
sub03.PFMP.ctl_param.moment_r.mean = mean(ctl{13,5},2,'omitnan');
sub03.PFMP.ctl_param.moment_r.std = std(ctl{13,5},0,2,'omitnan');
% sub03.MSMP.ctl_param.moment_r.raw = ctl{14,5};
sub03.MSMP.ctl_param.moment_r.mean = mean(ctl{14,5},2,'omitnan');
sub03.MSMP.ctl_param.moment_r.std = std(ctl{14,5},0,2,'omitnan');
% sub03.PSMP.ctl_param.moment_r.raw = ctl{15,5};
sub03.PSMP.ctl_param.moment_r.mean = mean(ctl{15,5},2,'omitnan');
sub03.PSMP.ctl_param.moment_r.std = std(ctl{15,5},0,2,'omitnan');

% sub04.MFMP.ctl_param.moment_r.raw = ctl{17,5};
sub04.MFMP.ctl_param.moment_r.mean = mean(ctl{17,5},2,'omitnan');
sub04.MFMP.ctl_param.moment_r.std = std(ctl{17,5},0,2,'omitnan');
% sub04.PFMP.ctl_param.moment_r.raw = ctl{18,5};
sub04.PFMP.ctl_param.moment_r.mean = mean(ctl{18,5},2,'omitnan');
sub04.PFMP.ctl_param.moment_r.std = std(ctl{18,5},0,2,'omitnan');
% sub04.MSMP.ctl_param.moment_r.raw = ctl{19,5};
sub04.MSMP.ctl_param.moment_r.mean = mean(ctl{19,5},2,'omitnan');
sub04.MSMP.ctl_param.moment_r.std = std(ctl{19,5},0,2,'omitnan');
% sub04.PSMP.ctl_param.moment_r.raw = ctl{20,5};
sub04.PSMP.ctl_param.moment_r.mean = mean(ctl{20,5},2,'omitnan');
sub04.PSMP.ctl_param.moment_r.std = std(ctl{20,5},0,2,'omitnan');

% sub05.MFMP.ctl_param.moment_r.raw = ctl{22,5};
sub05.MFMP.ctl_param.moment_r.mean = mean(ctl{22,5},2,'omitnan');
sub05.MFMP.ctl_param.moment_r.std = std(ctl{22,5},0,2,'omitnan');
% sub05.PFMP.ctl_param.moment_r.raw = ctl{23,5};
sub05.PFMP.ctl_param.moment_r.mean = mean(ctl{23,5},2,'omitnan');
sub05.PFMP.ctl_param.moment_r.std = std(ctl{23,5},0,2,'omitnan');
% sub05.MSMP.ctl_param.moment_r.raw = ctl{24,5};
sub05.MSMP.ctl_param.moment_r.mean = mean(ctl{24,5},2,'omitnan');
sub05.MSMP.ctl_param.moment_r.std = std(ctl{24,5},0,2,'omitnan');
% sub05.PSMP.ctl_param.moment_r.raw = ctl{25,5};
sub05.PSMP.ctl_param.moment_r.mean = mean(ctl{25,5},2,'omitnan');
sub05.PSMP.ctl_param.moment_r.std = std(ctl{25,5},0,2,'omitnan');

% sub06.MFMP.ctl_param.moment_r.raw = ctl{27,5};
sub06.MFMP.ctl_param.moment_r.mean = mean(ctl{27,5},2,'omitnan');
sub06.MFMP.ctl_param.moment_r.std = std(ctl{27,5},0,2,'omitnan');
% sub06.PFMP.ctl_param.moment_r.raw = ctl{28,5};
sub06.PFMP.ctl_param.moment_r.mean = mean(ctl{28,5},2,'omitnan');
sub06.PFMP.ctl_param.moment_r.std = std(ctl{28,5},0,2,'omitnan');
% sub06.MSMP.ctl_param.moment_r.raw = ctl{29,5};
sub06.MSMP.ctl_param.moment_r.mean = mean(ctl{29,5},2,'omitnan');
sub06.MSMP.ctl_param.moment_r.std = std(ctl{29,5},0,2,'omitnan');
% sub06.PSMP.ctl_param.moment_r.raw = ctl{30,5};
sub06.PSMP.ctl_param.moment_r.mean = mean(ctl{30,5},2,'omitnan');
sub06.PSMP.ctl_param.moment_r.std = std(ctl{30,5},0,2,'omitnan');

% sub07.MFMP.ctl_param.moment_r.raw = ctl{32,5};
sub07.MFMP.ctl_param.moment_r.mean = mean(ctl{32,5},2,'omitnan');
sub07.MFMP.ctl_param.moment_r.std = std(ctl{32,5},0,2,'omitnan');
% sub07.PFMP.ctl_param.moment_r.raw = ctl{33,5};
sub07.PFMP.ctl_param.moment_r.mean = mean(ctl{33,5},2,'omitnan');
sub07.PFMP.ctl_param.moment_r.std = std(ctl{33,5},0,2,'omitnan');
% sub07.MSMP.ctl_param.moment_r.raw = ctl{34,5};
sub07.MSMP.ctl_param.moment_r.mean = mean(ctl{34,5},2,'omitnan');
sub07.MSMP.ctl_param.moment_r.std = std(ctl{34,5},0,2,'omitnan');
% sub07.PSMP.ctl_param.moment_r.raw = ctl{35,5};
sub07.PSMP.ctl_param.moment_r.mean = mean(ctl{35,5},2,'omitnan');
sub07.PSMP.ctl_param.moment_r.std = std(ctl{35,5},0,2,'omitnan');

sub01.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub01.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub01.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub01.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub02.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub02.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub02.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub02.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub03.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub03.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub03.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub03.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub04.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub04.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub04.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub04.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub05.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub05.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub05.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub05.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub06.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub06.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub06.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub06.PSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub07.MFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub07.PFMP.ctl_param.moment_r.unit = 'Nm/kg';
sub07.MSMP.ctl_param.moment_r.unit = 'Nm/kg';
sub07.PSMP.ctl_param.moment_r.unit = 'Nm/kg';


% sub01.MFMP.ctl_param.moment_l.raw = ctl{2,14};
sub01.MFMP.ctl_param.moment_l.mean = mean(ctl{2,14},2,'omitnan');
sub01.MFMP.ctl_param.moment_l.std = std(ctl{2,14},0,2,'omitnan');
% sub01.PFMP.ctl_param.moment_l.raw = ctl{3,14};
sub01.PFMP.ctl_param.moment_l.mean = mean(ctl{3,14},2,'omitnan');
sub01.PFMP.ctl_param.moment_l.std = std(ctl{3,14},0,2,'omitnan');
% sub01.MSMP.ctl_param.moment_l.raw = ctl{4,14};
sub01.MSMP.ctl_param.moment_l.mean = mean(ctl{4,14},2,'omitnan');
sub01.MSMP.ctl_param.moment_l.std = std(ctl{4,14},0,2,'omitnan');
% sub01.PSMP.ctl_param.moment_l.raw = ctl{5,14};
sub01.PSMP.ctl_param.moment_l.mean = mean(ctl{5,14},2,'omitnan');
sub01.PSMP.ctl_param.moment_l.std = std(ctl{5,14},0,2,'omitnan');

sub02.MFMP.ctl_param.moment_l.raw = ctl{7,14};
sub02.MFMP.ctl_param.moment_l.mean = mean(ctl{7,14},2,'omitnan');
sub02.MFMP.ctl_param.moment_l.std = std(ctl{7,14},0,2,'omitnan');
sub02.PFMP.ctl_param.moment_l.raw = ctl{8,14};
sub02.PFMP.ctl_param.moment_l.mean = mean(ctl{8,14},2,'omitnan');
sub02.PFMP.ctl_param.moment_l.std = std(ctl{8,14},0,2,'omitnan');
sub02.MSMP.ctl_param.moment_l.raw = ctl{9,14};
sub02.MSMP.ctl_param.moment_l.mean = mean(ctl{9,14},2,'omitnan');
sub02.MSMP.ctl_param.moment_l.std = std(ctl{9,14},0,2,'omitnan');
sub02.PSMP.ctl_param.moment_l.raw = ctl{10,14};
sub02.PSMP.ctl_param.moment_l.mean = mean(ctl{10,14},2,'omitnan');
sub02.PSMP.ctl_param.moment_l.std = std(ctl{10,14},0,2,'omitnan');

% sub03.MFMP.ctl_param.moment_l.raw = ctl{12,14};
sub03.MFMP.ctl_param.moment_l.mean = mean(ctl{12,14},2,'omitnan');
sub03.MFMP.ctl_param.moment_l.std = std(ctl{12,14},0,2,'omitnan');
% sub03.PFMP.ctl_param.moment_l.raw = ctl{13,14};
sub03.PFMP.ctl_param.moment_l.mean = mean(ctl{13,14},2,'omitnan');
sub03.PFMP.ctl_param.moment_l.std = std(ctl{13,14},0,2,'omitnan');
% sub03.MSMP.ctl_param.moment_l.raw = ctl{14,14};
sub03.MSMP.ctl_param.moment_l.mean = mean(ctl{14,14},2,'omitnan');
sub03.MSMP.ctl_param.moment_l.std = std(ctl{14,14},0,2,'omitnan');
% sub03.PSMP.ctl_param.moment_l.raw = ctl{15,14};
sub03.PSMP.ctl_param.moment_l.mean = mean(ctl{15,14},2,'omitnan');
sub03.PSMP.ctl_param.moment_l.std = std(ctl{15,14},0,2,'omitnan');

% sub04.MFMP.ctl_param.moment_l.raw = ctl{17,14};
sub04.MFMP.ctl_param.moment_l.mean = mean(ctl{17,14},2,'omitnan');
sub04.MFMP.ctl_param.moment_l.std = std(ctl{17,14},0,2,'omitnan');
% sub04.PFMP.ctl_param.moment_l.raw = ctl{18,14};
sub04.PFMP.ctl_param.moment_l.mean = mean(ctl{18,14},2,'omitnan');
sub04.PFMP.ctl_param.moment_l.std = std(ctl{18,14},0,2,'omitnan');
% sub04.MSMP.ctl_param.moment_l.raw = ctl{19,14};
sub04.MSMP.ctl_param.moment_l.mean = mean(ctl{19,14},2,'omitnan');
sub04.MSMP.ctl_param.moment_l.std = std(ctl{19,14},0,2,'omitnan');
% sub04.PSMP.ctl_param.moment_l.raw = ctl{20,14};
sub04.PSMP.ctl_param.moment_l.mean = mean(ctl{20,14},2,'omitnan');
sub04.PSMP.ctl_param.moment_l.std = std(ctl{20,14},0,2,'omitnan');

% sub05.MFMP.ctl_param.moment_l.raw = ctl{22,14};
sub05.MFMP.ctl_param.moment_l.mean = mean(ctl{22,14},2,'omitnan');
sub05.MFMP.ctl_param.moment_l.std = std(ctl{22,14},0,2,'omitnan');
% sub05.PFMP.ctl_param.moment_l.raw = ctl{23,14};
sub05.PFMP.ctl_param.moment_l.mean = mean(ctl{23,14},2,'omitnan');
sub05.PFMP.ctl_param.moment_l.std = std(ctl{23,14},0,2,'omitnan');
% sub05.MSMP.ctl_param.moment_l.raw = ctl{24,14};
sub05.MSMP.ctl_param.moment_l.mean = mean(ctl{24,14},2,'omitnan');
sub05.MSMP.ctl_param.moment_l.std = std(ctl{24,14},0,2,'omitnan');
% sub05.PSMP.ctl_param.moment_l.raw = ctl{25,14};
sub05.PSMP.ctl_param.moment_l.mean = mean(ctl{25,14},2,'omitnan');
sub05.PSMP.ctl_param.moment_l.std = std(ctl{25,14},0,2,'omitnan');

% sub06.MFMP.ctl_param.moment_l.raw = ctl{27,14};
sub06.MFMP.ctl_param.moment_l.mean = mean(ctl{27,14},2,'omitnan');
sub06.MFMP.ctl_param.moment_l.std = std(ctl{27,14},0,2,'omitnan');
% sub06.PFMP.ctl_param.moment_l.raw = ctl{28,14};
sub06.PFMP.ctl_param.moment_l.mean = mean(ctl{28,14},2,'omitnan');
sub06.PFMP.ctl_param.moment_l.std = std(ctl{28,14},0,2,'omitnan');
% sub06.MSMP.ctl_param.moment_l.raw = ctl{29,14};
sub06.MSMP.ctl_param.moment_l.mean = mean(ctl{29,14},2,'omitnan');
sub06.MSMP.ctl_param.moment_l.std = std(ctl{29,14},0,2,'omitnan');
% sub06.PSMP.ctl_param.moment_l.raw = ctl{30,14};
sub06.PSMP.ctl_param.moment_l.mean = mean(ctl{30,14},2,'omitnan');
sub06.PSMP.ctl_param.moment_l.std = std(ctl{30,14},0,2,'omitnan');

% sub07.MFMP.ctl_param.moment_l.raw = ctl{32,14};
sub07.MFMP.ctl_param.moment_l.mean = mean(ctl{32,14},2,'omitnan');
sub07.MFMP.ctl_param.moment_l.std = std(ctl{32,14},0,2,'omitnan');
% sub07.PFMP.ctl_param.moment_l.raw = ctl{33,14};
sub07.PFMP.ctl_param.moment_l.mean = mean(ctl{33,14},2,'omitnan');
sub07.PFMP.ctl_param.moment_l.std = std(ctl{33,14},0,2,'omitnan');
% sub07.MSMP.ctl_param.moment_l.raw = ctl{34,14};
sub07.MSMP.ctl_param.moment_l.mean = mean(ctl{34,14},2,'omitnan');
sub07.MSMP.ctl_param.moment_l.std = std(ctl{34,14},0,2,'omitnan');
% sub07.PSMP.ctl_param.moment_l.raw = ctl{35,14};
sub07.PSMP.ctl_param.moment_l.mean = mean(ctl{35,14},2,'omitnan');
sub07.PSMP.ctl_param.moment_l.std = std(ctl{35,14},0,2,'omitnan');

sub01.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub01.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub01.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub01.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub02.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub02.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub02.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub02.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub03.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub03.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub03.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub03.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub04.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub04.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub04.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub04.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub05.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub05.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub05.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub05.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub06.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub06.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub06.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub06.PSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub07.MFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub07.PFMP.ctl_param.moment_l.unit = 'Nm/kg';
sub07.MSMP.ctl_param.moment_l.unit = 'Nm/kg';
sub07.PSMP.ctl_param.moment_l.unit = 'Nm/kg';

%% thigh angvel

% sub01.MFMP.ctl_param.angvel_t.raw = ctl{2,7};
sub01.MFMP.ctl_param.angvel_t.mean = mean(ctl{2,7},2,'omitnan');
sub01.MFMP.ctl_param.angvel_t.std = std(ctl{2,7},0,2,'omitnan');
% sub01.PFMP.ctl_param.angvel_t.raw = ctl{3,7};
sub01.PFMP.ctl_param.angvel_t.mean = mean(ctl{3,7},2,'omitnan');
sub01.PFMP.ctl_param.angvel_t.std = std(ctl{3,7},0,2,'omitnan');
% sub01.MSMP.ctl_param.angvel_t.raw = ctl{4,7};
sub01.MSMP.ctl_param.angvel_t.mean = mean(ctl{4,7},2,'omitnan');
sub01.MSMP.ctl_param.angvel_t.std = std(ctl{4,7},0,2,'omitnan');
% sub01.PSMP.ctl_param.angvel_t.raw = ctl{5,7};
sub01.PSMP.ctl_param.angvel_t.mean = mean(ctl{5,7},2,'omitnan');
sub01.PSMP.ctl_param.angvel_t.std = std(ctl{5,7},0,2,'omitnan');

% sub02.MFMP.ctl_param.angvel_t.raw = ctl{7,7};
sub02.MFMP.ctl_param.angvel_t.mean = mean(ctl{7,7},2,'omitnan');
sub02.MFMP.ctl_param.angvel_t.std = std(ctl{7,7},0,2,'omitnan');
% sub02.PFMP.ctl_param.angvel_t.raw = ctl{8,7};
sub02.PFMP.ctl_param.angvel_t.mean = mean(ctl{8,7},2,'omitnan');
sub02.PFMP.ctl_param.angvel_t.std = std(ctl{8,7},0,2,'omitnan');
% sub02.MSMP.ctl_param.angvel_t.raw = ctl{9,7};
sub02.MSMP.ctl_param.angvel_t.mean = mean(ctl{9,7},2,'omitnan');
sub02.MSMP.ctl_param.angvel_t.std = std(ctl{9,7},0,2,'omitnan');
% sub02.PSMP.ctl_param.angvel_t.raw = ctl{10,7};
sub02.PSMP.ctl_param.angvel_t.mean = mean(ctl{10,7},2,'omitnan');
sub02.PSMP.ctl_param.angvel_t.std = std(ctl{10,7},0,2,'omitnan');

% sub03.MFMP.ctl_param.angvel_t.raw = ctl{12,7};
sub03.MFMP.ctl_param.angvel_t.mean = mean(ctl{12,7},2,'omitnan');
sub03.MFMP.ctl_param.angvel_t.std = std(ctl{12,7},0,2,'omitnan');
% sub03.PFMP.ctl_param.angvel_t.raw = ctl{13,7};
sub03.PFMP.ctl_param.angvel_t.mean = mean(ctl{13,7},2,'omitnan');
sub03.PFMP.ctl_param.angvel_t.std = std(ctl{13,7},0,2,'omitnan');
% sub03.MSMP.ctl_param.angvel_t.raw = ctl{14,7};
sub03.MSMP.ctl_param.angvel_t.mean = mean(ctl{14,7},2,'omitnan');
sub03.MSMP.ctl_param.angvel_t.std = std(ctl{14,7},0,2,'omitnan');
% sub03.PSMP.ctl_param.angvel_t.raw = ctl{15,7};
sub03.PSMP.ctl_param.angvel_t.mean = mean(ctl{15,7},2,'omitnan');
sub03.PSMP.ctl_param.angvel_t.std = std(ctl{15,7},0,2,'omitnan');

% sub04.MFMP.ctl_param.angvel_t.raw = ctl{17,7};
sub04.MFMP.ctl_param.angvel_t.mean = mean(ctl{17,7},2,'omitnan');
sub04.MFMP.ctl_param.angvel_t.std = std(ctl{17,7},0,2,'omitnan');
% sub04.PFMP.ctl_param.angvel_t.raw = ctl{18,7};
sub04.PFMP.ctl_param.angvel_t.mean = mean(ctl{18,7},2,'omitnan');
sub04.PFMP.ctl_param.angvel_t.std = std(ctl{18,7},0,2,'omitnan');
% sub04.MSMP.ctl_param.angvel_t.raw = ctl{19,7};
sub04.MSMP.ctl_param.angvel_t.mean = mean(ctl{19,7},2,'omitnan');
sub04.MSMP.ctl_param.angvel_t.std = std(ctl{19,7},0,2,'omitnan');
% sub04.PSMP.ctl_param.angvel_t.raw = ctl{20,7};
sub04.PSMP.ctl_param.angvel_t.mean = mean(ctl{20,7},2,'omitnan');
sub04.PSMP.ctl_param.angvel_t.std = std(ctl{20,7},0,2,'omitnan');

% sub05.MFMP.ctl_param.angvel_t.raw = ctl{22,7};
sub05.MFMP.ctl_param.angvel_t.mean = mean(ctl{22,7},2,'omitnan');
sub05.MFMP.ctl_param.angvel_t.std = std(ctl{22,7},0,2,'omitnan');
% sub05.PFMP.ctl_param.angvel_t.raw = ctl{23,7};
sub05.PFMP.ctl_param.angvel_t.mean = mean(ctl{23,7},2,'omitnan');
sub05.PFMP.ctl_param.angvel_t.std = std(ctl{23,7},0,2,'omitnan');
% sub05.MSMP.ctl_param.angvel_t.raw = ctl{24,7};
sub05.MSMP.ctl_param.angvel_t.mean = mean(ctl{24,7},2,'omitnan');
sub05.MSMP.ctl_param.angvel_t.std = std(ctl{24,7},0,2,'omitnan');
% sub05.PSMP.ctl_param.angvel_t.raw = ctl{25,7};
sub05.PSMP.ctl_param.angvel_t.mean = mean(ctl{25,7},2,'omitnan');
sub05.PSMP.ctl_param.angvel_t.std = std(ctl{25,7},0,2,'omitnan');

% sub06.MFMP.ctl_param.angvel_t.raw = ctl{27,7};
sub06.MFMP.ctl_param.angvel_t.mean = mean(ctl{27,7},2,'omitnan');
sub06.MFMP.ctl_param.angvel_t.std = std(ctl{27,7},0,2,'omitnan');
% sub06.PFMP.ctl_param.angvel_t.raw = ctl{28,7};
sub06.PFMP.ctl_param.angvel_t.mean = mean(ctl{28,7},2,'omitnan');
sub06.PFMP.ctl_param.angvel_t.std = std(ctl{28,7},0,2,'omitnan');
% sub06.MSMP.ctl_param.angvel_t.raw = ctl{29,7};
sub06.MSMP.ctl_param.angvel_t.mean = mean(ctl{29,7},2,'omitnan');
sub06.MSMP.ctl_param.angvel_t.std = std(ctl{29,7},0,2,'omitnan');
% sub06.PSMP.ctl_param.angvel_t.raw = ctl{30,7};
sub06.PSMP.ctl_param.angvel_t.mean = mean(ctl{30,7},2,'omitnan');
sub06.PSMP.ctl_param.angvel_t.std = std(ctl{30,7},0,2,'omitnan');

% sub07.MFMP.ctl_param.angvel_t.raw = ctl{32,7};
sub07.MFMP.ctl_param.angvel_t.mean = mean(ctl{32,7},2,'omitnan');
sub07.MFMP.ctl_param.angvel_t.std = std(ctl{32,7},0,2,'omitnan');
% sub07.PFMP.ctl_param.angvel_t.raw = ctl{33,7};
sub07.PFMP.ctl_param.angvel_t.mean = mean(ctl{33,7},2,'omitnan');
sub07.PFMP.ctl_param.angvel_t.std = std(ctl{33,7},0,2,'omitnan');
% sub07.MSMP.ctl_param.angvel_t.raw = ctl{34,7};
sub07.MSMP.ctl_param.angvel_t.mean = mean(ctl{34,7},2,'omitnan');
sub07.MSMP.ctl_param.angvel_t.std = std(ctl{34,7},0,2,'omitnan');
% sub07.PSMP.ctl_param.angvel_t.raw = ctl{35,7};
sub07.PSMP.ctl_param.angvel_t.mean = mean(ctl{35,7},2,'omitnan');
sub07.PSMP.ctl_param.angvel_t.std = std(ctl{35,7},0,2,'omitnan');

sub01.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub01.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub01.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub01.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub02.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub02.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub02.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub02.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub03.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub03.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub03.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub03.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub04.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub04.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub04.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub04.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub05.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub05.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub05.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub05.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub06.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub06.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub06.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub06.PSMP.ctl_param.angvel_t.unit = 'rad/s';
sub07.MFMP.ctl_param.angvel_t.unit = 'rad/s';
sub07.PFMP.ctl_param.angvel_t.unit = 'rad/s';
sub07.MSMP.ctl_param.angvel_t.unit = 'rad/s';
sub07.PSMP.ctl_param.angvel_t.unit = 'rad/s';

%% foot angvel

% sub01.MFMP.ctl_param.angvel_r.raw = ctl{2,8};
sub01.MFMP.ctl_param.angvel_r.mean = mean(ctl{2,8},2,'omitnan');
sub01.MFMP.ctl_param.angvel_r.std = std(ctl{2,8},0,2,'omitnan');
% sub01.PFMP.ctl_param.angvel_r.raw = ctl{3,8};
sub01.PFMP.ctl_param.angvel_r.mean = mean(ctl{3,8},2,'omitnan');
sub01.PFMP.ctl_param.angvel_r.std = std(ctl{3,8},0,2,'omitnan');
% sub01.MSMP.ctl_param.angvel_r.raw = ctl{4,8};
sub01.MSMP.ctl_param.angvel_r.mean = mean(ctl{4,8},2,'omitnan');
sub01.MSMP.ctl_param.angvel_r.std = std(ctl{4,8},0,2,'omitnan');
% sub01.PSMP.ctl_param.angvel_r.raw = ctl{5,8};
sub01.PSMP.ctl_param.angvel_r.mean = mean(ctl{5,8},2,'omitnan');
sub01.PSMP.ctl_param.angvel_r.std = std(ctl{5,8},0,2,'omitnan');

% sub02.MFMP.ctl_param.angvel_r.raw = ctl{7,8};
sub02.MFMP.ctl_param.angvel_r.mean = mean(ctl{7,8},2,'omitnan');
sub02.MFMP.ctl_param.angvel_r.std = std(ctl{7,8},0,2,'omitnan');
% sub02.PFMP.ctl_param.angvel_r.raw = ctl{8,8};
sub02.PFMP.ctl_param.angvel_r.mean = mean(ctl{8,8},2,'omitnan');
sub02.PFMP.ctl_param.angvel_r.std = std(ctl{8,8},0,2,'omitnan');
% sub02.MSMP.ctl_param.angvel_r.raw = ctl{9,8};
sub02.MSMP.ctl_param.angvel_r.mean = mean(ctl{9,8},2,'omitnan');
sub02.MSMP.ctl_param.angvel_r.std = std(ctl{9,8},0,2,'omitnan');
% sub02.PSMP.ctl_param.angvel_r.raw = ctl{10,8};
sub02.PSMP.ctl_param.angvel_r.mean = mean(ctl{10,8},2,'omitnan');
sub02.PSMP.ctl_param.angvel_r.std = std(ctl{10,8},0,2,'omitnan');

% sub03.MFMP.ctl_param.angvel_r.raw = ctl{12,8};
sub03.MFMP.ctl_param.angvel_r.mean = mean(ctl{12,8},2,'omitnan');
sub03.MFMP.ctl_param.angvel_r.std = std(ctl{12,8},0,2,'omitnan');
% sub03.PFMP.ctl_param.angvel_r.raw = ctl{13,8};
sub03.PFMP.ctl_param.angvel_r.mean = mean(ctl{13,8},2,'omitnan');
sub03.PFMP.ctl_param.angvel_r.std = std(ctl{13,8},0,2,'omitnan');
% sub03.MSMP.ctl_param.angvel_r.raw = ctl{14,8};
sub03.MSMP.ctl_param.angvel_r.mean = mean(ctl{14,8},2,'omitnan');
sub03.MSMP.ctl_param.angvel_r.std = std(ctl{14,8},0,2,'omitnan');
% sub03.PSMP.ctl_param.angvel_r.raw = ctl{15,8};
sub03.PSMP.ctl_param.angvel_r.mean = mean(ctl{15,8},2,'omitnan');
sub03.PSMP.ctl_param.angvel_r.std = std(ctl{15,8},0,2,'omitnan');

% sub04.MFMP.ctl_param.angvel_r.raw = ctl{17,8};
sub04.MFMP.ctl_param.angvel_r.mean = mean(ctl{17,8},2,'omitnan');
sub04.MFMP.ctl_param.angvel_r.std = std(ctl{17,8},0,2,'omitnan');
% sub04.PFMP.ctl_param.angvel_r.raw = ctl{18,8};
sub04.PFMP.ctl_param.angvel_r.mean = mean(ctl{18,8},2,'omitnan');
sub04.PFMP.ctl_param.angvel_r.std = std(ctl{18,8},0,2,'omitnan');
% sub04.MSMP.ctl_param.angvel_r.raw = ctl{19,8};
sub04.MSMP.ctl_param.angvel_r.mean = mean(ctl{19,8},2,'omitnan');
sub04.MSMP.ctl_param.angvel_r.std = std(ctl{19,8},0,2,'omitnan');
% sub04.PSMP.ctl_param.angvel_r.raw = ctl{20,8};
sub04.PSMP.ctl_param.angvel_r.mean = mean(ctl{20,8},2,'omitnan');
sub04.PSMP.ctl_param.angvel_r.std = std(ctl{20,8},0,2,'omitnan');

% sub05.MFMP.ctl_param.angvel_r.raw = ctl{22,8};
sub05.MFMP.ctl_param.angvel_r.mean = mean(ctl{22,8},2,'omitnan');
sub05.MFMP.ctl_param.angvel_r.std = std(ctl{22,8},0,2,'omitnan');
% sub05.PFMP.ctl_param.angvel_r.raw = ctl{23,8};
sub05.PFMP.ctl_param.angvel_r.mean = mean(ctl{23,8},2,'omitnan');
sub05.PFMP.ctl_param.angvel_r.std = std(ctl{23,8},0,2,'omitnan');
% sub05.MSMP.ctl_param.angvel_r.raw = ctl{24,8};
sub05.MSMP.ctl_param.angvel_r.mean = mean(ctl{24,8},2,'omitnan');
sub05.MSMP.ctl_param.angvel_r.std = std(ctl{24,8},0,2,'omitnan');
% sub05.PSMP.ctl_param.angvel_r.raw = ctl{25,8};
sub05.PSMP.ctl_param.angvel_r.mean = mean(ctl{25,8},2,'omitnan');
sub05.PSMP.ctl_param.angvel_r.std = std(ctl{25,8},0,2,'omitnan');

% sub06.MFMP.ctl_param.angvel_r.raw = ctl{27,8};
sub06.MFMP.ctl_param.angvel_r.mean = mean(ctl{27,8},2,'omitnan');
sub06.MFMP.ctl_param.angvel_r.std = std(ctl{27,8},0,2,'omitnan');
% sub06.PFMP.ctl_param.angvel_r.raw = ctl{28,8};
sub06.PFMP.ctl_param.angvel_r.mean = mean(ctl{28,8},2,'omitnan');
sub06.PFMP.ctl_param.angvel_r.std = std(ctl{28,8},0,2,'omitnan');
% sub06.MSMP.ctl_param.angvel_r.raw = ctl{29,8};
sub06.MSMP.ctl_param.angvel_r.mean = mean(ctl{29,8},2,'omitnan');
sub06.MSMP.ctl_param.angvel_r.std = std(ctl{29,8},0,2,'omitnan');
% sub06.PSMP.ctl_param.angvel_r.raw = ctl{30,8};
sub06.PSMP.ctl_param.angvel_r.mean = mean(ctl{30,8},2,'omitnan');
sub06.PSMP.ctl_param.angvel_r.std = std(ctl{30,8},0,2,'omitnan');

% sub07.MFMP.ctl_param.angvel_r.raw = ctl{32,8};
sub07.MFMP.ctl_param.angvel_r.mean = mean(ctl{32,8},2,'omitnan');
sub07.MFMP.ctl_param.angvel_r.std = std(ctl{32,8},0,2,'omitnan');
% sub07.PFMP.ctl_param.angvel_r.raw = ctl{33,8};
sub07.PFMP.ctl_param.angvel_r.mean = mean(ctl{33,8},2,'omitnan');
sub07.PFMP.ctl_param.angvel_r.std = std(ctl{33,8},0,2,'omitnan');
% sub07.MSMP.ctl_param.angvel_r.raw = ctl{34,8};
sub07.MSMP.ctl_param.angvel_r.mean = mean(ctl{34,8},2,'omitnan');
sub07.MSMP.ctl_param.angvel_r.std = std(ctl{34,8},0,2,'omitnan');
% sub07.PSMP.ctl_param.angvel_r.raw = ctl{35,8};
sub07.PSMP.ctl_param.angvel_r.mean = mean(ctl{35,8},2,'omitnan');
sub07.PSMP.ctl_param.angvel_r.std = std(ctl{35,8},0,2,'omitnan');

sub01.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub01.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub01.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub01.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub02.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub02.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub02.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub02.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub03.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub03.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub03.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub03.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub04.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub04.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub04.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub04.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub05.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub05.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub05.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub05.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub06.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub06.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub06.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub06.PSMP.ctl_param.angvel_r.unit = 'rad/s';
sub07.MFMP.ctl_param.angvel_r.unit = 'rad/s';
sub07.PFMP.ctl_param.angvel_r.unit = 'rad/s';
sub07.MSMP.ctl_param.angvel_r.unit = 'rad/s';
sub07.PSMP.ctl_param.angvel_r.unit = 'rad/s';



% sub01.MFMP.ctl_param.angvel_l.raw = ctl{2,11};
sub01.MFMP.ctl_param.angvel_l.mean = mean(ctl{2,11},2,'omitnan');
sub01.MFMP.ctl_param.angvel_l.std = std(ctl{2,11},0,2,'omitnan');
% sub01.PFMP.ctl_param.angvel_l.raw = ctl{3,11};
sub01.PFMP.ctl_param.angvel_l.mean = mean(ctl{3,11},2,'omitnan');
sub01.PFMP.ctl_param.angvel_l.std = std(ctl{3,11},0,2,'omitnan');
% sub01.MSMP.ctl_param.angvel_l.raw = ctl{4,11};
sub01.MSMP.ctl_param.angvel_l.mean = mean(ctl{4,11},2,'omitnan');
sub01.MSMP.ctl_param.angvel_l.std = std(ctl{4,11},0,2,'omitnan');
% sub01.PSMP.ctl_param.angvel_l.raw = ctl{5,11};
sub01.PSMP.ctl_param.angvel_l.mean = mean(ctl{5,11},2,'omitnan');
sub01.PSMP.ctl_param.angvel_l.std = std(ctl{5,11},0,2,'omitnan');

% sub02.MFMP.ctl_param.angvel_l.raw = ctl{7,11};
sub02.MFMP.ctl_param.angvel_l.mean = mean(ctl{7,11},2,'omitnan');
sub02.MFMP.ctl_param.angvel_l.std = std(ctl{7,11},0,2,'omitnan');
% sub02.PFMP.ctl_param.angvel_l.raw = ctl{8,11};
sub02.PFMP.ctl_param.angvel_l.mean = mean(ctl{8,11},2,'omitnan');
sub02.PFMP.ctl_param.angvel_l.std = std(ctl{8,11},0,2,'omitnan');
% sub02.MSMP.ctl_param.angvel_l.raw = ctl{9,11};
sub02.MSMP.ctl_param.angvel_l.mean = mean(ctl{9,11},2,'omitnan');
sub02.MSMP.ctl_param.angvel_l.std = std(ctl{9,11},0,2,'omitnan');
% sub02.PSMP.ctl_param.angvel_l.raw = ctl{10,11};
sub02.PSMP.ctl_param.angvel_l.mean = mean(ctl{10,11},2,'omitnan');
sub02.PSMP.ctl_param.angvel_l.std = std(ctl{10,11},0,2,'omitnan');

% sub03.MFMP.ctl_param.angvel_l.raw = ctl{12,11};
sub03.MFMP.ctl_param.angvel_l.mean = mean(ctl{12,11},2,'omitnan');
sub03.MFMP.ctl_param.angvel_l.std = std(ctl{12,11},0,2,'omitnan');
% sub03.PFMP.ctl_param.angvel_l.raw = ctl{13,11};
sub03.PFMP.ctl_param.angvel_l.mean = mean(ctl{13,11},2,'omitnan');
sub03.PFMP.ctl_param.angvel_l.std = std(ctl{13,11},0,2,'omitnan');
% sub03.MSMP.ctl_param.angvel_l.raw = ctl{14,11};
sub03.MSMP.ctl_param.angvel_l.mean = mean(ctl{14,11},2,'omitnan');
sub03.MSMP.ctl_param.angvel_l.std = std(ctl{14,11},0,2,'omitnan');
% sub03.PSMP.ctl_param.angvel_l.raw = ctl{15,11};
sub03.PSMP.ctl_param.angvel_l.mean = mean(ctl{15,11},2,'omitnan');
sub03.PSMP.ctl_param.angvel_l.std = std(ctl{15,11},0,2,'omitnan');

% sub04.MFMP.ctl_param.angvel_l.raw = ctl{17,11};
sub04.MFMP.ctl_param.angvel_l.mean = mean(ctl{17,11},2,'omitnan');
sub04.MFMP.ctl_param.angvel_l.std = std(ctl{17,11},0,2,'omitnan');
% sub04.PFMP.ctl_param.angvel_l.raw = ctl{18,11};
sub04.PFMP.ctl_param.angvel_l.mean = mean(ctl{18,11},2,'omitnan');
sub04.PFMP.ctl_param.angvel_l.std = std(ctl{18,11},0,2,'omitnan');
% sub04.MSMP.ctl_param.angvel_l.raw = ctl{19,11};
sub04.MSMP.ctl_param.angvel_l.mean = mean(ctl{19,11},2,'omitnan');
sub04.MSMP.ctl_param.angvel_l.std = std(ctl{19,11},0,2,'omitnan');
% sub04.PSMP.ctl_param.angvel_l.raw = ctl{20,11};
sub04.PSMP.ctl_param.angvel_l.mean = mean(ctl{20,11},2,'omitnan');
sub04.PSMP.ctl_param.angvel_l.std = std(ctl{20,11},0,2,'omitnan');

% sub05.MFMP.ctl_param.angvel_l.raw = ctl{22,11};
sub05.MFMP.ctl_param.angvel_l.mean = mean(ctl{22,11},2,'omitnan');
sub05.MFMP.ctl_param.angvel_l.std = std(ctl{22,11},0,2,'omitnan');
% sub05.PFMP.ctl_param.angvel_l.raw = ctl{23,11};
sub05.PFMP.ctl_param.angvel_l.mean = mean(ctl{23,11},2,'omitnan');
sub05.PFMP.ctl_param.angvel_l.std = std(ctl{23,11},0,2,'omitnan');
% sub05.MSMP.ctl_param.angvel_l.raw = ctl{24,11};
sub05.MSMP.ctl_param.angvel_l.mean = mean(ctl{24,11},2,'omitnan');
sub05.MSMP.ctl_param.angvel_l.std = std(ctl{24,11},0,2,'omitnan');
% sub05.PSMP.ctl_param.angvel_l.raw = ctl{25,11};
sub05.PSMP.ctl_param.angvel_l.mean = mean(ctl{25,11},2,'omitnan');
sub05.PSMP.ctl_param.angvel_l.std = std(ctl{25,11},0,2,'omitnan');

% sub06.MFMP.ctl_param.angvel_l.raw = ctl{27,11};
sub06.MFMP.ctl_param.angvel_l.mean = mean(ctl{27,11},2,'omitnan');
sub06.MFMP.ctl_param.angvel_l.std = std(ctl{27,11},0,2,'omitnan');
% sub06.PFMP.ctl_param.angvel_l.raw = ctl{28,11};
sub06.PFMP.ctl_param.angvel_l.mean = mean(ctl{28,11},2,'omitnan');
sub06.PFMP.ctl_param.angvel_l.std = std(ctl{28,11},0,2,'omitnan');
% sub06.MSMP.ctl_param.angvel_l.raw = ctl{29,11};
sub06.MSMP.ctl_param.angvel_l.mean = mean(ctl{29,11},2,'omitnan');
sub06.MSMP.ctl_param.angvel_l.std = std(ctl{29,11},0,2,'omitnan');
% sub06.PSMP.ctl_param.angvel_l.raw = ctl{30,11};
sub06.PSMP.ctl_param.angvel_l.mean = mean(ctl{30,11},2,'omitnan');
sub06.PSMP.ctl_param.angvel_l.std = std(ctl{30,11},0,2,'omitnan');

% sub07.MFMP.ctl_param.angvel_l.raw = ctl{32,11};
sub07.MFMP.ctl_param.angvel_l.mean = mean(ctl{32,11},2,'omitnan');
sub07.MFMP.ctl_param.angvel_l.std = std(ctl{32,11},0,2,'omitnan');
% sub07.PFMP.ctl_param.angvel_l.raw = ctl{33,11};
sub07.PFMP.ctl_param.angvel_l.mean = mean(ctl{33,11},2,'omitnan');
sub07.PFMP.ctl_param.angvel_l.std = std(ctl{33,11},0,2,'omitnan');
% sub07.MSMP.ctl_param.angvel_l.raw = ctl{34,11};
sub07.MSMP.ctl_param.angvel_l.mean = mean(ctl{34,11},2,'omitnan');
sub07.MSMP.ctl_param.angvel_l.std = std(ctl{34,11},0,2,'omitnan');
% sub07.PSMP.ctl_param.angvel_l.raw = ctl{35,11};
sub07.PSMP.ctl_param.angvel_l.mean = mean(ctl{35,11},2,'omitnan');
sub07.PSMP.ctl_param.angvel_l.std = std(ctl{35,11},0,2,'omitnan');

sub01.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub01.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub01.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub01.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub02.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub02.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub02.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub02.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub03.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub03.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub03.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub03.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub04.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub04.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub04.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub04.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub05.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub05.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub05.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub05.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub06.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub06.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub06.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub06.PSMP.ctl_param.angvel_l.unit = 'rad/s';
sub07.MFMP.ctl_param.angvel_l.unit = 'rad/s';
sub07.PFMP.ctl_param.angvel_l.unit = 'rad/s';
sub07.MSMP.ctl_param.angvel_l.unit = 'rad/s';
sub07.PSMP.ctl_param.angvel_l.unit = 'rad/s';
%% ref Current

% sub01.MFMP.ctl_param.ref_cur_r.raw = ctl{2,9};
sub01.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{2,9},2,'omitnan');
sub01.MFMP.ctl_param.ref_cur_r.std = std(ctl{2,9},0,2,'omitnan');
% sub01.PFMP.ctl_param.ref_cur_r.raw = ctl{3,9};
sub01.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{3,9},2,'omitnan');
sub01.PFMP.ctl_param.ref_cur_r.std = std(ctl{3,9},0,2,'omitnan');
% sub01.MSMP.ctl_param.ref_cur_r.raw = ctl{4,9};
sub01.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{4,9},2,'omitnan');
sub01.MSMP.ctl_param.ref_cur_r.std = std(ctl{4,9},0,2,'omitnan');
% sub01.PSMP.ctl_param.ref_cur_r.raw = ctl{5,9};
sub01.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{5,9},2,'omitnan');
sub01.PSMP.ctl_param.ref_cur_r.std = std(ctl{5,9},0,2,'omitnan');

% sub02.MFMP.ctl_param.ref_cur_r.raw = ctl{7,9};
sub02.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{7,9},2,'omitnan');
sub02.MFMP.ctl_param.ref_cur_r.std = std(ctl{7,9},0,2,'omitnan');
% sub02.PFMP.ctl_param.ref_cur_r.raw = ctl{8,9};
sub02.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{8,9},2,'omitnan');
sub02.PFMP.ctl_param.ref_cur_r.std = std(ctl{8,9},0,2,'omitnan');
% sub02.MSMP.ctl_param.ref_cur_r.raw = ctl{9,9};
sub02.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{9,9},2,'omitnan');
sub02.MSMP.ctl_param.ref_cur_r.std = std(ctl{9,9},0,2,'omitnan');
% sub02.PSMP.ctl_param.ref_cur_r.raw = ctl{10,9};
sub02.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{10,9},2,'omitnan');
sub02.PSMP.ctl_param.ref_cur_r.std = std(ctl{10,9},0,2,'omitnan');

% sub03.MFMP.ctl_param.ref_cur_r.raw = ctl{12,9};
sub03.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{12,9},2,'omitnan');
sub03.MFMP.ctl_param.ref_cur_r.std = std(ctl{12,9},0,2,'omitnan');
% sub03.PFMP.ctl_param.ref_cur_r.raw = ctl{13,9};
sub03.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{13,9},2,'omitnan');
sub03.PFMP.ctl_param.ref_cur_r.std = std(ctl{13,9},0,2,'omitnan');
% sub03.MSMP.ctl_param.ref_cur_r.raw = ctl{14,9};
sub03.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{14,9},2,'omitnan');
sub03.MSMP.ctl_param.ref_cur_r.std = std(ctl{14,9},0,2,'omitnan');
% sub03.PSMP.ctl_param.ref_cur_r.raw = ctl{15,9};
sub03.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{15,9},2,'omitnan');
sub03.PSMP.ctl_param.ref_cur_r.std = std(ctl{15,9},0,2,'omitnan');

% sub04.MFMP.ctl_param.ref_cur_r.raw = ctl{17,9};
sub04.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{17,9},2,'omitnan');
sub04.MFMP.ctl_param.ref_cur_r.std = std(ctl{17,9},0,2,'omitnan');
% sub04.PFMP.ctl_param.ref_cur_r.raw = ctl{18,9};
sub04.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{18,9},2,'omitnan');
sub04.PFMP.ctl_param.ref_cur_r.std = std(ctl{18,9},0,2,'omitnan');
% sub04.MSMP.ctl_param.ref_cur_r.raw = ctl{19,9};
sub04.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{19,9},2,'omitnan');
sub04.MSMP.ctl_param.ref_cur_r.std = std(ctl{19,9},0,2,'omitnan');
% sub04.PSMP.ctl_param.ref_cur_r.raw = ctl{20,9};
sub04.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{20,9},2,'omitnan');
sub04.PSMP.ctl_param.ref_cur_r.std = std(ctl{20,9},0,2,'omitnan');

% sub05.MFMP.ctl_param.ref_cur_r.raw = ctl{22,9};
sub05.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{22,9},2,'omitnan');
sub05.MFMP.ctl_param.ref_cur_r.std = std(ctl{22,9},0,2,'omitnan');
% sub05.PFMP.ctl_param.ref_cur_r.raw = ctl{23,9};
sub05.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{23,9},2,'omitnan');
sub05.PFMP.ctl_param.ref_cur_r.std = std(ctl{23,9},0,2,'omitnan');
% sub05.MSMP.ctl_param.ref_cur_r.raw = ctl{24,9};
sub05.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{24,9},2,'omitnan');
sub05.MSMP.ctl_param.ref_cur_r.std = std(ctl{24,9},0,2,'omitnan');
% sub05.PSMP.ctl_param.ref_cur_r.raw = ctl{25,9};
sub05.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{25,9},2,'omitnan');
sub05.PSMP.ctl_param.ref_cur_r.std = std(ctl{25,9},0,2,'omitnan');

% sub06.MFMP.ctl_param.ref_cur_r.raw = ctl{27,9};
sub06.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{27,9},2,'omitnan');
sub06.MFMP.ctl_param.ref_cur_r.std = std(ctl{27,9},0,2,'omitnan');
% sub06.PFMP.ctl_param.ref_cur_r.raw = ctl{28,9};
sub06.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{28,9},2,'omitnan');
sub06.PFMP.ctl_param.ref_cur_r.std = std(ctl{28,9},0,2,'omitnan');
% sub06.MSMP.ctl_param.ref_cur_r.raw = ctl{29,9};
sub06.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{29,9},2,'omitnan');
sub06.MSMP.ctl_param.ref_cur_r.std = std(ctl{29,9},0,2,'omitnan');
% sub06.PSMP.ctl_param.ref_cur_r.raw = ctl{30,9};
sub06.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{30,9},2,'omitnan');
sub06.PSMP.ctl_param.ref_cur_r.std = std(ctl{30,9},0,2,'omitnan');

% sub07.MFMP.ctl_param.ref_cur_r.raw = ctl{32,9};
sub07.MFMP.ctl_param.ref_cur_r.mean = mean(ctl{32,9},2,'omitnan');
sub07.MFMP.ctl_param.ref_cur_r.std = std(ctl{32,9},0,2,'omitnan');
% sub07.PFMP.ctl_param.ref_cur_r.raw = ctl{33,9};
sub07.PFMP.ctl_param.ref_cur_r.mean = mean(ctl{33,9},2,'omitnan');
sub07.PFMP.ctl_param.ref_cur_r.std = std(ctl{33,9},0,2,'omitnan');
% sub07.MSMP.ctl_param.ref_cur_r.raw = ctl{34,9};
sub07.MSMP.ctl_param.ref_cur_r.mean = mean(ctl{34,9},2,'omitnan');
sub07.MSMP.ctl_param.ref_cur_r.std = std(ctl{34,9},0,2,'omitnan');
% sub07.PSMP.ctl_param.ref_cur_r.raw = ctl{35,9};
sub07.PSMP.ctl_param.ref_cur_r.mean = mean(ctl{35,9},2,'omitnan');
sub07.PSMP.ctl_param.ref_cur_r.std = std(ctl{35,9},0,2,'omitnan');

sub01.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub01.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub01.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub01.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub02.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub02.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub02.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub02.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub03.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub03.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub03.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub03.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub04.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub04.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub04.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub04.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub05.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub05.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub05.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub05.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub06.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub06.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub06.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub06.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub07.MFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub07.PFMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub07.MSMP.ctl_param.ref_cur_r.unit = '% norminal current';
sub07.PSMP.ctl_param.ref_cur_r.unit = '% norminal current';


% sub01.MFMP.ctl_param.ref_cur_l.raw = ctl{2,12};
sub01.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{2,12},2,'omitnan');
sub01.MFMP.ctl_param.ref_cur_l.std = std(ctl{2,12},0,2,'omitnan');
% sub01.PFMP.ctl_param.ref_cur_l.raw = ctl{3,12};
sub01.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{3,12},2,'omitnan');
sub01.PFMP.ctl_param.ref_cur_l.std = std(ctl{3,12},0,2,'omitnan');
% sub01.MSMP.ctl_param.ref_cur_l.raw = ctl{4,12};
sub01.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{4,12},2,'omitnan');
sub01.MSMP.ctl_param.ref_cur_l.std = std(ctl{4,12},0,2,'omitnan');
% sub01.PSMP.ctl_param.ref_cur_l.raw = ctl{5,12};
sub01.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{5,12},2,'omitnan');
sub01.PSMP.ctl_param.ref_cur_l.std = std(ctl{5,12},0,2,'omitnan');

% sub02.MFMP.ctl_param.ref_cur_l.raw = ctl{7,12};
sub02.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{7,12},2,'omitnan');
sub02.MFMP.ctl_param.ref_cur_l.std = std(ctl{7,12},0,2,'omitnan');
% sub02.PFMP.ctl_param.ref_cur_l.raw = ctl{8,12};
sub02.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{8,12},2,'omitnan');
sub02.PFMP.ctl_param.ref_cur_l.std = std(ctl{8,12},0,2,'omitnan');
% sub02.MSMP.ctl_param.ref_cur_l.raw = ctl{9,12};
sub02.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{9,12},2,'omitnan');
sub02.MSMP.ctl_param.ref_cur_l.std = std(ctl{9,12},0,2,'omitnan');
% sub02.PSMP.ctl_param.ref_cur_l.raw = ctl{10,12};
sub02.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{10,12},2,'omitnan');
sub02.PSMP.ctl_param.ref_cur_l.std = std(ctl{10,12},0,2,'omitnan');

% sub03.MFMP.ctl_param.ref_cur_l.raw = ctl{12,12};
sub03.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{12,12},2,'omitnan');
sub03.MFMP.ctl_param.ref_cur_l.std = std(ctl{12,12},0,2,'omitnan');
% sub03.PFMP.ctl_param.ref_cur_l.raw = ctl{13,12};
sub03.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{13,12},2,'omitnan');
sub03.PFMP.ctl_param.ref_cur_l.std = std(ctl{13,12},0,2,'omitnan');
% sub03.MSMP.ctl_param.ref_cur_l.raw = ctl{14,12};
sub03.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{14,12},2,'omitnan');
sub03.MSMP.ctl_param.ref_cur_l.std = std(ctl{14,12},0,2,'omitnan');
% sub03.PSMP.ctl_param.ref_cur_l.raw = ctl{15,12};
sub03.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{15,12},2,'omitnan');
sub03.PSMP.ctl_param.ref_cur_l.std = std(ctl{15,12},0,2,'omitnan');

% sub04.MFMP.ctl_param.ref_cur_l.raw = ctl{17,12};
sub04.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{17,12},2,'omitnan');
sub04.MFMP.ctl_param.ref_cur_l.std = std(ctl{17,12},0,2,'omitnan');
% sub04.PFMP.ctl_param.ref_cur_l.raw = ctl{18,12};
sub04.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{18,12},2,'omitnan');
sub04.PFMP.ctl_param.ref_cur_l.std = std(ctl{18,12},0,2,'omitnan');
% sub04.MSMP.ctl_param.ref_cur_l.raw = ctl{19,12};
sub04.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{19,12},2,'omitnan');
sub04.MSMP.ctl_param.ref_cur_l.std = std(ctl{19,12},0,2,'omitnan');
% sub04.PSMP.ctl_param.ref_cur_l.raw = ctl{20,12};
sub04.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{20,12},2,'omitnan');
sub04.PSMP.ctl_param.ref_cur_l.std = std(ctl{20,12},0,2,'omitnan');

% sub05.MFMP.ctl_param.ref_cur_l.raw = ctl{22,12};
sub05.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{22,12},2,'omitnan');
sub05.MFMP.ctl_param.ref_cur_l.std = std(ctl{22,12},0,2,'omitnan');
% sub05.PFMP.ctl_param.ref_cur_l.raw = ctl{23,12};
sub05.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{23,12},2,'omitnan');
sub05.PFMP.ctl_param.ref_cur_l.std = std(ctl{23,12},0,2,'omitnan');
% sub05.MSMP.ctl_param.ref_cur_l.raw = ctl{24,12};
sub05.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{24,12},2,'omitnan');
sub05.MSMP.ctl_param.ref_cur_l.std = std(ctl{24,12},0,2,'omitnan');
% sub05.PSMP.ctl_param.ref_cur_l.raw = ctl{25,12};
sub05.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{25,12},2,'omitnan');
sub05.PSMP.ctl_param.ref_cur_l.std = std(ctl{25,12},0,2,'omitnan');

% sub06.MFMP.ctl_param.ref_cur_l.raw = ctl{27,12};
sub06.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{27,12},2,'omitnan');
sub06.MFMP.ctl_param.ref_cur_l.std = std(ctl{27,12},0,2,'omitnan');
% sub06.PFMP.ctl_param.ref_cur_l.raw = ctl{28,12};
sub06.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{28,12},2,'omitnan');
sub06.PFMP.ctl_param.ref_cur_l.std = std(ctl{28,12},0,2,'omitnan');
% sub06.MSMP.ctl_param.ref_cur_l.raw = ctl{29,12};
sub06.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{29,12},2,'omitnan');
sub06.MSMP.ctl_param.ref_cur_l.std = std(ctl{29,12},0,2,'omitnan');
% sub06.PSMP.ctl_param.ref_cur_l.raw = ctl{30,12};
sub06.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{30,12},2,'omitnan');
sub06.PSMP.ctl_param.ref_cur_l.std = std(ctl{30,12},0,2,'omitnan');

% sub07.MFMP.ctl_param.ref_cur_l.raw = ctl{32,12};
sub07.MFMP.ctl_param.ref_cur_l.mean = mean(ctl{32,12},2,'omitnan');
sub07.MFMP.ctl_param.ref_cur_l.std = std(ctl{32,12},0,2,'omitnan');
% sub07.PFMP.ctl_param.ref_cur_l.raw = ctl{33,12};
sub07.PFMP.ctl_param.ref_cur_l.mean = mean(ctl{33,12},2,'omitnan');
sub07.PFMP.ctl_param.ref_cur_l.std = std(ctl{33,12},0,2,'omitnan');
% sub07.MSMP.ctl_param.ref_cur_l.raw = ctl{34,12};
sub07.MSMP.ctl_param.ref_cur_l.mean = mean(ctl{34,12},2,'omitnan');
sub07.MSMP.ctl_param.ref_cur_l.std = std(ctl{34,12},0,2,'omitnan');
% sub07.PSMP.ctl_param.ref_cur_l.raw = ctl{35,12};
sub07.PSMP.ctl_param.ref_cur_l.mean = mean(ctl{35,12},2,'omitnan');
sub07.PSMP.ctl_param.ref_cur_l.std = std(ctl{35,12},0,2,'omitnan');

sub01.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub01.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub01.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub01.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub02.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub02.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub02.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub02.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub03.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub03.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub03.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub03.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub04.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub04.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub04.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub04.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub05.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub05.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub05.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub05.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub06.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub06.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub06.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub06.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub07.MFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub07.PFMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub07.MSMP.ctl_param.ref_cur_l.unit = '% norminal current';
sub07.PSMP.ctl_param.ref_cur_l.unit = '% norminal current';
%% act current

% sub01.MFMP.ctl_param.act_cur_r.raw = ctl{2,10};
sub01.MFMP.ctl_param.act_cur_r.mean = mean(ctl{2,10},2,'omitnan');
sub01.MFMP.ctl_param.act_cur_r.std = std(ctl{2,10},0,2,'omitnan');
% sub01.PFMP.ctl_param.act_cur_r.raw = ctl{3,10};
sub01.PFMP.ctl_param.act_cur_r.mean = mean(ctl{3,10},2,'omitnan');
sub01.PFMP.ctl_param.act_cur_r.std = std(ctl{3,10},0,2,'omitnan');
% sub01.MSMP.ctl_param.act_cur_r.raw = ctl{4,10};
sub01.MSMP.ctl_param.act_cur_r.mean = mean(ctl{4,10},2,'omitnan');
sub01.MSMP.ctl_param.act_cur_r.std = std(ctl{4,10},0,2,'omitnan');
% sub01.PSMP.ctl_param.act_cur_r.raw = ctl{5,10};
sub01.PSMP.ctl_param.act_cur_r.mean = mean(ctl{5,10},2,'omitnan');
sub01.PSMP.ctl_param.act_cur_r.std = std(ctl{5,10},0,2,'omitnan');

% sub02.MFMP.ctl_param.act_cur_r.raw = ctl{7,10};
sub02.MFMP.ctl_param.act_cur_r.mean = mean(ctl{7,10},2,'omitnan');
sub02.MFMP.ctl_param.act_cur_r.std = std(ctl{7,10},0,2,'omitnan');
% sub02.PFMP.ctl_param.act_cur_r.raw = ctl{8,10};
sub02.PFMP.ctl_param.act_cur_r.mean = mean(ctl{8,10},2,'omitnan');
sub02.PFMP.ctl_param.act_cur_r.std = std(ctl{8,10},0,2,'omitnan');
% sub02.MSMP.ctl_param.act_cur_r.raw = ctl{9,10};
sub02.MSMP.ctl_param.act_cur_r.mean = mean(ctl{9,10},2,'omitnan');
sub02.MSMP.ctl_param.act_cur_r.std = std(ctl{9,10},0,2,'omitnan');
% sub02.PSMP.ctl_param.act_cur_r.raw = ctl{10,10};
sub02.PSMP.ctl_param.act_cur_r.mean = mean(ctl{10,10},2,'omitnan');
sub02.PSMP.ctl_param.act_cur_r.std = std(ctl{10,10},0,2,'omitnan');

% sub03.MFMP.ctl_param.act_cur_r.raw = ctl{12,10};
sub03.MFMP.ctl_param.act_cur_r.mean = mean(ctl{12,10},2,'omitnan');
sub03.MFMP.ctl_param.act_cur_r.std = std(ctl{12,10},0,2,'omitnan');
% sub03.PFMP.ctl_param.act_cur_r.raw = ctl{13,10};
sub03.PFMP.ctl_param.act_cur_r.mean = mean(ctl{13,10},2,'omitnan');
sub03.PFMP.ctl_param.act_cur_r.std = std(ctl{13,10},0,2,'omitnan');
% sub03.MSMP.ctl_param.act_cur_r.raw = ctl{14,10};
sub03.MSMP.ctl_param.act_cur_r.mean = mean(ctl{14,10},2,'omitnan');
sub03.MSMP.ctl_param.act_cur_r.std = std(ctl{14,10},0,2,'omitnan');
% sub03.PSMP.ctl_param.act_cur_r.raw = ctl{15,10};
sub03.PSMP.ctl_param.act_cur_r.mean = mean(ctl{15,10},2,'omitnan');
sub03.PSMP.ctl_param.act_cur_r.std = std(ctl{15,10},0,2,'omitnan');

% sub04.MFMP.ctl_param.act_cur_r.raw = ctl{17,10};
sub04.MFMP.ctl_param.act_cur_r.mean = mean(ctl{17,10},2,'omitnan');
sub04.MFMP.ctl_param.act_cur_r.std = std(ctl{17,10},0,2,'omitnan');
% sub04.PFMP.ctl_param.act_cur_r.raw = ctl{18,10};
sub04.PFMP.ctl_param.act_cur_r.mean = mean(ctl{18,10},2,'omitnan');
sub04.PFMP.ctl_param.act_cur_r.std = std(ctl{18,10},0,2,'omitnan');
% sub04.MSMP.ctl_param.act_cur_r.raw = ctl{19,10};
sub04.MSMP.ctl_param.act_cur_r.mean = mean(ctl{19,10},2,'omitnan');
sub04.MSMP.ctl_param.act_cur_r.std = std(ctl{19,10},0,2,'omitnan');
% sub04.PSMP.ctl_param.act_cur_r.raw = ctl{20,10};
sub04.PSMP.ctl_param.act_cur_r.mean = mean(ctl{20,10},2,'omitnan');
sub04.PSMP.ctl_param.act_cur_r.std = std(ctl{20,10},0,2,'omitnan');

% sub05.MFMP.ctl_param.act_cur_r.raw = ctl{22,10};
sub05.MFMP.ctl_param.act_cur_r.mean = mean(ctl{22,10},2,'omitnan');
sub05.MFMP.ctl_param.act_cur_r.std = std(ctl{22,10},0,2,'omitnan');
% sub05.PFMP.ctl_param.act_cur_r.raw = ctl{23,10};
sub05.PFMP.ctl_param.act_cur_r.mean = mean(ctl{23,10},2,'omitnan');
sub05.PFMP.ctl_param.act_cur_r.std = std(ctl{23,10},0,2,'omitnan');
% sub05.MSMP.ctl_param.act_cur_r.raw = ctl{24,10};
sub05.MSMP.ctl_param.act_cur_r.mean = mean(ctl{24,10},2,'omitnan');
sub05.MSMP.ctl_param.act_cur_r.std = std(ctl{24,10},0,2,'omitnan');
% sub05.PSMP.ctl_param.act_cur_r.raw = ctl{25,10};
sub05.PSMP.ctl_param.act_cur_r.mean = mean(ctl{25,10},2,'omitnan');
sub05.PSMP.ctl_param.act_cur_r.std = std(ctl{25,10},0,2,'omitnan');

% sub06.MFMP.ctl_param.act_cur_r.raw = ctl{27,10};
sub06.MFMP.ctl_param.act_cur_r.mean = mean(ctl{27,10},2,'omitnan');
sub06.MFMP.ctl_param.act_cur_r.std = std(ctl{27,10},0,2,'omitnan');
% sub06.PFMP.ctl_param.act_cur_r.raw = ctl{28,10};
sub06.PFMP.ctl_param.act_cur_r.mean = mean(ctl{28,10},2,'omitnan');
sub06.PFMP.ctl_param.act_cur_r.std = std(ctl{28,10},0,2,'omitnan');
% sub06.MSMP.ctl_param.act_cur_r.raw = ctl{29,10};
sub06.MSMP.ctl_param.act_cur_r.mean = mean(ctl{29,10},2,'omitnan');
sub06.MSMP.ctl_param.act_cur_r.std = std(ctl{29,10},0,2,'omitnan');
% sub06.PSMP.ctl_param.act_cur_r.raw = ctl{30,10};
sub06.PSMP.ctl_param.act_cur_r.mean = mean(ctl{30,10},2,'omitnan');
sub06.PSMP.ctl_param.act_cur_r.std = std(ctl{30,10},0,2,'omitnan');

% sub07.MFMP.ctl_param.act_cur_r.raw = ctl{32,10};
sub07.MFMP.ctl_param.act_cur_r.mean = mean(ctl{32,10},2,'omitnan');
sub07.MFMP.ctl_param.act_cur_r.std = std(ctl{32,10},0,2,'omitnan');
% sub07.PFMP.ctl_param.act_cur_r.raw = ctl{33,10};
sub07.PFMP.ctl_param.act_cur_r.mean = mean(ctl{33,10},2,'omitnan');
sub07.PFMP.ctl_param.act_cur_r.std = std(ctl{33,10},0,2,'omitnan');
% sub07.MSMP.ctl_param.act_cur_r.raw = ctl{34,10};
sub07.MSMP.ctl_param.act_cur_r.mean = mean(ctl{34,10},2,'omitnan');
sub07.MSMP.ctl_param.act_cur_r.std = std(ctl{34,10},0,2,'omitnan');
% sub07.PSMP.ctl_param.act_cur_r.raw = ctl{35,10};
sub07.PSMP.ctl_param.act_cur_r.mean = mean(ctl{35,10},2,'omitnan');
sub07.PSMP.ctl_param.act_cur_r.std = std(ctl{35,10},0,2,'omitnan');

sub01.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub01.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub01.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub01.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub02.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub02.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub02.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub02.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub03.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub03.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub03.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub03.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub04.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub04.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub04.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub04.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub05.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub05.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub05.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub05.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub06.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub06.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub06.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub06.PSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub07.MFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub07.PFMP.ctl_param.act_cur_r.unit = '% norminal current';
sub07.MSMP.ctl_param.act_cur_r.unit = '% norminal current';
sub07.PSMP.ctl_param.act_cur_r.unit = '% norminal current';


% sub01.MFMP.ctl_param.act_cur_l.raw = ctl{2,13};
sub01.MFMP.ctl_param.act_cur_l.mean = mean(ctl{2,13},2,'omitnan');
sub01.MFMP.ctl_param.act_cur_l.std = std(ctl{2,13},0,2,'omitnan');
% sub01.PFMP.ctl_param.act_cur_l.raw = ctl{3,13};
sub01.PFMP.ctl_param.act_cur_l.mean = mean(ctl{3,13},2,'omitnan');
sub01.PFMP.ctl_param.act_cur_l.std = std(ctl{3,13},0,2,'omitnan');
% sub01.MSMP.ctl_param.act_cur_l.raw = ctl{4,13};
sub01.MSMP.ctl_param.act_cur_l.mean = mean(ctl{4,13},2,'omitnan');
sub01.MSMP.ctl_param.act_cur_l.std = std(ctl{4,13},0,2,'omitnan');
% sub01.PSMP.ctl_param.act_cur_l.raw = ctl{5,13};
sub01.PSMP.ctl_param.act_cur_l.mean = mean(ctl{5,13},2,'omitnan');
sub01.PSMP.ctl_param.act_cur_l.std = std(ctl{5,13},0,2,'omitnan');

% sub02.MFMP.ctl_param.act_cur_l.raw = ctl{7,13};
sub02.MFMP.ctl_param.act_cur_l.mean = mean(ctl{7,13},2,'omitnan');
sub02.MFMP.ctl_param.act_cur_l.std = std(ctl{7,13},0,2,'omitnan');
% sub02.PFMP.ctl_param.act_cur_l.raw = ctl{8,13};
sub02.PFMP.ctl_param.act_cur_l.mean = mean(ctl{8,13},2,'omitnan');
sub02.PFMP.ctl_param.act_cur_l.std = std(ctl{8,13},0,2,'omitnan');
% sub02.MSMP.ctl_param.act_cur_l.raw = ctl{9,13};
sub02.MSMP.ctl_param.act_cur_l.mean = mean(ctl{9,13},2,'omitnan');
sub02.MSMP.ctl_param.act_cur_l.std = std(ctl{9,13},0,2,'omitnan');
% sub02.PSMP.ctl_param.act_cur_l.raw = ctl{10,13};
sub02.PSMP.ctl_param.act_cur_l.mean = mean(ctl{10,13},2,'omitnan');
sub02.PSMP.ctl_param.act_cur_l.std = std(ctl{10,13},0,2,'omitnan');

% sub03.MFMP.ctl_param.act_cur_l.raw = ctl{12,13};
sub03.MFMP.ctl_param.act_cur_l.mean = mean(ctl{12,13},2,'omitnan');
sub03.MFMP.ctl_param.act_cur_l.std = std(ctl{12,13},0,2,'omitnan');
% sub03.PFMP.ctl_param.act_cur_l.raw = ctl{13,13};
sub03.PFMP.ctl_param.act_cur_l.mean = mean(ctl{13,13},2,'omitnan');
sub03.PFMP.ctl_param.act_cur_l.std = std(ctl{13,13},0,2,'omitnan');
% sub03.MSMP.ctl_param.act_cur_l.raw = ctl{14,13};
sub03.MSMP.ctl_param.act_cur_l.mean = mean(ctl{14,13},2,'omitnan');
sub03.MSMP.ctl_param.act_cur_l.std = std(ctl{14,13},0,2,'omitnan');
% sub03.PSMP.ctl_param.act_cur_l.raw = ctl{15,13};
sub03.PSMP.ctl_param.act_cur_l.mean = mean(ctl{15,13},2,'omitnan');
sub03.PSMP.ctl_param.act_cur_l.std = std(ctl{15,13},0,2,'omitnan');

% sub04.MFMP.ctl_param.act_cur_l.raw = ctl{17,13};
sub04.MFMP.ctl_param.act_cur_l.mean = mean(ctl{17,13},2,'omitnan');
sub04.MFMP.ctl_param.act_cur_l.std = std(ctl{17,13},0,2,'omitnan');
% sub04.PFMP.ctl_param.act_cur_l.raw = ctl{18,13};
sub04.PFMP.ctl_param.act_cur_l.mean = mean(ctl{18,13},2,'omitnan');
sub04.PFMP.ctl_param.act_cur_l.std = std(ctl{18,13},0,2,'omitnan');
% sub04.MSMP.ctl_param.act_cur_l.raw = ctl{19,13};
sub04.MSMP.ctl_param.act_cur_l.mean = mean(ctl{19,13},2,'omitnan');
sub04.MSMP.ctl_param.act_cur_l.std = std(ctl{19,13},0,2,'omitnan');
% sub04.PSMP.ctl_param.act_cur_l.raw = ctl{20,13};
sub04.PSMP.ctl_param.act_cur_l.mean = mean(ctl{20,13},2,'omitnan');
sub04.PSMP.ctl_param.act_cur_l.std = std(ctl{20,13},0,2,'omitnan');

% sub05.MFMP.ctl_param.act_cur_l.raw = ctl{22,13};
sub05.MFMP.ctl_param.act_cur_l.mean = mean(ctl{22,13},2,'omitnan');
sub05.MFMP.ctl_param.act_cur_l.std = std(ctl{22,13},0,2,'omitnan');
% sub05.PFMP.ctl_param.act_cur_l.raw = ctl{23,13};
sub05.PFMP.ctl_param.act_cur_l.mean = mean(ctl{23,13},2,'omitnan');
sub05.PFMP.ctl_param.act_cur_l.std = std(ctl{23,13},0,2,'omitnan');
% sub05.MSMP.ctl_param.act_cur_l.raw = ctl{24,13};
sub05.MSMP.ctl_param.act_cur_l.mean = mean(ctl{24,13},2,'omitnan');
sub05.MSMP.ctl_param.act_cur_l.std = std(ctl{24,13},0,2,'omitnan');
% sub05.PSMP.ctl_param.act_cur_l.raw = ctl{25,13};
sub05.PSMP.ctl_param.act_cur_l.mean = mean(ctl{25,13},2,'omitnan');
sub05.PSMP.ctl_param.act_cur_l.std = std(ctl{25,13},0,2,'omitnan');

% sub06.MFMP.ctl_param.act_cur_l.raw = ctl{27,13};
sub06.MFMP.ctl_param.act_cur_l.mean = mean(ctl{27,13},2,'omitnan');
sub06.MFMP.ctl_param.act_cur_l.std = std(ctl{27,13},0,2,'omitnan');
% sub06.PFMP.ctl_param.act_cur_l.raw = ctl{28,13};
sub06.PFMP.ctl_param.act_cur_l.mean = mean(ctl{28,13},2,'omitnan');
sub06.PFMP.ctl_param.act_cur_l.std = std(ctl{28,13},0,2,'omitnan');
% sub06.MSMP.ctl_param.act_cur_l.raw = ctl{29,13};
sub06.MSMP.ctl_param.act_cur_l.mean = mean(ctl{29,13},2,'omitnan');
sub06.MSMP.ctl_param.act_cur_l.std = std(ctl{29,13},0,2,'omitnan');
% sub06.PSMP.ctl_param.act_cur_l.raw = ctl{30,13};
sub06.PSMP.ctl_param.act_cur_l.mean = mean(ctl{30,13},2,'omitnan');
sub06.PSMP.ctl_param.act_cur_l.std = std(ctl{30,13},0,2,'omitnan');

% sub07.MFMP.ctl_param.act_cur_l.raw = ctl{32,13};
sub07.MFMP.ctl_param.act_cur_l.mean = mean(ctl{32,13},2,'omitnan');
sub07.MFMP.ctl_param.act_cur_l.std = std(ctl{32,13},0,2,'omitnan');
% sub07.PFMP.ctl_param.act_cur_l.raw = ctl{33,13};
sub07.PFMP.ctl_param.act_cur_l.mean = mean(ctl{33,13},2,'omitnan');
sub07.PFMP.ctl_param.act_cur_l.std = std(ctl{33,13},0,2,'omitnan');
% sub07.MSMP.ctl_param.act_cur_l.raw = ctl{34,13};
sub07.MSMP.ctl_param.act_cur_l.mean = mean(ctl{34,13},2,'omitnan');
sub07.MSMP.ctl_param.act_cur_l.std = std(ctl{34,13},0,2,'omitnan');
% sub07.PSMP.ctl_param.act_cur_l.raw = ctl{35,13};
sub07.PSMP.ctl_param.act_cur_l.mean = mean(ctl{35,13},2,'omitnan');
sub07.PSMP.ctl_param.act_cur_l.std = std(ctl{35,13},0,2,'omitnan');

sub01.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub01.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub01.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub01.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub02.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub02.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub02.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub02.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub03.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub03.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub03.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub03.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub04.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub04.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub04.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub04.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub05.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub05.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub05.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub05.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub06.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub06.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub06.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub06.PSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub07.MFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub07.PFMP.ctl_param.act_cur_l.unit = '% norminal current';
sub07.MSMP.ctl_param.act_cur_l.unit = '% norminal current';
sub07.PSMP.ctl_param.act_cur_l.unit = '% norminal current';



%% kinetic com
%% position
% sub01.NW.k_CoM.pos.raw = COM{1,1};
sub01.NW.k_CoM.pos.mean = mean(COM{1,1},2,'omitnan');
sub01.NW.k_CoM.pos.std = std(COM{1,1},0,2,'omitnan');
% sub01.MFMP.k_CoM.pos.raw = COM{2,1};
sub01.MFMP.k_CoM.pos.mean = mean(COM{2,1},2,'omitnan');
sub01.MFMP.k_CoM.pos.std = std(COM{2,1},0,2,'omitnan');
% sub01.PFMP.k_CoM.pos.raw = COM{3,1};
sub01.PFMP.k_CoM.pos.mean = mean(COM{3,1},2,'omitnan');
sub01.PFMP.k_CoM.pos.std = std(COM{3,1},0,2,'omitnan');
% sub01.MSMP.k_CoM.pos.raw = COM{4,1};
sub01.MSMP.k_CoM.pos.mean = mean(COM{4,1},2,'omitnan');
sub01.MSMP.k_CoM.pos.std = std(COM{4,1},0,2,'omitnan');
% sub01.PSMP.k_CoM.pos.raw = COM{5,1};
sub01.PSMP.k_CoM.pos.mean = mean(COM{5,1},2,'omitnan');
sub01.PSMP.k_CoM.pos.std = std(COM{5,1},0,2,'omitnan');

% sub02.NW.k_CoM.pos.raw = COM{6,1};
sub02.NW.k_CoM.pos.mean = mean(COM{6,1},2,'omitnan');
sub02.NW.k_CoM.pos.std = std(COM{6,1},0,2,'omitnan');
% sub02.MFMP.k_CoM.pos.raw = COM{7,1};
sub02.MFMP.k_CoM.pos.mean = mean(COM{7,1},2,'omitnan');
sub02.MFMP.k_CoM.pos.std = std(COM{7,1},0,2,'omitnan');
% sub02.PFMP.k_CoM.pos.raw = COM{8,1};
sub02.PFMP.k_CoM.pos.mean = mean(COM{8,1},2,'omitnan');
sub02.PFMP.k_CoM.pos.std = std(COM{8,1},0,2,'omitnan');
% sub02.MSMP.k_CoM.pos.raw = COM{9,1};
sub02.MSMP.k_CoM.pos.mean = mean(COM{9,1},2,'omitnan');
sub02.MSMP.k_CoM.pos.std = std(COM{9,1},0,2,'omitnan');
% sub02.PSMP.k_CoM.pos.raw = COM{10,1};
sub02.PSMP.k_CoM.pos.mean = mean(COM{10,1},2,'omitnan');
sub02.PSMP.k_CoM.pos.std = std(COM{10,1},0,2,'omitnan');

% sub03.NW.k_CoM.pos.raw = COM{11,1};
sub03.NW.k_CoM.pos.mean = mean(COM{1,1},2,'omitnan');
sub03.NW.k_CoM.pos.std = std(COM{11,1},0,2,'omitnan');
% sub03.MFMP.k_CoM.pos.raw = COM{12,1};
sub03.MFMP.k_CoM.pos.mean = mean(COM{12,1},2,'omitnan');
sub03.MFMP.k_CoM.pos.std = std(COM{12,1},0,2,'omitnan');
% sub03.PFMP.k_CoM.pos.raw = COM{13,1};
sub03.PFMP.k_CoM.pos.mean = mean(COM{13,1},2,'omitnan');
sub03.PFMP.k_CoM.pos.std = std(COM{13,1},0,2,'omitnan');
% sub03.MSMP.k_CoM.pos.raw = COM{14,1};
sub03.MSMP.k_CoM.pos.mean = mean(COM{14,1},2,'omitnan');
sub03.MSMP.k_CoM.pos.std = std(COM{14,1},0,2,'omitnan');
% sub03.PSMP.k_CoM.pos.raw = COM{15,1};
sub03.PSMP.k_CoM.pos.mean = mean(COM{15,1},2,'omitnan');
sub03.PSMP.k_CoM.pos.std = std(COM{15,1},0,2,'omitnan');

% sub04.NW.k_CoM.pos.raw = COM{16,1};
sub04.NW.k_CoM.pos.mean = mean(COM{16,1},2,'omitnan');
sub04.NW.k_CoM.pos.std = std(COM{16,1},0,2,'omitnan');
% sub04.MFMP.k_CoM.pos.raw = COM{17,1};
sub04.MFMP.k_CoM.pos.mean = mean(COM{17,1},2,'omitnan');
sub04.MFMP.k_CoM.pos.std = std(COM{17,1},0,2,'omitnan');
% sub04.PFMP.k_CoM.pos.raw = COM{18,1};
sub04.PFMP.k_CoM.pos.mean = mean(COM{18,1},2,'omitnan');
sub04.PFMP.k_CoM.pos.std = std(COM{18,1},0,2,'omitnan');
% sub04.MSMP.k_CoM.pos.raw = COM{19,1};
sub04.MSMP.k_CoM.pos.mean = mean(COM{19,1},2,'omitnan');
sub04.MSMP.k_CoM.pos.std = std(COM{19,1},0,2,'omitnan');
% sub04.PSMP.k_CoM.pos.raw = COM{20,1};
sub04.PSMP.k_CoM.pos.mean = mean(COM{20,1},2,'omitnan');
sub04.PSMP.k_CoM.pos.std = std(COM{20,1},0,2,'omitnan');


% sub05.NW.k_CoM.pos.raw = COM{21,1};
sub05.NW.k_CoM.pos.mean = mean(COM{21,1},2,'omitnan');
sub05.NW.k_CoM.pos.std = std(COM{21,1},0,2,'omitnan');
% sub05.MFMP.k_CoM.pos.raw = COM{22,1};
sub05.MFMP.k_CoM.pos.mean = mean(COM{22,1},2,'omitnan');
sub05.MFMP.k_CoM.pos.std = std(COM{22,1},0,2,'omitnan');
% sub05.PFMP.k_CoM.pos.raw = COM{23,1};
sub05.PFMP.k_CoM.pos.mean = mean(COM{23,1},2,'omitnan');
sub05.PFMP.k_CoM.pos.std = std(COM{23,1},0,2,'omitnan');
% sub05.MSMP.k_CoM.pos.raw = COM{24,1};
sub05.MSMP.k_CoM.pos.mean = mean(COM{24,1},2,'omitnan');
sub05.MSMP.k_CoM.pos.std = std(COM{24,1},0,2,'omitnan');
% sub05.PSMP.k_CoM.pos.raw = COM{25,1};
sub05.PSMP.k_CoM.pos.mean = mean(COM{25,1},2,'omitnan');
sub05.PSMP.k_CoM.pos.std = std(COM{25,1},0,2,'omitnan');


% sub06.NW.k_CoM.pos.raw = COM{26,1};
sub06.NW.k_CoM.pos.mean = mean(COM{26,1},2,'omitnan');
sub06.NW.k_CoM.pos.std = std(COM{26,1},0,2,'omitnan');
% sub06.MFMP.k_CoM.pos.raw = COM{27,1};
sub06.MFMP.k_CoM.pos.mean = mean(COM{27,1},2,'omitnan');
sub06.MFMP.k_CoM.pos.std = std(COM{27,1},0,2,'omitnan');
% sub06.PFMP.k_CoM.pos.raw = COM{28,1};
sub06.PFMP.k_CoM.pos.mean = mean(COM{28,1},2,'omitnan');
sub06.PFMP.k_CoM.pos.std = std(COM{28,1},0,2,'omitnan');
% sub06.MSMP.k_CoM.pos.raw = COM{29,1};
sub06.MSMP.k_CoM.pos.mean = mean(COM{29,1},2,'omitnan');
sub06.MSMP.k_CoM.pos.std = std(COM{29,1},0,2,'omitnan');
% sub06.PSMP.k_CoM.pos.raw = COM{30,1};
sub06.PSMP.k_CoM.pos.mean = mean(COM{30,1},2,'omitnan');
sub06.PSMP.k_CoM.pos.std = std(COM{30,1},0,2,'omitnan');


% sub07.NW.k_CoM.pos.raw = COM{31,1};
sub07.NW.k_CoM.pos.mean = mean(COM{31,1},2,'omitnan');
sub07.NW.k_CoM.pos.std = std(COM{31,1},0,2,'omitnan');
% sub07.MFMP.k_CoM.pos.raw = COM{32,1};
sub07.MFMP.k_CoM.pos.mean = mean(COM{32,1},2,'omitnan');
sub07.MFMP.k_CoM.pos.std = std(COM{32,1},0,2,'omitnan');
% sub07.PFMP.k_CoM.pos.raw = COM{33,1};
sub07.PFMP.k_CoM.pos.mean = mean(COM{33,1},2,'omitnan');
sub07.PFMP.k_CoM.pos.std = std(COM{33,1},0,2,'omitnan');
% sub07.MSMP.k_CoM.pos.raw = COM{34,1};
sub07.MSMP.k_CoM.pos.mean = mean(COM{34,1},2,'omitnan');
sub07.MSMP.k_CoM.pos.std = std(COM{34,1},0,2,'omitnan');
% sub07.PSMP.k_CoM.pos.raw = COM{35,1};
sub07.PSMP.k_CoM.pos.mean = mean(COM{35,1},2,'omitnan');
sub07.PSMP.k_CoM.pos.std = std(COM{35,1},0,2,'omitnan');

sub01.NW.k_CoM.pos.unit = 'm';
sub02.NW.k_CoM.pos.unit = 'm';
sub03.NW.k_CoM.pos.unit = 'm';
sub04.NW.k_CoM.pos.unit = 'm';
sub05.NW.k_CoM.pos.unit = 'm';
sub06.NW.k_CoM.pos.unit = 'm';
sub07.NW.k_CoM.pos.unit = 'm';
sub01.MFMP.k_CoM.pos.unit = 'm';
sub01.PFMP.k_CoM.pos.unit = 'm';
sub01.MSMP.k_CoM.pos.unit = 'm';
sub01.PSMP.k_CoM.pos.unit = 'm';
sub02.MFMP.k_CoM.pos.unit = 'm';
sub02.PFMP.k_CoM.pos.unit = 'm';
sub02.MSMP.k_CoM.pos.unit = 'm';
sub02.PSMP.k_CoM.pos.unit = 'm';
sub03.MFMP.k_CoM.pos.unit = 'm';
sub03.PFMP.k_CoM.pos.unit = 'm';
sub03.MSMP.k_CoM.pos.unit = 'm';
sub03.PSMP.k_CoM.pos.unit = 'm';
sub04.MFMP.k_CoM.pos.unit = 'm';
sub04.PFMP.k_CoM.pos.unit = 'm';
sub04.MSMP.k_CoM.pos.unit = 'm';
sub04.PSMP.k_CoM.pos.unit = 'm';
sub05.MFMP.k_CoM.pos.unit = 'm';
sub05.PFMP.k_CoM.pos.unit = 'm';
sub05.MSMP.k_CoM.pos.unit = 'm';
sub05.PSMP.k_CoM.pos.unit = 'm';
sub06.MFMP.k_CoM.pos.unit = 'm';
sub06.PFMP.k_CoM.pos.unit = 'm';
sub06.MSMP.k_CoM.pos.unit = 'm';
sub06.PSMP.k_CoM.pos.unit = 'm';
sub07.MFMP.k_CoM.pos.unit = 'm';
sub07.PFMP.k_CoM.pos.unit = 'm';
sub07.MSMP.k_CoM.pos.unit = 'm';
sub07.PSMP.k_CoM.pos.unit = 'm';
%% velocity
% sub01.NW.k_CoM.vel.raw = COM{1,4};
sub01.NW.k_CoM.vel.mean = mean(COM{1,4},2,'omitnan');
sub01.NW.k_CoM.vel.std = std(COM{1,4},0,2,'omitnan');
% sub01.MFMP.k_CoM.vel.raw = COM{2,4};
sub01.MFMP.k_CoM.vel.mean = mean(COM{2,4},2,'omitnan');
sub01.MFMP.k_CoM.vel.std = std(COM{2,4},0,2,'omitnan');
% sub01.PFMP.k_CoM.vel.raw = COM{3,4};
sub01.PFMP.k_CoM.vel.mean = mean(COM{3,4},2,'omitnan');
sub01.PFMP.k_CoM.vel.std = std(COM{3,4},0,2,'omitnan');
% sub01.MSMP.k_CoM.vel.raw = COM{4,4};
sub01.MSMP.k_CoM.vel.mean = mean(COM{4,4},2,'omitnan');
sub01.MSMP.k_CoM.vel.std = std(COM{4,4},0,2,'omitnan');
% sub01.PSMP.k_CoM.vel.raw = COM{5,4};
sub01.PSMP.k_CoM.vel.mean = mean(COM{5,4},2,'omitnan');
sub01.PSMP.k_CoM.vel.std = std(COM{5,4},0,2,'omitnan');

% sub02.NW.k_CoM.vel.raw = COM{6,4};
sub02.NW.k_CoM.vel.mean = mean(COM{6,4},2,'omitnan');
sub02.NW.k_CoM.vel.std = std(COM{6,4},0,2,'omitnan');
% sub02.MFMP.k_CoM.vel.raw = COM{7,4};
sub02.MFMP.k_CoM.vel.mean = mean(COM{7,4},2,'omitnan');
sub02.MFMP.k_CoM.vel.std = std(COM{7,4},0,2,'omitnan');
% sub02.PFMP.k_CoM.vel.raw = COM{8,4};
sub02.PFMP.k_CoM.vel.mean = mean(COM{8,4},2,'omitnan');
sub02.PFMP.k_CoM.vel.std = std(COM{8,4},0,2,'omitnan');
% sub02.MSMP.k_CoM.vel.raw = COM{9,4};
sub02.MSMP.k_CoM.vel.mean = mean(COM{9,4},2,'omitnan');
sub02.MSMP.k_CoM.vel.std = std(COM{9,4},0,2,'omitnan');
% sub02.PSMP.k_CoM.vel.raw = COM{10,4};
sub02.PSMP.k_CoM.vel.mean = mean(COM{10,4},2,'omitnan');
sub02.PSMP.k_CoM.vel.std = std(COM{10,4},0,2,'omitnan');

% sub03.NW.k_CoM.vel.raw = COM{11,4};
sub03.NW.k_CoM.vel.mean = mean(COM{1,4},2,'omitnan');
sub03.NW.k_CoM.vel.std = std(COM{11,4},0,2,'omitnan');
% sub03.MFMP.k_CoM.vel.raw = COM{12,4};
sub03.MFMP.k_CoM.vel.mean = mean(COM{12,4},2,'omitnan');
sub03.MFMP.k_CoM.vel.std = std(COM{12,4},0,2,'omitnan');
% sub03.PFMP.k_CoM.vel.raw = COM{13,4};
sub03.PFMP.k_CoM.vel.mean = mean(COM{13,4},2,'omitnan');
sub03.PFMP.k_CoM.vel.std = std(COM{13,4},0,2,'omitnan');
% sub03.MSMP.k_CoM.vel.raw = COM{14,4};
sub03.MSMP.k_CoM.vel.mean = mean(COM{14,4},2,'omitnan');
sub03.MSMP.k_CoM.vel.std = std(COM{14,4},0,2,'omitnan');
% sub03.PSMP.k_CoM.vel.raw = COM{15,4};
sub03.PSMP.k_CoM.vel.mean = mean(COM{15,4},2,'omitnan');
sub03.PSMP.k_CoM.vel.std = std(COM{15,4},0,2,'omitnan');

% sub04.NW.k_CoM.vel.raw = COM{16,4};
sub04.NW.k_CoM.vel.mean = mean(COM{16,4},2,'omitnan');
sub04.NW.k_CoM.vel.std = std(COM{16,4},0,2,'omitnan');
% sub04.MFMP.k_CoM.vel.raw = COM{17,4};
sub04.MFMP.k_CoM.vel.mean = mean(COM{17,4},2,'omitnan');
sub04.MFMP.k_CoM.vel.std = std(COM{17,4},0,2,'omitnan');
% sub04.PFMP.k_CoM.vel.raw = COM{18,4};
sub04.PFMP.k_CoM.vel.mean = mean(COM{18,4},2,'omitnan');
sub04.PFMP.k_CoM.vel.std = std(COM{18,4},0,2,'omitnan');
% sub04.MSMP.k_CoM.vel.raw = COM{19,4};
sub04.MSMP.k_CoM.vel.mean = mean(COM{19,4},2,'omitnan');
sub04.MSMP.k_CoM.vel.std = std(COM{19,4},0,2,'omitnan');
% sub04.PSMP.k_CoM.vel.raw = COM{20,4};
sub04.PSMP.k_CoM.vel.mean = mean(COM{20,4},2,'omitnan');
sub04.PSMP.k_CoM.vel.std = std(COM{20,4},0,2,'omitnan');


% sub05.NW.k_CoM.vel.raw = COM{21,4};
sub05.NW.k_CoM.vel.mean = mean(COM{21,4},2,'omitnan');
sub05.NW.k_CoM.vel.std = std(COM{21,4},0,2,'omitnan');
% sub05.MFMP.k_CoM.vel.raw = COM{22,4};
sub05.MFMP.k_CoM.vel.mean = mean(COM{22,4},2,'omitnan');
sub05.MFMP.k_CoM.vel.std = std(COM{22,4},0,2,'omitnan');
% sub05.PFMP.k_CoM.vel.raw = COM{23,4};
sub05.PFMP.k_CoM.vel.mean = mean(COM{23,4},2,'omitnan');
sub05.PFMP.k_CoM.vel.std = std(COM{23,4},0,2,'omitnan');
% sub05.MSMP.k_CoM.vel.raw = COM{24,4};
sub05.MSMP.k_CoM.vel.mean = mean(COM{24,4},2,'omitnan');
sub05.MSMP.k_CoM.vel.std = std(COM{24,4},0,2,'omitnan');
% sub05.PSMP.k_CoM.vel.raw = COM{25,4};
sub05.PSMP.k_CoM.vel.mean = mean(COM{25,4},2,'omitnan');
sub05.PSMP.k_CoM.vel.std = std(COM{25,4},0,2,'omitnan');


% sub06.NW.k_CoM.vel.raw = COM{26,4};
sub06.NW.k_CoM.vel.mean = mean(COM{26,4},2,'omitnan');
sub06.NW.k_CoM.vel.std = std(COM{26,4},0,2,'omitnan');
% sub06.MFMP.k_CoM.vel.raw = COM{27,4};
sub06.MFMP.k_CoM.vel.mean = mean(COM{27,4},2,'omitnan');
sub06.MFMP.k_CoM.vel.std = std(COM{27,4},0,2,'omitnan');
% sub06.PFMP.k_CoM.vel.raw = COM{28,4};
sub06.PFMP.k_CoM.vel.mean = mean(COM{28,4},2,'omitnan');
sub06.PFMP.k_CoM.vel.std = std(COM{28,4},0,2,'omitnan');
% sub06.MSMP.k_CoM.vel.raw = COM{29,4};
sub06.MSMP.k_CoM.vel.mean = mean(COM{29,4},2,'omitnan');
sub06.MSMP.k_CoM.vel.std = std(COM{29,4},0,2,'omitnan');
% sub06.PSMP.k_CoM.vel.raw = COM{30,4};
sub06.PSMP.k_CoM.vel.mean = mean(COM{30,4},2,'omitnan');
sub06.PSMP.k_CoM.vel.std = std(COM{30,4},0,2,'omitnan');


% sub07.NW.k_CoM.vel.raw = COM{31,4};
sub07.NW.k_CoM.vel.mean = mean(COM{31,4},2,'omitnan');
sub07.NW.k_CoM.vel.std = std(COM{31,4},0,2,'omitnan');
% sub07.MFMP.k_CoM.vel.raw = COM{32,4};
sub07.MFMP.k_CoM.vel.mean = mean(COM{32,4},2,'omitnan');
sub07.MFMP.k_CoM.vel.std = std(COM{32,4},0,2,'omitnan');
% sub07.PFMP.k_CoM.vel.raw = COM{33,4};
sub07.PFMP.k_CoM.vel.mean = mean(COM{33,4},2,'omitnan');
sub07.PFMP.k_CoM.vel.std = std(COM{33,4},0,2,'omitnan');
% sub07.MSMP.k_CoM.vel.raw = COM{34,4};
sub07.MSMP.k_CoM.vel.mean = mean(COM{34,4},2,'omitnan');
sub07.MSMP.k_CoM.vel.std = std(COM{34,4},0,2,'omitnan');
% sub07.PSMP.k_CoM.vel.raw = COM{35,4};
sub07.PSMP.k_CoM.vel.mean = mean(COM{35,4},2,'omitnan');
sub07.PSMP.k_CoM.vel.std = std(COM{35,4},0,2,'omitnan');

sub01.NW.k_CoM.vel.unit = 'm';
sub02.NW.k_CoM.vel.unit = 'm';
sub03.NW.k_CoM.vel.unit = 'm';
sub04.NW.k_CoM.vel.unit = 'm';
sub05.NW.k_CoM.vel.unit = 'm';
sub06.NW.k_CoM.vel.unit = 'm';
sub07.NW.k_CoM.vel.unit = 'm';
sub01.MFMP.k_CoM.vel.unit = 'm';
sub01.PFMP.k_CoM.vel.unit = 'm';
sub01.MSMP.k_CoM.vel.unit = 'm';
sub01.PSMP.k_CoM.vel.unit = 'm';
sub02.MFMP.k_CoM.vel.unit = 'm';
sub02.PFMP.k_CoM.vel.unit = 'm';
sub02.MSMP.k_CoM.vel.unit = 'm';
sub02.PSMP.k_CoM.vel.unit = 'm';
sub03.MFMP.k_CoM.vel.unit = 'm';
sub03.PFMP.k_CoM.vel.unit = 'm';
sub03.MSMP.k_CoM.vel.unit = 'm';
sub03.PSMP.k_CoM.vel.unit = 'm';
sub04.MFMP.k_CoM.vel.unit = 'm';
sub04.PFMP.k_CoM.vel.unit = 'm';
sub04.MSMP.k_CoM.vel.unit = 'm';
sub04.PSMP.k_CoM.vel.unit = 'm';
sub05.MFMP.k_CoM.vel.unit = 'm';
sub05.PFMP.k_CoM.vel.unit = 'm';
sub05.MSMP.k_CoM.vel.unit = 'm';
sub05.PSMP.k_CoM.vel.unit = 'm';
sub06.MFMP.k_CoM.vel.unit = 'm';
sub06.PFMP.k_CoM.vel.unit = 'm';
sub06.MSMP.k_CoM.vel.unit = 'm';
sub06.PSMP.k_CoM.vel.unit = 'm';
sub07.MFMP.k_CoM.vel.unit = 'm';
sub07.PFMP.k_CoM.vel.unit = 'm';
sub07.MSMP.k_CoM.vel.unit = 'm';
sub07.PSMP.k_CoM.vel.unit = 'm';
%% acc

% sub01.NW.k_CoM.acc.raw = COM{1,13};
sub01.NW.k_CoM.acc.mean = mean(COM{1,13},2,'omitnan');
sub01.NW.k_CoM.acc.std = std(COM{1,13},0,2,'omitnan');
% sub01.MFMP.k_CoM.acc.raw = COM{2,13};
sub01.MFMP.k_CoM.acc.mean = mean(COM{2,13},2,'omitnan');
sub01.MFMP.k_CoM.acc.std = std(COM{2,13},0,2,'omitnan');
% sub01.PFMP.k_CoM.acc.raw = COM{3,13};
sub01.PFMP.k_CoM.acc.mean = mean(COM{3,13},2,'omitnan');
sub01.PFMP.k_CoM.acc.std = std(COM{3,13},0,2,'omitnan');
% sub01.MSMP.k_CoM.acc.raw = COM{4,13};
sub01.MSMP.k_CoM.acc.mean = mean(COM{4,13},2,'omitnan');
sub01.MSMP.k_CoM.acc.std = std(COM{4,13},0,2,'omitnan');
% sub01.PSMP.k_CoM.acc.raw = COM{5,13};
sub01.PSMP.k_CoM.acc.mean = mean(COM{5,13},2,'omitnan');
sub01.PSMP.k_CoM.acc.std = std(COM{5,13},0,2,'omitnan');

% sub02.NW.k_CoM.acc.raw = COM{6,13};
sub02.NW.k_CoM.acc.mean = mean(COM{6,13},2,'omitnan');
sub02.NW.k_CoM.acc.std = std(COM{6,13},0,2,'omitnan');
% sub02.MFMP.k_CoM.acc.raw = COM{7,13};
sub02.MFMP.k_CoM.acc.mean = mean(COM{7,13},2,'omitnan');
sub02.MFMP.k_CoM.acc.std = std(COM{7,13},0,2,'omitnan');
% sub02.PFMP.k_CoM.acc.raw = COM{8,13};
sub02.PFMP.k_CoM.acc.mean = mean(COM{8,13},2,'omitnan');
sub02.PFMP.k_CoM.acc.std = std(COM{8,13},0,2,'omitnan');
% sub02.MSMP.k_CoM.acc.raw = COM{9,13};
sub02.MSMP.k_CoM.acc.mean = mean(COM{9,13},2,'omitnan');
sub02.MSMP.k_CoM.acc.std = std(COM{9,13},0,2,'omitnan');
% sub02.PSMP.k_CoM.acc.raw = COM{10,13};
sub02.PSMP.k_CoM.acc.mean = mean(COM{10,13},2,'omitnan');
sub02.PSMP.k_CoM.acc.std = std(COM{10,13},0,2,'omitnan');

% sub03.NW.k_CoM.acc.raw = COM{11,13};
sub03.NW.k_CoM.acc.mean = mean(COM{1,13},2,'omitnan');
sub03.NW.k_CoM.acc.std = std(COM{11,13},0,2,'omitnan');
% sub03.MFMP.k_CoM.acc.raw = COM{12,13};
sub03.MFMP.k_CoM.acc.mean = mean(COM{12,13},2,'omitnan');
sub03.MFMP.k_CoM.acc.std = std(COM{12,13},0,2,'omitnan');
% sub03.PFMP.k_CoM.acc.raw = COM{13,13};
sub03.PFMP.k_CoM.acc.mean = mean(COM{13,13},2,'omitnan');
sub03.PFMP.k_CoM.acc.std = std(COM{13,13},0,2,'omitnan');
% sub03.MSMP.k_CoM.acc.raw = COM{14,13};
sub03.MSMP.k_CoM.acc.mean = mean(COM{14,13},2,'omitnan');
sub03.MSMP.k_CoM.acc.std = std(COM{14,13},0,2,'omitnan');
% sub03.PSMP.k_CoM.acc.raw = COM{15,13};
sub03.PSMP.k_CoM.acc.mean = mean(COM{15,13},2,'omitnan');
sub03.PSMP.k_CoM.acc.std = std(COM{15,13},0,2,'omitnan');

% sub04.NW.k_CoM.acc.raw = COM{16,13};
sub04.NW.k_CoM.acc.mean = mean(COM{16,13},2,'omitnan');
sub04.NW.k_CoM.acc.std = std(COM{16,13},0,2,'omitnan');
% sub04.MFMP.k_CoM.acc.raw = COM{17,13};
sub04.MFMP.k_CoM.acc.mean = mean(COM{17,13},2,'omitnan');
sub04.MFMP.k_CoM.acc.std = std(COM{17,13},0,2,'omitnan');
% sub04.PFMP.k_CoM.acc.raw = COM{18,13};
sub04.PFMP.k_CoM.acc.mean = mean(COM{18,13},2,'omitnan');
sub04.PFMP.k_CoM.acc.std = std(COM{18,13},0,2,'omitnan');
% sub04.MSMP.k_CoM.acc.raw = COM{19,13};
sub04.MSMP.k_CoM.acc.mean = mean(COM{19,13},2,'omitnan');
sub04.MSMP.k_CoM.acc.std = std(COM{19,13},0,2,'omitnan');
% sub04.PSMP.k_CoM.acc.raw = COM{20,13};
sub04.PSMP.k_CoM.acc.mean = mean(COM{20,13},2,'omitnan');
sub04.PSMP.k_CoM.acc.std = std(COM{20,13},0,2,'omitnan');


% sub05.NW.k_CoM.acc.raw = COM{21,13};
sub05.NW.k_CoM.acc.mean = mean(COM{21,13},2,'omitnan');
sub05.NW.k_CoM.acc.std = std(COM{21,13},0,2,'omitnan');
% sub05.MFMP.k_CoM.acc.raw = COM{22,13};
sub05.MFMP.k_CoM.acc.mean = mean(COM{22,13},2,'omitnan');
sub05.MFMP.k_CoM.acc.std = std(COM{22,13},0,2,'omitnan');
% sub05.PFMP.k_CoM.acc.raw = COM{23,13};
sub05.PFMP.k_CoM.acc.mean = mean(COM{23,13},2,'omitnan');
sub05.PFMP.k_CoM.acc.std = std(COM{23,13},0,2,'omitnan');
% sub05.MSMP.k_CoM.acc.raw = COM{24,13};
sub05.MSMP.k_CoM.acc.mean = mean(COM{24,13},2,'omitnan');
sub05.MSMP.k_CoM.acc.std = std(COM{24,13},0,2,'omitnan');
% sub05.PSMP.k_CoM.acc.raw = COM{25,13};
sub05.PSMP.k_CoM.acc.mean = mean(COM{25,13},2,'omitnan');
sub05.PSMP.k_CoM.acc.std = std(COM{25,13},0,2,'omitnan');


% sub06.NW.k_CoM.acc.raw = COM{26,13};
sub06.NW.k_CoM.acc.mean = mean(COM{26,13},2,'omitnan');
sub06.NW.k_CoM.acc.std = std(COM{26,13},0,2,'omitnan');
% sub06.MFMP.k_CoM.acc.raw = COM{27,13};
sub06.MFMP.k_CoM.acc.mean = mean(COM{27,13},2,'omitnan');
sub06.MFMP.k_CoM.acc.std = std(COM{27,13},0,2,'omitnan');
% sub06.PFMP.k_CoM.acc.raw = COM{28,13};
sub06.PFMP.k_CoM.acc.mean = mean(COM{28,13},2,'omitnan');
sub06.PFMP.k_CoM.acc.std = std(COM{28,13},0,2,'omitnan');
% sub06.MSMP.k_CoM.acc.raw = COM{29,13};
sub06.MSMP.k_CoM.acc.mean = mean(COM{29,13},2,'omitnan');
sub06.MSMP.k_CoM.acc.std = std(COM{29,13},0,2,'omitnan');
% sub06.PSMP.k_CoM.acc.raw = COM{30,13};
sub06.PSMP.k_CoM.acc.mean = mean(COM{30,13},2,'omitnan');
sub06.PSMP.k_CoM.acc.std = std(COM{30,13},0,2,'omitnan');


% sub07.NW.k_CoM.acc.raw = COM{31,13};
sub07.NW.k_CoM.acc.mean = mean(COM{31,13},2,'omitnan');
sub07.NW.k_CoM.acc.std = std(COM{31,13},0,2,'omitnan');
% sub07.MFMP.k_CoM.acc.raw = COM{32,13};
sub07.MFMP.k_CoM.acc.mean = mean(COM{32,13},2,'omitnan');
sub07.MFMP.k_CoM.acc.std = std(COM{32,13},0,2,'omitnan');
% sub07.PFMP.k_CoM.acc.raw = COM{33,13};
sub07.PFMP.k_CoM.acc.mean = mean(COM{33,13},2,'omitnan');
sub07.PFMP.k_CoM.acc.std = std(COM{33,13},0,2,'omitnan');
% sub07.MSMP.k_CoM.acc.raw = COM{34,13};
sub07.MSMP.k_CoM.acc.mean = mean(COM{34,13},2,'omitnan');
sub07.MSMP.k_CoM.acc.std = std(COM{34,13},0,2,'omitnan');
% sub07.PSMP.k_CoM.acc.raw = COM{35,13};
sub07.PSMP.k_CoM.acc.mean = mean(COM{35,13},2,'omitnan');
sub07.PSMP.k_CoM.acc.std = std(COM{35,13},0,2,'omitnan');

sub01.NW.k_CoM.acc.unit = 'm';
sub02.NW.k_CoM.acc.unit = 'm';
sub03.NW.k_CoM.acc.unit = 'm';
sub04.NW.k_CoM.acc.unit = 'm';
sub05.NW.k_CoM.acc.unit = 'm';
sub06.NW.k_CoM.acc.unit = 'm';
sub07.NW.k_CoM.acc.unit = 'm';
sub01.MFMP.k_CoM.acc.unit = 'm';
sub01.PFMP.k_CoM.acc.unit = 'm';
sub01.MSMP.k_CoM.acc.unit = 'm';
sub01.PSMP.k_CoM.acc.unit = 'm';
sub02.MFMP.k_CoM.acc.unit = 'm';
sub02.PFMP.k_CoM.acc.unit = 'm';
sub02.MSMP.k_CoM.acc.unit = 'm';
sub02.PSMP.k_CoM.acc.unit = 'm';
sub03.MFMP.k_CoM.acc.unit = 'm';
sub03.PFMP.k_CoM.acc.unit = 'm';
sub03.MSMP.k_CoM.acc.unit = 'm';
sub03.PSMP.k_CoM.acc.unit = 'm';
sub04.MFMP.k_CoM.acc.unit = 'm';
sub04.PFMP.k_CoM.acc.unit = 'm';
sub04.MSMP.k_CoM.acc.unit = 'm';
sub04.PSMP.k_CoM.acc.unit = 'm';
sub05.MFMP.k_CoM.acc.unit = 'm';
sub05.PFMP.k_CoM.acc.unit = 'm';
sub05.MSMP.k_CoM.acc.unit = 'm';
sub05.PSMP.k_CoM.acc.unit = 'm';
sub06.MFMP.k_CoM.acc.unit = 'm';
sub06.PFMP.k_CoM.acc.unit = 'm';
sub06.MSMP.k_CoM.acc.unit = 'm';
sub06.PSMP.k_CoM.acc.unit = 'm';
sub07.MFMP.k_CoM.acc.unit = 'm';
sub07.PFMP.k_CoM.acc.unit = 'm';
sub07.MSMP.k_CoM.acc.unit = 'm';
sub07.PSMP.k_CoM.acc.unit = 'm';

%% balance_related save
%% MOS
% sub01.NW.br_param.mos.raw = BR{1,1};
sub01.NW.br_param.mos.mean = mean(BR{1,1},2,'omitnan');
sub01.NW.br_param.mos.std = std(BR{1,1},0,2,'omitnan');
% sub01.MFMP.br_param.mos.raw = BR{2,1};
sub01.MFMP.br_param.mos.mean = mean(BR{2,1},2,'omitnan');
sub01.MFMP.br_param.mos.std = std(BR{2,1},0,2,'omitnan');
% sub01.PFMP.br_param.mos.raw = BR{3,1};
sub01.PFMP.br_param.mos.mean = mean(BR{3,1},2,'omitnan');
sub01.PFMP.br_param.mos.std = std(BR{3,1},0,2,'omitnan');
% sub01.MSMP.br_param.mos.raw = BR{4,1};
sub01.MSMP.br_param.mos.mean = mean(BR{4,1},2,'omitnan');
sub01.MSMP.br_param.mos.std = std(BR{4,1},0,2,'omitnan');
% sub01.PSMP.br_param.mos.raw = BR{5,1};
sub01.PSMP.br_param.mos.mean = mean(BR{5,1},2,'omitnan');
sub01.PSMP.br_param.mos.std = std(BR{5,1},0,2,'omitnan');

% sub02.NW.br_param.mos.raw = BR{6,1};
sub02.NW.br_param.mos.mean = mean(BR{6,1},2,'omitnan');
sub02.NW.br_param.mos.std = std(BR{6,1},0,2,'omitnan');
% sub02.MFMP.br_param.mos.raw = BR{7,1};
sub02.MFMP.br_param.mos.mean = mean(BR{7,1},2,'omitnan');
sub02.MFMP.br_param.mos.std = std(BR{7,1},0,2,'omitnan');
% sub02.PFMP.br_param.mos.raw = BR{8,1};
sub02.PFMP.br_param.mos.mean = mean(BR{8,1},2,'omitnan');
sub02.PFMP.br_param.mos.std = std(BR{8,1},0,2,'omitnan');
% sub02.MSMP.br_param.mos.raw = BR{9,1};
sub02.MSMP.br_param.mos.mean = mean(BR{9,1},2,'omitnan');
sub02.MSMP.br_param.mos.std = std(BR{9,1},0,2,'omitnan');
% sub02.PSMP.br_param.mos.raw = BR{10,1};
sub02.PSMP.br_param.mos.mean = mean(BR{10,1},2,'omitnan');
sub02.PSMP.br_param.mos.std = std(BR{10,1},0,2,'omitnan');

% sub03.NW.br_param.mos.raw = BR{11,1};
sub03.NW.br_param.mos.mean = mean(BR{1,1},2,'omitnan');
sub03.NW.br_param.mos.std = std(BR{11,1},0,2,'omitnan');
% sub03.MFMP.br_param.mos.raw = BR{12,1};
sub03.MFMP.br_param.mos.mean = mean(BR{12,1},2,'omitnan');
sub03.MFMP.br_param.mos.std = std(BR{12,1},0,2,'omitnan');
% sub03.PFMP.br_param.mos.raw = BR{13,1};
sub03.PFMP.br_param.mos.mean = mean(BR{13,1},2,'omitnan');
sub03.PFMP.br_param.mos.std = std(BR{13,1},0,2,'omitnan');
% sub03.MSMP.br_param.mos.raw = BR{14,1};
sub03.MSMP.br_param.mos.mean = mean(BR{14,1},2,'omitnan');
sub03.MSMP.br_param.mos.std = std(BR{14,1},0,2,'omitnan');
% sub03.PSMP.br_param.mos.raw = BR{15,1};
sub03.PSMP.br_param.mos.mean = mean(BR{15,1},2,'omitnan');
sub03.PSMP.br_param.mos.std = std(BR{15,1},0,2,'omitnan');

% sub04.NW.br_param.mos.raw = BR{16,1};
sub04.NW.br_param.mos.mean = mean(BR{16,1},2,'omitnan');
sub04.NW.br_param.mos.std = std(BR{16,1},0,2,'omitnan');
% sub04.MFMP.br_param.mos.raw = BR{17,1};
sub04.MFMP.br_param.mos.mean = mean(BR{17,1},2,'omitnan');
sub04.MFMP.br_param.mos.std = std(BR{17,1},0,2,'omitnan');
% sub04.PFMP.br_param.mos.raw = BR{18,1};
sub04.PFMP.br_param.mos.mean = mean(BR{18,1},2,'omitnan');
sub04.PFMP.br_param.mos.std = std(BR{18,1},0,2,'omitnan');
% sub04.MSMP.br_param.mos.raw = BR{19,1};
sub04.MSMP.br_param.mos.mean = mean(BR{19,1},2,'omitnan');
sub04.MSMP.br_param.mos.std = std(BR{19,1},0,2,'omitnan');
% sub04.PSMP.br_param.mos.raw = BR{20,1};
sub04.PSMP.br_param.mos.mean = mean(BR{20,1},2,'omitnan');
sub04.PSMP.br_param.mos.std = std(BR{20,1},0,2,'omitnan');


% sub05.NW.br_param.mos.raw = BR{21,1};
sub05.NW.br_param.mos.mean = mean(BR{21,1},2,'omitnan');
sub05.NW.br_param.mos.std = std(BR{21,1},0,2,'omitnan');
% sub05.MFMP.br_param.mos.raw = BR{22,1};
sub05.MFMP.br_param.mos.mean = mean(BR{22,1},2,'omitnan');
sub05.MFMP.br_param.mos.std = std(BR{22,1},0,2,'omitnan');
% sub05.PFMP.br_param.mos.raw = BR{23,1};
sub05.PFMP.br_param.mos.mean = mean(BR{23,1},2,'omitnan');
sub05.PFMP.br_param.mos.std = std(BR{23,1},0,2,'omitnan');
% sub05.MSMP.br_param.mos.raw = BR{24,1};
sub05.MSMP.br_param.mos.mean = mean(BR{24,1},2,'omitnan');
sub05.MSMP.br_param.mos.std = std(BR{24,1},0,2,'omitnan');
% sub05.PSMP.br_param.mos.raw = BR{25,1};
sub05.PSMP.br_param.mos.mean = mean(BR{25,1},2,'omitnan');
sub05.PSMP.br_param.mos.std = std(BR{25,1},0,2,'omitnan');


% sub06.NW.br_param.mos.raw = BR{26,1};
sub06.NW.br_param.mos.mean = mean(BR{26,1},2,'omitnan');
sub06.NW.br_param.mos.std = std(BR{26,1},0,2,'omitnan');
% sub06.MFMP.br_param.mos.raw = BR{27,1};
sub06.MFMP.br_param.mos.mean = mean(BR{27,1},2,'omitnan');
sub06.MFMP.br_param.mos.std = std(BR{27,1},0,2,'omitnan');
% sub06.PFMP.br_param.mos.raw = BR{28,1};
sub06.PFMP.br_param.mos.mean = mean(BR{28,1},2,'omitnan');
sub06.PFMP.br_param.mos.std = std(BR{28,1},0,2,'omitnan');
% sub06.MSMP.br_param.mos.raw = BR{29,1};
sub06.MSMP.br_param.mos.mean = mean(BR{29,1},2,'omitnan');
sub06.MSMP.br_param.mos.std = std(BR{29,1},0,2,'omitnan');
% sub06.PSMP.br_param.mos.raw = BR{30,1};
sub06.PSMP.br_param.mos.mean = mean(BR{30,1},2,'omitnan');
sub06.PSMP.br_param.mos.std = std(BR{30,1},0,2,'omitnan');


% sub07.NW.br_param.mos.raw = BR{31,1};
sub07.NW.br_param.mos.mean = mean(BR{31,1},2,'omitnan');
sub07.NW.br_param.mos.std = std(BR{31,1},0,2,'omitnan');
% sub07.MFMP.br_param.mos.raw = BR{32,1};
sub07.MFMP.br_param.mos.mean = mean(BR{32,1},2,'omitnan');
sub07.MFMP.br_param.mos.std = std(BR{32,1},0,2,'omitnan');
% sub07.PFMP.br_param.mos.raw = BR{33,1};
sub07.PFMP.br_param.mos.mean = mean(BR{33,1},2,'omitnan');
sub07.PFMP.br_param.mos.std = std(BR{33,1},0,2,'omitnan');
% sub07.MSMP.br_param.mos.raw = BR{34,1};
sub07.MSMP.br_param.mos.mean = mean(BR{34,1},2,'omitnan');
sub07.MSMP.br_param.mos.std = std(BR{34,1},0,2,'omitnan');
% sub07.PSMP.br_param.mos.raw = BR{35,1};
sub07.PSMP.br_param.mos.mean = mean(BR{35,1},2,'omitnan');
sub07.PSMP.br_param.mos.std = std(BR{35,1},0,2,'omitnan');

sub01.NW.br_param.mos.unit = 'm';
sub02.NW.br_param.mos.unit = 'm';
sub03.NW.br_param.mos.unit = 'm';
sub04.NW.br_param.mos.unit = 'm';
sub05.NW.br_param.mos.unit = 'm';
sub06.NW.br_param.mos.unit = 'm';
sub07.NW.br_param.mos.unit = 'm';
sub01.MFMP.br_param.mos.unit = 'm';
sub01.PFMP.br_param.mos.unit = 'm';
sub01.MSMP.br_param.mos.unit = 'm';
sub01.PSMP.br_param.mos.unit = 'm';
sub02.MFMP.br_param.mos.unit = 'm';
sub02.PFMP.br_param.mos.unit = 'm';
sub02.MSMP.br_param.mos.unit = 'm';
sub02.PSMP.br_param.mos.unit = 'm';
sub03.MFMP.br_param.mos.unit = 'm';
sub03.PFMP.br_param.mos.unit = 'm';
sub03.MSMP.br_param.mos.unit = 'm';
sub03.PSMP.br_param.mos.unit = 'm';
sub04.MFMP.br_param.mos.unit = 'm';
sub04.PFMP.br_param.mos.unit = 'm';
sub04.MSMP.br_param.mos.unit = 'm';
sub04.PSMP.br_param.mos.unit = 'm';
sub05.MFMP.br_param.mos.unit = 'm';
sub05.PFMP.br_param.mos.unit = 'm';
sub05.MSMP.br_param.mos.unit = 'm';
sub05.PSMP.br_param.mos.unit = 'm';
sub06.MFMP.br_param.mos.unit = 'm';
sub06.PFMP.br_param.mos.unit = 'm';
sub06.MSMP.br_param.mos.unit = 'm';
sub06.PSMP.br_param.mos.unit = 'm';
sub07.MFMP.br_param.mos.unit = 'm';
sub07.PFMP.br_param.mos.unit = 'm';
sub07.MSMP.br_param.mos.unit = 'm';
sub07.PSMP.br_param.mos.unit = 'm';
%% XCOM

% sub01.NW.br_param.xcom.raw = BR{1,2};
sub01.NW.br_param.xcom.mean = mean(BR{1,2},2,'omitnan');
sub01.NW.br_param.xcom.std = std(BR{1,2},0,2,'omitnan');
% sub01.MFMP.br_param.xcom.raw = BR{2,2};
sub01.MFMP.br_param.xcom.mean = mean(BR{2,2},2,'omitnan');
sub01.MFMP.br_param.xcom.std = std(BR{2,2},0,2,'omitnan');
% sub01.PFMP.br_param.xcom.raw = BR{3,2};
sub01.PFMP.br_param.xcom.mean = mean(BR{3,2},2,'omitnan');
sub01.PFMP.br_param.xcom.std = std(BR{3,2},0,2,'omitnan');
% sub01.MSMP.br_param.xcom.raw = BR{4,2};
sub01.MSMP.br_param.xcom.mean = mean(BR{4,2},2,'omitnan');
sub01.MSMP.br_param.xcom.std = std(BR{4,2},0,2,'omitnan');
% sub01.PSMP.br_param.xcom.raw = BR{5,2};
sub01.PSMP.br_param.xcom.mean = mean(BR{5,2},2,'omitnan');
sub01.PSMP.br_param.xcom.std = std(BR{5,2},0,2,'omitnan');

% sub02.NW.br_param.xcom.raw = BR{6,2};
sub02.NW.br_param.xcom.mean = mean(BR{6,2},2,'omitnan');
sub02.NW.br_param.xcom.std = std(BR{6,2},0,2,'omitnan');
% sub02.MFMP.br_param.xcom.raw = BR{7,2};
sub02.MFMP.br_param.xcom.mean = mean(BR{7,2},2,'omitnan');
sub02.MFMP.br_param.xcom.std = std(BR{7,2},0,2,'omitnan');
% sub02.PFMP.br_param.xcom.raw = BR{8,2};
sub02.PFMP.br_param.xcom.mean = mean(BR{8,2},2,'omitnan');
sub02.PFMP.br_param.xcom.std = std(BR{8,2},0,2,'omitnan');
% sub02.MSMP.br_param.xcom.raw = BR{9,2};
sub02.MSMP.br_param.xcom.mean = mean(BR{9,2},2,'omitnan');
sub02.MSMP.br_param.xcom.std = std(BR{9,2},0,2,'omitnan');
% sub02.PSMP.br_param.xcom.raw = BR{10,2};
sub02.PSMP.br_param.xcom.mean = mean(BR{10,2},2,'omitnan');
sub02.PSMP.br_param.xcom.std = std(BR{10,2},0,2,'omitnan');

% sub03.NW.br_param.xcom.raw = BR{11,2};
sub03.NW.br_param.xcom.mean = mean(BR{1,2},2,'omitnan');
sub03.NW.br_param.xcom.std = std(BR{11,2},0,2,'omitnan');
% sub03.MFMP.br_param.xcom.raw = BR{12,2};
sub03.MFMP.br_param.xcom.mean = mean(BR{12,2},2,'omitnan');
sub03.MFMP.br_param.xcom.std = std(BR{12,2},0,2,'omitnan');
% sub03.PFMP.br_param.xcom.raw = BR{13,2};
sub03.PFMP.br_param.xcom.mean = mean(BR{13,2},2,'omitnan');
sub03.PFMP.br_param.xcom.std = std(BR{13,2},0,2,'omitnan');
% sub03.MSMP.br_param.xcom.raw = BR{14,2};
sub03.MSMP.br_param.xcom.mean = mean(BR{14,2},2,'omitnan');
sub03.MSMP.br_param.xcom.std = std(BR{14,2},0,2,'omitnan');
% sub03.PSMP.br_param.xcom.raw = BR{15,2};
sub03.PSMP.br_param.xcom.mean = mean(BR{15,2},2,'omitnan');
sub03.PSMP.br_param.xcom.std = std(BR{15,2},0,2,'omitnan');

% sub04.NW.br_param.xcom.raw = BR{16,2};
sub04.NW.br_param.xcom.mean = mean(BR{16,2},2,'omitnan');
sub04.NW.br_param.xcom.std = std(BR{16,2},0,2,'omitnan');
% sub04.MFMP.br_param.xcom.raw = BR{17,2};
sub04.MFMP.br_param.xcom.mean = mean(BR{17,2},2,'omitnan');
sub04.MFMP.br_param.xcom.std = std(BR{17,2},0,2,'omitnan');
% sub04.PFMP.br_param.xcom.raw = BR{18,2};
sub04.PFMP.br_param.xcom.mean = mean(BR{18,2},2,'omitnan');
sub04.PFMP.br_param.xcom.std = std(BR{18,2},0,2,'omitnan');
% sub04.MSMP.br_param.xcom.raw = BR{19,2};
sub04.MSMP.br_param.xcom.mean = mean(BR{19,2},2,'omitnan');
sub04.MSMP.br_param.xcom.std = std(BR{19,2},0,2,'omitnan');
% sub04.PSMP.br_param.xcom.raw = BR{20,2};
sub04.PSMP.br_param.xcom.mean = mean(BR{20,2},2,'omitnan');
sub04.PSMP.br_param.xcom.std = std(BR{20,2},0,2,'omitnan');


% sub05.NW.br_param.xcom.raw = BR{21,2};
sub05.NW.br_param.xcom.mean = mean(BR{21,2},2,'omitnan');
sub05.NW.br_param.xcom.std = std(BR{21,2},0,2,'omitnan');
% sub05.MFMP.br_param.xcom.raw = BR{22,2};
sub05.MFMP.br_param.xcom.mean = mean(BR{22,2},2,'omitnan');
sub05.MFMP.br_param.xcom.std = std(BR{22,2},0,2,'omitnan');
% sub05.PFMP.br_param.xcom.raw = BR{23,2};
sub05.PFMP.br_param.xcom.mean = mean(BR{23,2},2,'omitnan');
sub05.PFMP.br_param.xcom.std = std(BR{23,2},0,2,'omitnan');
% sub05.MSMP.br_param.xcom.raw = BR{24,2};
sub05.MSMP.br_param.xcom.mean = mean(BR{24,2},2,'omitnan');
sub05.MSMP.br_param.xcom.std = std(BR{24,2},0,2,'omitnan');
% sub05.PSMP.br_param.xcom.raw = BR{25,2};
sub05.PSMP.br_param.xcom.mean = mean(BR{25,2},2,'omitnan');
sub05.PSMP.br_param.xcom.std = std(BR{25,2},0,2,'omitnan');


% sub06.NW.br_param.xcom.raw = BR{26,2};
sub06.NW.br_param.xcom.mean = mean(BR{26,2},2,'omitnan');
sub06.NW.br_param.xcom.std = std(BR{26,2},0,2,'omitnan');
% sub06.MFMP.br_param.xcom.raw = BR{27,2};
sub06.MFMP.br_param.xcom.mean = mean(BR{27,2},2,'omitnan');
sub06.MFMP.br_param.xcom.std = std(BR{27,2},0,2,'omitnan');
% sub06.PFMP.br_param.xcom.raw = BR{28,2};
sub06.PFMP.br_param.xcom.mean = mean(BR{28,2},2,'omitnan');
sub06.PFMP.br_param.xcom.std = std(BR{28,2},0,2,'omitnan');
% sub06.MSMP.br_param.xcom.raw = BR{29,2};
sub06.MSMP.br_param.xcom.mean = mean(BR{29,2},2,'omitnan');
sub06.MSMP.br_param.xcom.std = std(BR{29,2},0,2,'omitnan');
% sub06.PSMP.br_param.xcom.raw = BR{30,2};
sub06.PSMP.br_param.xcom.mean = mean(BR{30,2},2,'omitnan');
sub06.PSMP.br_param.xcom.std = std(BR{30,2},0,2,'omitnan');


% sub07.NW.br_param.xcom.raw = BR{31,2};
sub07.NW.br_param.xcom.mean = mean(BR{31,2},2,'omitnan');
sub07.NW.br_param.xcom.std = std(BR{31,2},0,2,'omitnan');
% sub07.MFMP.br_param.xcom.raw = BR{32,2};
sub07.MFMP.br_param.xcom.mean = mean(BR{32,2},2,'omitnan');
sub07.MFMP.br_param.xcom.std = std(BR{32,2},0,2,'omitnan');
% sub07.PFMP.br_param.xcom.raw = BR{33,2};
sub07.PFMP.br_param.xcom.mean = mean(BR{33,2},2,'omitnan');
sub07.PFMP.br_param.xcom.std = std(BR{33,2},0,2,'omitnan');
% sub07.MSMP.br_param.xcom.raw = BR{34,2};
sub07.MSMP.br_param.xcom.mean = mean(BR{34,2},2,'omitnan');
sub07.MSMP.br_param.xcom.std = std(BR{34,2},0,2,'omitnan');
% sub07.PSMP.br_param.xcom.raw = BR{35,2};
sub07.PSMP.br_param.xcom.mean = mean(BR{35,2},2,'omitnan');
sub07.PSMP.br_param.xcom.std = std(BR{35,2},0,2,'omitnan');

sub01.NW.br_param.xcom.unit = 'm';
sub02.NW.br_param.xcom.unit = 'm';
sub03.NW.br_param.xcom.unit = 'm';
sub04.NW.br_param.xcom.unit = 'm';
sub05.NW.br_param.xcom.unit = 'm';
sub06.NW.br_param.xcom.unit = 'm';
sub07.NW.br_param.xcom.unit = 'm';
sub01.MFMP.br_param.xcom.unit = 'm';
sub01.PFMP.br_param.xcom.unit = 'm';
sub01.MSMP.br_param.xcom.unit = 'm';
sub01.PSMP.br_param.xcom.unit = 'm';
sub02.MFMP.br_param.xcom.unit = 'm';
sub02.PFMP.br_param.xcom.unit = 'm';
sub02.MSMP.br_param.xcom.unit = 'm';
sub02.PSMP.br_param.xcom.unit = 'm';
sub03.MFMP.br_param.xcom.unit = 'm';
sub03.PFMP.br_param.xcom.unit = 'm';
sub03.MSMP.br_param.xcom.unit = 'm';
sub03.PSMP.br_param.xcom.unit = 'm';
sub04.MFMP.br_param.xcom.unit = 'm';
sub04.PFMP.br_param.xcom.unit = 'm';
sub04.MSMP.br_param.xcom.unit = 'm';
sub04.PSMP.br_param.xcom.unit = 'm';
sub05.MFMP.br_param.xcom.unit = 'm';
sub05.PFMP.br_param.xcom.unit = 'm';
sub05.MSMP.br_param.xcom.unit = 'm';
sub05.PSMP.br_param.xcom.unit = 'm';
sub06.MFMP.br_param.xcom.unit = 'm';
sub06.PFMP.br_param.xcom.unit = 'm';
sub06.MSMP.br_param.xcom.unit = 'm';
sub06.PSMP.br_param.xcom.unit = 'm';
sub07.MFMP.br_param.xcom.unit = 'm';
sub07.PFMP.br_param.xcom.unit = 'm';
sub07.MSMP.br_param.xcom.unit = 'm';
sub07.PSMP.br_param.xcom.unit = 'm';
%% BOS

% sub01.NW.br_param.bos.raw = BR{1,3};
sub01.NW.br_param.bos.mean = mean(BR{1,3},2,'omitnan');
sub01.NW.br_param.bos.std = std(BR{1,3},0,2,'omitnan');
% sub01.MFMP.br_param.bos.raw = BR{2,3};
sub01.MFMP.br_param.bos.mean = mean(BR{2,3},2,'omitnan');
sub01.MFMP.br_param.bos.std = std(BR{2,3},0,2,'omitnan');
% sub01.PFMP.br_param.bos.raw = BR{3,3};
sub01.PFMP.br_param.bos.mean = mean(BR{3,3},2,'omitnan');
sub01.PFMP.br_param.bos.std = std(BR{3,3},0,2,'omitnan');
% sub01.MSMP.br_param.bos.raw = BR{4,3};
sub01.MSMP.br_param.bos.mean = mean(BR{4,3},2,'omitnan');
sub01.MSMP.br_param.bos.std = std(BR{4,3},0,2,'omitnan');
% sub01.PSMP.br_param.bos.raw = BR{5,3};
sub01.PSMP.br_param.bos.mean = mean(BR{5,3},2,'omitnan');
sub01.PSMP.br_param.bos.std = std(BR{5,3},0,2,'omitnan');

% sub02.NW.br_param.bos.raw = BR{6,3};
sub02.NW.br_param.bos.mean = mean(BR{6,3},2,'omitnan');
sub02.NW.br_param.bos.std = std(BR{6,3},0,2,'omitnan');
% sub02.MFMP.br_param.bos.raw = BR{7,3};
sub02.MFMP.br_param.bos.mean = mean(BR{7,3},2,'omitnan');
sub02.MFMP.br_param.bos.std = std(BR{7,3},0,2,'omitnan');
% sub02.PFMP.br_param.bos.raw = BR{8,3};
sub02.PFMP.br_param.bos.mean = mean(BR{8,3},2,'omitnan');
sub02.PFMP.br_param.bos.std = std(BR{8,3},0,2,'omitnan');
% sub02.MSMP.br_param.bos.raw = BR{9,3};
sub02.MSMP.br_param.bos.mean = mean(BR{9,3},2,'omitnan');
sub02.MSMP.br_param.bos.std = std(BR{9,3},0,2,'omitnan');
% sub02.PSMP.br_param.bos.raw = BR{10,3};
sub02.PSMP.br_param.bos.mean = mean(BR{10,3},2,'omitnan');
sub02.PSMP.br_param.bos.std = std(BR{10,3},0,2,'omitnan');

% sub03.NW.br_param.bos.raw = BR{11,3};
sub03.NW.br_param.bos.mean = mean(BR{1,3},2,'omitnan');
sub03.NW.br_param.bos.std = std(BR{11,3},0,2,'omitnan');
% sub03.MFMP.br_param.bos.raw = BR{12,3};
sub03.MFMP.br_param.bos.mean = mean(BR{12,3},2,'omitnan');
sub03.MFMP.br_param.bos.std = std(BR{12,3},0,2,'omitnan');
% sub03.PFMP.br_param.bos.raw = BR{13,3};
sub03.PFMP.br_param.bos.mean = mean(BR{13,3},2,'omitnan');
sub03.PFMP.br_param.bos.std = std(BR{13,3},0,2,'omitnan');
% sub03.MSMP.br_param.bos.raw = BR{14,3};
sub03.MSMP.br_param.bos.mean = mean(BR{14,3},2,'omitnan');
sub03.MSMP.br_param.bos.std = std(BR{14,3},0,2,'omitnan');
% sub03.PSMP.br_param.bos.raw = BR{15,3};
sub03.PSMP.br_param.bos.mean = mean(BR{15,3},2,'omitnan');
sub03.PSMP.br_param.bos.std = std(BR{15,3},0,2,'omitnan');

% sub04.NW.br_param.bos.raw = BR{16,3};
sub04.NW.br_param.bos.mean = mean(BR{16,3},2,'omitnan');
sub04.NW.br_param.bos.std = std(BR{16,3},0,2,'omitnan');
% sub04.MFMP.br_param.bos.raw = BR{17,3};
sub04.MFMP.br_param.bos.mean = mean(BR{17,3},2,'omitnan');
sub04.MFMP.br_param.bos.std = std(BR{17,3},0,2,'omitnan');
% sub04.PFMP.br_param.bos.raw = BR{18,3};
sub04.PFMP.br_param.bos.mean = mean(BR{18,3},2,'omitnan');
sub04.PFMP.br_param.bos.std = std(BR{18,3},0,2,'omitnan');
% sub04.MSMP.br_param.bos.raw = BR{19,3};
sub04.MSMP.br_param.bos.mean = mean(BR{19,3},2,'omitnan');
sub04.MSMP.br_param.bos.std = std(BR{19,3},0,2,'omitnan');
% sub04.PSMP.br_param.bos.raw = BR{20,3};
sub04.PSMP.br_param.bos.mean = mean(BR{20,3},2,'omitnan');
sub04.PSMP.br_param.bos.std = std(BR{20,3},0,2,'omitnan');


% sub05.NW.br_param.bos.raw = BR{21,3};
sub05.NW.br_param.bos.mean = mean(BR{21,3},2,'omitnan');
sub05.NW.br_param.bos.std = std(BR{21,3},0,2,'omitnan');
% sub05.MFMP.br_param.bos.raw = BR{22,3};
sub05.MFMP.br_param.bos.mean = mean(BR{22,3},2,'omitnan');
sub05.MFMP.br_param.bos.std = std(BR{22,3},0,2,'omitnan');
% sub05.PFMP.br_param.bos.raw = BR{23,3};
sub05.PFMP.br_param.bos.mean = mean(BR{23,3},2,'omitnan');
sub05.PFMP.br_param.bos.std = std(BR{23,3},0,2,'omitnan');
% sub05.MSMP.br_param.bos.raw = BR{24,3};
sub05.MSMP.br_param.bos.mean = mean(BR{24,3},2,'omitnan');
sub05.MSMP.br_param.bos.std = std(BR{24,3},0,2,'omitnan');
% sub05.PSMP.br_param.bos.raw = BR{25,3};
sub05.PSMP.br_param.bos.mean = mean(BR{25,3},2,'omitnan');
sub05.PSMP.br_param.bos.std = std(BR{25,3},0,2,'omitnan');


% sub06.NW.br_param.bos.raw = BR{26,3};
sub06.NW.br_param.bos.mean = mean(BR{26,3},2,'omitnan');
sub06.NW.br_param.bos.std = std(BR{26,3},0,2,'omitnan');
% sub06.MFMP.br_param.bos.raw = BR{27,3};
sub06.MFMP.br_param.bos.mean = mean(BR{27,3},2,'omitnan');
sub06.MFMP.br_param.bos.std = std(BR{27,3},0,2,'omitnan');
% sub06.PFMP.br_param.bos.raw = BR{28,3};
sub06.PFMP.br_param.bos.mean = mean(BR{28,3},2,'omitnan');
sub06.PFMP.br_param.bos.std = std(BR{28,3},0,2,'omitnan');
% sub06.MSMP.br_param.bos.raw = BR{29,3};
sub06.MSMP.br_param.bos.mean = mean(BR{29,3},2,'omitnan');
sub06.MSMP.br_param.bos.std = std(BR{29,3},0,2,'omitnan');
% sub06.PSMP.br_param.bos.raw = BR{30,3};
sub06.PSMP.br_param.bos.mean = mean(BR{30,3},2,'omitnan');
sub06.PSMP.br_param.bos.std = std(BR{30,3},0,2,'omitnan');


% sub07.NW.br_param.bos.raw = BR{31,3};
sub07.NW.br_param.bos.mean = mean(BR{31,3},2,'omitnan');
sub07.NW.br_param.bos.std = std(BR{31,3},0,2,'omitnan');
% sub07.MFMP.br_param.bos.raw = BR{32,3};
sub07.MFMP.br_param.bos.mean = mean(BR{32,3},2,'omitnan');
sub07.MFMP.br_param.bos.std = std(BR{32,3},0,2,'omitnan');
% sub07.PFMP.br_param.bos.raw = BR{33,3};
sub07.PFMP.br_param.bos.mean = mean(BR{33,3},2,'omitnan');
sub07.PFMP.br_param.bos.std = std(BR{33,3},0,2,'omitnan');
% sub07.MSMP.br_param.bos.raw = BR{34,3};
sub07.MSMP.br_param.bos.mean = mean(BR{34,3},2,'omitnan');
sub07.MSMP.br_param.bos.std = std(BR{34,3},0,2,'omitnan');
% sub07.PSMP.br_param.bos.raw = BR{35,3};
sub07.PSMP.br_param.bos.mean = mean(BR{35,3},2,'omitnan');
sub07.PSMP.br_param.bos.std = std(BR{35,3},0,2,'omitnan');

sub01.NW.br_param.bos.unit = 'm';
sub02.NW.br_param.bos.unit = 'm';
sub03.NW.br_param.bos.unit = 'm';
sub04.NW.br_param.bos.unit = 'm';
sub05.NW.br_param.bos.unit = 'm';
sub06.NW.br_param.bos.unit = 'm';
sub07.NW.br_param.bos.unit = 'm';
sub01.MFMP.br_param.bos.unit = 'm';
sub01.PFMP.br_param.bos.unit = 'm';
sub01.MSMP.br_param.bos.unit = 'm';
sub01.PSMP.br_param.bos.unit = 'm';
sub02.MFMP.br_param.bos.unit = 'm';
sub02.PFMP.br_param.bos.unit = 'm';
sub02.MSMP.br_param.bos.unit = 'm';
sub02.PSMP.br_param.bos.unit = 'm';
sub03.MFMP.br_param.bos.unit = 'm';
sub03.PFMP.br_param.bos.unit = 'm';
sub03.MSMP.br_param.bos.unit = 'm';
sub03.PSMP.br_param.bos.unit = 'm';
sub04.MFMP.br_param.bos.unit = 'm';
sub04.PFMP.br_param.bos.unit = 'm';
sub04.MSMP.br_param.bos.unit = 'm';
sub04.PSMP.br_param.bos.unit = 'm';
sub05.MFMP.br_param.bos.unit = 'm';
sub05.PFMP.br_param.bos.unit = 'm';
sub05.MSMP.br_param.bos.unit = 'm';
sub05.PSMP.br_param.bos.unit = 'm';
sub06.MFMP.br_param.bos.unit = 'm';
sub06.PFMP.br_param.bos.unit = 'm';
sub06.MSMP.br_param.bos.unit = 'm';
sub06.PSMP.br_param.bos.unit = 'm';
sub07.MFMP.br_param.bos.unit = 'm';
sub07.PFMP.br_param.bos.unit = 'm';
sub07.MSMP.br_param.bos.unit = 'm';
sub07.PSMP.br_param.bos.unit = 'm';
%% stepwidth

% sub01.NW.br_param.stepwidth.raw = BR{1,4};
sub01.NW.br_param.stepwidth.mean = mean(BR{1,4});
sub01.NW.br_param.stepwidth.std = std(BR{1,4});
% sub01.MFMP.br_param.stepwidth.raw = BR{2,4};
sub01.MFMP.br_param.stepwidth.mean = mean(BR{2,4});
sub01.MFMP.br_param.stepwidth.std = std(BR{2,4});
% sub01.PFMP.br_param.stepwidth.raw = BR{3,4};
sub01.PFMP.br_param.stepwidth.mean = mean(BR{3,4});
sub01.PFMP.br_param.stepwidth.std = std(BR{3,4});
% sub01.MSMP.br_param.stepwidth.raw = BR{4,4};
sub01.MSMP.br_param.stepwidth.mean = mean(BR{4,4});
sub01.MSMP.br_param.stepwidth.std = std(BR{4,4});
% sub01.PSMP.br_param.stepwidth.raw = BR{5,4};
sub01.PSMP.br_param.stepwidth.mean = mean(BR{5,4});
sub01.PSMP.br_param.stepwidth.std = std(BR{5,4});

% sub02.NW.br_param.stepwidth.raw = BR{6,4};
sub02.NW.br_param.stepwidth.mean = mean(BR{6,4});
sub02.NW.br_param.stepwidth.std = std(BR{6,4});
% sub02.MFMP.br_param.stepwidth.raw = BR{7,4};
sub02.MFMP.br_param.stepwidth.mean = mean(BR{7,4});
sub02.MFMP.br_param.stepwidth.std = std(BR{7,4});
% sub02.PFMP.br_param.stepwidth.raw = BR{8,4};
sub02.PFMP.br_param.stepwidth.mean = mean(BR{8,4});
sub02.PFMP.br_param.stepwidth.std = std(BR{8,4});
% sub02.MSMP.br_param.stepwidth.raw = BR{9,4};
sub02.MSMP.br_param.stepwidth.mean = mean(BR{9,4});
sub02.MSMP.br_param.stepwidth.std = std(BR{9,4});
% sub02.PSMP.br_param.stepwidth.raw = BR{10,4};
sub02.PSMP.br_param.stepwidth.mean = mean(BR{10,4});
sub02.PSMP.br_param.stepwidth.std = std(BR{10,4});

% sub03.NW.br_param.stepwidth.raw = BR{11,4};
sub03.NW.br_param.stepwidth.mean = mean(BR{1,4});
sub03.NW.br_param.stepwidth.std = std(BR{11,4});
% sub03.MFMP.br_param.stepwidth.raw = BR{12,4};
sub03.MFMP.br_param.stepwidth.mean = mean(BR{12,4});
sub03.MFMP.br_param.stepwidth.std = std(BR{12,4});
% sub03.PFMP.br_param.stepwidth.raw = BR{13,4};
sub03.PFMP.br_param.stepwidth.mean = mean(BR{13,4});
sub03.PFMP.br_param.stepwidth.std = std(BR{13,4});
% sub03.MSMP.br_param.stepwidth.raw = BR{14,4};
sub03.MSMP.br_param.stepwidth.mean = mean(BR{14,4});
sub03.MSMP.br_param.stepwidth.std = std(BR{14,4});
% sub03.PSMP.br_param.stepwidth.raw = BR{15,4};
sub03.PSMP.br_param.stepwidth.mean = mean(BR{15,4});
sub03.PSMP.br_param.stepwidth.std = std(BR{15,4});

% sub04.NW.br_param.stepwidth.raw = BR{16,4};
sub04.NW.br_param.stepwidth.mean = mean(BR{16,4});
sub04.NW.br_param.stepwidth.std = std(BR{16,4});
% sub04.MFMP.br_param.stepwidth.raw = BR{17,4};
sub04.MFMP.br_param.stepwidth.mean = mean(BR{17,4});
sub04.MFMP.br_param.stepwidth.std = std(BR{17,4});
% sub04.PFMP.br_param.stepwidth.raw = BR{18,4};
sub04.PFMP.br_param.stepwidth.mean = mean(BR{18,4});
sub04.PFMP.br_param.stepwidth.std = std(BR{18,4});
% sub04.MSMP.br_param.stepwidth.raw = BR{19,4};
sub04.MSMP.br_param.stepwidth.mean = mean(BR{19,4});
sub04.MSMP.br_param.stepwidth.std = std(BR{19,4});
% sub04.PSMP.br_param.stepwidth.raw = BR{20,4};
sub04.PSMP.br_param.stepwidth.mean = mean(BR{20,4});
sub04.PSMP.br_param.stepwidth.std = std(BR{20,4});


% sub05.NW.br_param.stepwidth.raw = BR{21,4};
sub05.NW.br_param.stepwidth.mean = mean(BR{21,4});
sub05.NW.br_param.stepwidth.std = std(BR{21,4});
% sub05.MFMP.br_param.stepwidth.raw = BR{22,4};
sub05.MFMP.br_param.stepwidth.mean = mean(BR{22,4});
sub05.MFMP.br_param.stepwidth.std = std(BR{22,4});
% sub05.PFMP.br_param.stepwidth.raw = BR{23,4};
sub05.PFMP.br_param.stepwidth.mean = mean(BR{23,4});
sub05.PFMP.br_param.stepwidth.std = std(BR{23,4});
% sub05.MSMP.br_param.stepwidth.raw = BR{24,4};
sub05.MSMP.br_param.stepwidth.mean = mean(BR{24,4});
sub05.MSMP.br_param.stepwidth.std = std(BR{24,4});
% sub05.PSMP.br_param.stepwidth.raw = BR{25,4};
sub05.PSMP.br_param.stepwidth.mean = mean(BR{25,4});
sub05.PSMP.br_param.stepwidth.std = std(BR{25,4});


% sub06.NW.br_param.stepwidth.raw = BR{26,4};
sub06.NW.br_param.stepwidth.mean = mean(BR{26,4});
sub06.NW.br_param.stepwidth.std = std(BR{26,4});
% sub06.MFMP.br_param.stepwidth.raw = BR{27,4};
sub06.MFMP.br_param.stepwidth.mean = mean(BR{27,4});
sub06.MFMP.br_param.stepwidth.std = std(BR{27,4});
% sub06.PFMP.br_param.stepwidth.raw = BR{28,4};
sub06.PFMP.br_param.stepwidth.mean = mean(BR{28,4});
sub06.PFMP.br_param.stepwidth.std = std(BR{28,4});
% sub06.MSMP.br_param.stepwidth.raw = BR{29,4};
sub06.MSMP.br_param.stepwidth.mean = mean(BR{29,4});
sub06.MSMP.br_param.stepwidth.std = std(BR{29,4});
% sub06.PSMP.br_param.stepwidth.raw = BR{30,4};
sub06.PSMP.br_param.stepwidth.mean = mean(BR{30,4});
sub06.PSMP.br_param.stepwidth.std = std(BR{30,4});


% sub07.NW.br_param.stepwidth.raw = BR{31,4};
sub07.NW.br_param.stepwidth.mean = mean(BR{31,4});
sub07.NW.br_param.stepwidth.std = std(BR{31,4});
% sub07.MFMP.br_param.stepwidth.raw = BR{32,4};
sub07.MFMP.br_param.stepwidth.mean = mean(BR{32,4});
sub07.MFMP.br_param.stepwidth.std = std(BR{32,4});
% sub07.PFMP.br_param.stepwidth.raw = BR{33,4};
sub07.PFMP.br_param.stepwidth.mean = mean(BR{33,4});
sub07.PFMP.br_param.stepwidth.std = std(BR{33,4});
% sub07.MSMP.br_param.stepwidth.raw = BR{34,4};
sub07.MSMP.br_param.stepwidth.mean = mean(BR{34,4});
sub07.MSMP.br_param.stepwidth.std = std(BR{34,4});
% sub07.PSMP.br_param.stepwidth.raw = BR{35,4};
sub07.PSMP.br_param.stepwidth.mean = mean(BR{35,4});
sub07.PSMP.br_param.stepwidth.std = std(BR{35,4});

sub01.NW.br_param.stepwidth.unit = 'm';
sub02.NW.br_param.stepwidth.unit = 'm';
sub03.NW.br_param.stepwidth.unit = 'm';
sub04.NW.br_param.stepwidth.unit = 'm';
sub05.NW.br_param.stepwidth.unit = 'm';
sub06.NW.br_param.stepwidth.unit = 'm';
sub07.NW.br_param.stepwidth.unit = 'm';
sub01.MFMP.br_param.stepwidth.unit = 'm';
sub01.PFMP.br_param.stepwidth.unit = 'm';
sub01.MSMP.br_param.stepwidth.unit = 'm';
sub01.PSMP.br_param.stepwidth.unit = 'm';
sub02.MFMP.br_param.stepwidth.unit = 'm';
sub02.PFMP.br_param.stepwidth.unit = 'm';
sub02.MSMP.br_param.stepwidth.unit = 'm';
sub02.PSMP.br_param.stepwidth.unit = 'm';
sub03.MFMP.br_param.stepwidth.unit = 'm';
sub03.PFMP.br_param.stepwidth.unit = 'm';
sub03.MSMP.br_param.stepwidth.unit = 'm';
sub03.PSMP.br_param.stepwidth.unit = 'm';
sub04.MFMP.br_param.stepwidth.unit = 'm';
sub04.PFMP.br_param.stepwidth.unit = 'm';
sub04.MSMP.br_param.stepwidth.unit = 'm';
sub04.PSMP.br_param.stepwidth.unit = 'm';
sub05.MFMP.br_param.stepwidth.unit = 'm';
sub05.PFMP.br_param.stepwidth.unit = 'm';
sub05.MSMP.br_param.stepwidth.unit = 'm';
sub05.PSMP.br_param.stepwidth.unit = 'm';
sub06.MFMP.br_param.stepwidth.unit = 'm';
sub06.PFMP.br_param.stepwidth.unit = 'm';
sub06.MSMP.br_param.stepwidth.unit = 'm';
sub06.PSMP.br_param.stepwidth.unit = 'm';
sub07.MFMP.br_param.stepwidth.unit = 'm';
sub07.PFMP.br_param.stepwidth.unit = 'm';
sub07.MSMP.br_param.stepwidth.unit = 'm';
sub07.PSMP.br_param.stepwidth.unit = 'm';

%% COP

% sub01.NW.br_param.CoP_variability.raw = T_cop{1,3}(:,2);
sub01.NW.br_param.CoP_variability.mean = mean(mean(T_cop{1,3}(:,2),2,'omitnan'));
sub01.NW.br_param.CoP_variability.std = std(T_cop{1,3}(:,2),'omitnan');
% sub01.MFMP.br_param.CoP_variability.raw = T_cop{2,3}(:,2);
sub01.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{2,3}(:,2),2,'omitnan'));
sub01.MFMP.br_param.CoP_variability.std = std(T_cop{2,3}(:,2),'omitnan');
% sub01.PFMP.br_param.CoP_variability.raw = T_cop{3,3}(:,2);
sub01.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{3,3}(:,2),2,'omitnan'));
sub01.PFMP.br_param.CoP_variability.std = std(T_cop{3,3}(:,2),'omitnan');
% sub01.MSMP.br_param.CoP_variability.raw = T_cop{4,3}(:,2);
sub01.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{4,3}(:,2),2,'omitnan'));
sub01.MSMP.br_param.CoP_variability.std = std(T_cop{4,3}(:,2),'omitnan');
% sub01.PSMP.br_param.CoP_variability.raw = T_cop{5,3}(:,2);
sub01.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{5,3}(:,2),2,'omitnan'));
sub01.PSMP.br_param.CoP_variability.std = std(T_cop{5,3}(:,2),'omitnan');

% sub02.NW.br_param.CoP_variability.raw = T_cop{6,3}(:,2);
sub02.NW.br_param.CoP_variability.mean = mean(mean(T_cop{6,3}(:,2),2,'omitnan'));
sub02.NW.br_param.CoP_variability.std = std(T_cop{6,3}(:,2),'omitnan');
% sub02.MFMP.br_param.CoP_variability.raw = T_cop{7,3}(:,2);
sub02.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{7,3}(:,2),2,'omitnan'));
sub02.MFMP.br_param.CoP_variability.std = std(T_cop{7,3}(:,2),'omitnan');
% sub02.PFMP.br_param.CoP_variability.raw = T_cop{8,3}(:,2);
sub02.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{8,3}(:,2),2,'omitnan'));
sub02.PFMP.br_param.CoP_variability.std = std(T_cop{8,3}(:,2),'omitnan');
% sub02.MSMP.br_param.CoP_variability.raw = T_cop{9,3}(:,2);
sub02.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{9,3}(:,2),2,'omitnan'));
sub02.MSMP.br_param.CoP_variability.std = std(T_cop{9,3}(:,2),'omitnan');
% sub02.PSMP.br_param.CoP_variability.raw = T_cop{10,3}(:,2);
sub02.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{10,3}(:,2),2,'omitnan'));
sub02.PSMP.br_param.CoP_variability.std = std(T_cop{10,3}(:,2),'omitnan');

% sub03.NW.br_param.CoP_variability.raw = T_cop{11,3}(:,2);
sub03.NW.br_param.CoP_variability.mean = mean(mean(T_cop{1,3}(:,2),2,'omitnan'));
sub03.NW.br_param.CoP_variability.std = std(T_cop{11,3}(:,2),'omitnan');
% sub03.MFMP.br_param.CoP_variability.raw = T_cop{12,3}(:,2);
sub03.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{12,3}(:,2),2,'omitnan'));
sub03.MFMP.br_param.CoP_variability.std = std(T_cop{12,3}(:,2),'omitnan');
% sub03.PFMP.br_param.CoP_variability.raw = T_cop{13,3}(:,2);
sub03.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{13,3}(:,2),2,'omitnan'));
sub03.PFMP.br_param.CoP_variability.std = std(T_cop{13,3}(:,2),'omitnan');
% sub03.MSMP.br_param.CoP_variability.raw = T_cop{14,3}(:,2);
sub03.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{14,3}(:,2),2,'omitnan'));
sub03.MSMP.br_param.CoP_variability.std = std(T_cop{14,3}(:,2),'omitnan');
% sub03.PSMP.br_param.CoP_variability.raw = T_cop{15,3}(:,2);
sub03.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{15,3}(:,2),2,'omitnan'));
sub03.PSMP.br_param.CoP_variability.std = std(T_cop{15,3}(:,2),'omitnan');

% sub04.NW.br_param.CoP_variability.raw = T_cop{16,3}(:,2);
sub04.NW.br_param.CoP_variability.mean = mean(mean(T_cop{16,3}(:,2),2,'omitnan'));
sub04.NW.br_param.CoP_variability.std = std(T_cop{16,3}(:,2),'omitnan');
% sub04.MFMP.br_param.CoP_variability.raw = T_cop{17,3}(:,2);
sub04.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{17,3}(:,2),2,'omitnan'));
sub04.MFMP.br_param.CoP_variability.std = std(T_cop{17,3}(:,2),'omitnan');
% sub04.PFMP.br_param.CoP_variability.raw = T_cop{18,3}(:,2);
sub04.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{18,3}(:,2),2,'omitnan'));
sub04.PFMP.br_param.CoP_variability.std = std(T_cop{18,3}(:,2),'omitnan');
% sub04.MSMP.br_param.CoP_variability.raw = T_cop{19,3}(:,2);
sub04.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{19,3}(:,2),2,'omitnan'));
sub04.MSMP.br_param.CoP_variability.std = std(T_cop{19,3}(:,2),'omitnan');
% sub04.PSMP.br_param.CoP_variability.raw = T_cop{20,3}(:,2);
sub04.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{20,3}(:,2),2,'omitnan'));
sub04.PSMP.br_param.CoP_variability.std = std(T_cop{20,3}(:,2),'omitnan');


sub05.NW.br_param.CoP_variability.raw = T_cop{21,3}(:,2);
sub05.NW.br_param.CoP_variability.mean = mean(mean(T_cop{21,3}(:,2),2,'omitnan'));
sub05.NW.br_param.CoP_variability.std = std(T_cop{21,3}(:,2),'omitnan');
sub05.MFMP.br_param.CoP_variability.raw = T_cop{22,3}(:,2);
sub05.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{22,3}(:,2),2,'omitnan'));
sub05.MFMP.br_param.CoP_variability.std = std(T_cop{22,3}(:,2),'omitnan');
sub05.PFMP.br_param.CoP_variability.raw = T_cop{23,3}(:,2);
sub05.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{23,3}(:,2),2,'omitnan'));
sub05.PFMP.br_param.CoP_variability.std = std(T_cop{23,3}(:,2),'omitnan');
sub05.MSMP.br_param.CoP_variability.raw = T_cop{24,3}(:,2);
sub05.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{24,3}(:,2),2,'omitnan'));
sub05.MSMP.br_param.CoP_variability.std = std(T_cop{24,3}(:,2),'omitnan');
sub05.PSMP.br_param.CoP_variability.raw = T_cop{25,3}(:,2);
sub05.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{25,3}(:,2),2,'omitnan'));
sub05.PSMP.br_param.CoP_variability.std = std(T_cop{25,3}(:,2),'omitnan');


sub06.NW.br_param.CoP_variability.raw = T_cop{26,3}(:,2);
sub06.NW.br_param.CoP_variability.mean = mean(mean(T_cop{26,3}(:,2),2,'omitnan'));
sub06.NW.br_param.CoP_variability.std = std(T_cop{26,3}(:,2),'omitnan');
sub06.MFMP.br_param.CoP_variability.raw = T_cop{27,3}(:,2);
sub06.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{27,3}(:,2),2,'omitnan'));
sub06.MFMP.br_param.CoP_variability.std = std(T_cop{27,3}(:,2),'omitnan');
sub06.PFMP.br_param.CoP_variability.raw = T_cop{28,3}(:,2);
sub06.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{28,3}(:,2),2,'omitnan'));
sub06.PFMP.br_param.CoP_variability.std = std(T_cop{28,3}(:,2),'omitnan');
sub06.MSMP.br_param.CoP_variability.raw = T_cop{29,3}(:,2);
sub06.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{29,3}(:,2),2,'omitnan'));
sub06.MSMP.br_param.CoP_variability.std = std(T_cop{29,3}(:,2),'omitnan');
sub06.PSMP.br_param.CoP_variability.raw = T_cop{30,3}(:,2);
sub06.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{30,3}(:,2),2,'omitnan'));
sub06.PSMP.br_param.CoP_variability.std = std(T_cop{30,3}(:,2),'omitnan');


sub07.NW.br_param.CoP_variability.raw = T_cop{31,3}(:,2);
sub07.NW.br_param.CoP_variability.mean = mean(mean(T_cop{31,3}(:,2),2,'omitnan'));
sub07.NW.br_param.CoP_variability.std = std(T_cop{31,3}(:,2),'omitnan');
sub07.MFMP.br_param.CoP_variability.raw = T_cop{32,3}(:,2);
sub07.MFMP.br_param.CoP_variability.mean = mean(mean(T_cop{32,3}(:,2),2,'omitnan'));
sub07.MFMP.br_param.CoP_variability.std = std(T_cop{32,3}(:,2),'omitnan');
sub07.PFMP.br_param.CoP_variability.raw = T_cop{33,3}(:,2);
sub07.PFMP.br_param.CoP_variability.mean = mean(mean(T_cop{33,3}(:,2),2,'omitnan'));
sub07.PFMP.br_param.CoP_variability.std = std(T_cop{33,3}(:,2),'omitnan');
sub07.MSMP.br_param.CoP_variability.raw = T_cop{34,3}(:,2);
sub07.MSMP.br_param.CoP_variability.mean = mean(mean(T_cop{34,3}(:,2),2,'omitnan'));
sub07.MSMP.br_param.CoP_variability.std = std(T_cop{34,3}(:,2),'omitnan');
sub07.PSMP.br_param.CoP_variability.raw = T_cop{35,3}(:,2);
sub07.PSMP.br_param.CoP_variability.mean = mean(mean(T_cop{35,3}(:,2),2,'omitnan'));
sub07.PSMP.br_param.CoP_variability.std = std(T_cop{35,3}(:,2),'omitnan');

sub01.NW.br_param.CoP_variability.unit = 'mm';
sub02.NW.br_param.CoP_variability.unit = 'mm';
sub03.NW.br_param.CoP_variability.unit = 'mm';
sub04.NW.br_param.CoP_variability.unit = 'mm';
sub05.NW.br_param.CoP_variability.unit = 'mm';
sub06.NW.br_param.CoP_variability.unit = 'mm';
sub07.NW.br_param.CoP_variability.unit = 'mm';
sub01.MFMP.br_param.CoP_variability.unit = 'mm';
sub01.PFMP.br_param.CoP_variability.unit = 'mm';
sub01.MSMP.br_param.CoP_variability.unit = 'mm';
sub01.PSMP.br_param.CoP_variability.unit = 'mm';
sub02.MFMP.br_param.CoP_variability.unit = 'mm';
sub02.PFMP.br_param.CoP_variability.unit = 'mm';
sub02.MSMP.br_param.CoP_variability.unit = 'mm';
sub02.PSMP.br_param.CoP_variability.unit = 'mm';
sub03.MFMP.br_param.CoP_variability.unit = 'mm';
sub03.PFMP.br_param.CoP_variability.unit = 'mm';
sub03.MSMP.br_param.CoP_variability.unit = 'mm';
sub03.PSMP.br_param.CoP_variability.unit = 'mm';
sub04.MFMP.br_param.CoP_variability.unit = 'mm';
sub04.PFMP.br_param.CoP_variability.unit = 'mm';
sub04.MSMP.br_param.CoP_variability.unit = 'mm';
sub04.PSMP.br_param.CoP_variability.unit = 'mm';
sub05.MFMP.br_param.CoP_variability.unit = 'mm';
sub05.PFMP.br_param.CoP_variability.unit = 'mm';
sub05.MSMP.br_param.CoP_variability.unit = 'mm';
sub05.PSMP.br_param.CoP_variability.unit = 'mm';
sub06.MFMP.br_param.CoP_variability.unit = 'mm';
sub06.PFMP.br_param.CoP_variability.unit = 'mm';
sub06.MSMP.br_param.CoP_variability.unit = 'mm';
sub06.PSMP.br_param.CoP_variability.unit = 'mm';
sub07.MFMP.br_param.CoP_variability.unit = 'mm';
sub07.PFMP.br_param.CoP_variability.unit = 'mm';
sub07.MSMP.br_param.CoP_variability.unit = 'mm';
sub07.PSMP.br_param.CoP_variability.unit = 'mm';

%% ST save
%% stlength

% sub01.NW.st_param.stridelength.raw = s_stp{1,1};
sub01.NW.st_param.stridelength.mean = mean(s_stp{1,1});
sub01.NW.st_param.stridelength.std = std(s_stp{1,1});
% sub01.MFMP.st_param.stridelength.raw = s_stp{2,1};
sub01.MFMP.st_param.stridelength.mean = mean(s_stp{2,1});
sub01.MFMP.st_param.stridelength.std = std(s_stp{2,1});
% sub01.PFMP.st_param.stridelength.raw = s_stp{3,1};
sub01.PFMP.st_param.stridelength.mean = mean(s_stp{3,1});
sub01.PFMP.st_param.stridelength.std = std(s_stp{3,1});
% sub01.MSMP.st_param.stridelength.raw = s_stp{4,1};
sub01.MSMP.st_param.stridelength.mean = mean(s_stp{4,1});
sub01.MSMP.st_param.stridelength.std = std(s_stp{4,1});
% sub01.PSMP.st_param.stridelength.raw = s_stp{5,1};
sub01.PSMP.st_param.stridelength.mean = mean(s_stp{5,1});
sub01.PSMP.st_param.stridelength.std = std(s_stp{5,1});

% sub02.NW.st_param.stridelength.raw = s_stp{6,1};
sub02.NW.st_param.stridelength.mean = mean(s_stp{6,1});
sub02.NW.st_param.stridelength.std = std(s_stp{6,1});
% sub02.MFMP.st_param.stridelength.raw = s_stp{7,1};
sub02.MFMP.st_param.stridelength.mean = mean(s_stp{7,1});
sub02.MFMP.st_param.stridelength.std = std(s_stp{7,1});
% sub02.PFMP.st_param.stridelength.raw = s_stp{8,1};
sub02.PFMP.st_param.stridelength.mean = mean(s_stp{8,1});
sub02.PFMP.st_param.stridelength.std = std(s_stp{8,1});
% sub02.MSMP.st_param.stridelength.raw = s_stp{9,1};
sub02.MSMP.st_param.stridelength.mean = mean(s_stp{9,1});
sub02.MSMP.st_param.stridelength.std = std(s_stp{9,1});
% sub02.PSMP.st_param.stridelength.raw = s_stp{10,1};
sub02.PSMP.st_param.stridelength.mean = mean(s_stp{10,1});
sub02.PSMP.st_param.stridelength.std = std(s_stp{10,1});

% sub03.NW.st_param.stridelength.raw = s_stp{11,1};
sub03.NW.st_param.stridelength.mean = mean(s_stp{1,1});
sub03.NW.st_param.stridelength.std = std(s_stp{11,1});
% sub03.MFMP.st_param.stridelength.raw = s_stp{12,1};
sub03.MFMP.st_param.stridelength.mean = mean(s_stp{12,1});
sub03.MFMP.st_param.stridelength.std = std(s_stp{12,1});
% sub03.PFMP.st_param.stridelength.raw = s_stp{13,1};
sub03.PFMP.st_param.stridelength.mean = mean(s_stp{13,1});
sub03.PFMP.st_param.stridelength.std = std(s_stp{13,1});
% sub03.MSMP.st_param.stridelength.raw = s_stp{14,1};
sub03.MSMP.st_param.stridelength.mean = mean(s_stp{14,1});
sub03.MSMP.st_param.stridelength.std = std(s_stp{14,1});
% sub03.PSMP.st_param.stridelength.raw = s_stp{15,1};
sub03.PSMP.st_param.stridelength.mean = mean(s_stp{15,1});
sub03.PSMP.st_param.stridelength.std = std(s_stp{15,1});

% sub04.NW.st_param.stridelength.raw = s_stp{16,1};
sub04.NW.st_param.stridelength.mean = mean(s_stp{16,1});
sub04.NW.st_param.stridelength.std = std(s_stp{16,1});
% sub04.MFMP.st_param.stridelength.raw = s_stp{17,1};
sub04.MFMP.st_param.stridelength.mean = mean(s_stp{17,1});
sub04.MFMP.st_param.stridelength.std = std(s_stp{17,1});
% sub04.PFMP.st_param.stridelength.raw = s_stp{18,1};
sub04.PFMP.st_param.stridelength.mean = mean(s_stp{18,1});
sub04.PFMP.st_param.stridelength.std = std(s_stp{18,1});
% sub04.MSMP.st_param.stridelength.raw = s_stp{19,1};
sub04.MSMP.st_param.stridelength.mean = mean(s_stp{19,1});
sub04.MSMP.st_param.stridelength.std = std(s_stp{19,1});
% sub04.PSMP.st_param.stridelength.raw = s_stp{20,1};
sub04.PSMP.st_param.stridelength.mean = mean(s_stp{20,1});
sub04.PSMP.st_param.stridelength.std = std(s_stp{20,1});


% sub05.NW.st_param.stridelength.raw = s_stp{21,1};
sub05.NW.st_param.stridelength.mean = mean(s_stp{21,1});
sub05.NW.st_param.stridelength.std = std(s_stp{21,1});
% sub05.MFMP.st_param.stridelength.raw = s_stp{22,1};
sub05.MFMP.st_param.stridelength.mean = mean(s_stp{22,1});
sub05.MFMP.st_param.stridelength.std = std(s_stp{22,1});
% sub05.PFMP.st_param.stridelength.raw = s_stp{23,1};
sub05.PFMP.st_param.stridelength.mean = mean(s_stp{23,1});
sub05.PFMP.st_param.stridelength.std = std(s_stp{23,1});
% sub05.MSMP.st_param.stridelength.raw = s_stp{24,1};
sub05.MSMP.st_param.stridelength.mean = mean(s_stp{24,1});
sub05.MSMP.st_param.stridelength.std = std(s_stp{24,1});
% sub05.PSMP.st_param.stridelength.raw = s_stp{25,1};
sub05.PSMP.st_param.stridelength.mean = mean(s_stp{25,1});
sub05.PSMP.st_param.stridelength.std = std(s_stp{25,1});


% sub06.NW.st_param.stridelength.raw = s_stp{26,1};
sub06.NW.st_param.stridelength.mean = mean(s_stp{26,1});
sub06.NW.st_param.stridelength.std = std(s_stp{26,1});
% sub06.MFMP.st_param.stridelength.raw = s_stp{27,1};
sub06.MFMP.st_param.stridelength.mean = mean(s_stp{27,1});
sub06.MFMP.st_param.stridelength.std = std(s_stp{27,1});
% sub06.PFMP.st_param.stridelength.raw = s_stp{28,1};
sub06.PFMP.st_param.stridelength.mean = mean(s_stp{28,1});
sub06.PFMP.st_param.stridelength.std = std(s_stp{28,1});
% sub06.MSMP.st_param.stridelength.raw = s_stp{29,1};
sub06.MSMP.st_param.stridelength.mean = mean(s_stp{29,1});
sub06.MSMP.st_param.stridelength.std = std(s_stp{29,1});
% sub06.PSMP.st_param.stridelength.raw = s_stp{30,1};
sub06.PSMP.st_param.stridelength.mean = mean(s_stp{30,1});
sub06.PSMP.st_param.stridelength.std = std(s_stp{30,1});


% sub07.NW.st_param.stridelength.raw = s_stp{31,1};
sub07.NW.st_param.stridelength.mean = mean(s_stp{31,1});
sub07.NW.st_param.stridelength.std = std(s_stp{31,1});
% sub07.MFMP.st_param.stridelength.raw = s_stp{32,1};
sub07.MFMP.st_param.stridelength.mean = mean(s_stp{32,1});
sub07.MFMP.st_param.stridelength.std = std(s_stp{32,1});
% sub07.PFMP.st_param.stridelength.raw = s_stp{33,1};
sub07.PFMP.st_param.stridelength.mean = mean(s_stp{33,1});
sub07.PFMP.st_param.stridelength.std = std(s_stp{33,1});
% sub07.MSMP.st_param.stridelength.raw = s_stp{34,1};
sub07.MSMP.st_param.stridelength.mean = mean(s_stp{34,1});
sub07.MSMP.st_param.stridelength.std = std(s_stp{34,1});
% sub07.PSMP.st_param.stridelength.raw = s_stp{35,1};
sub07.PSMP.st_param.stridelength.mean = mean(s_stp{35,1});
sub07.PSMP.st_param.stridelength.std = std(s_stp{35,1});

sub01.NW.st_param.stridelength.unit = 'm';
sub02.NW.st_param.stridelength.unit = 'm';
sub03.NW.st_param.stridelength.unit = 'm';
sub04.NW.st_param.stridelength.unit = 'm';
sub05.NW.st_param.stridelength.unit = 'm';
sub06.NW.st_param.stridelength.unit = 'm';
sub07.NW.st_param.stridelength.unit = 'm';
sub01.MFMP.st_param.stridelength.unit = 'm';
sub01.PFMP.st_param.stridelength.unit = 'm';
sub01.MSMP.st_param.stridelength.unit = 'm';
sub01.PSMP.st_param.stridelength.unit = 'm';
sub02.MFMP.st_param.stridelength.unit = 'm';
sub02.PFMP.st_param.stridelength.unit = 'm';
sub02.MSMP.st_param.stridelength.unit = 'm';
sub02.PSMP.st_param.stridelength.unit = 'm';
sub03.MFMP.st_param.stridelength.unit = 'm';
sub03.PFMP.st_param.stridelength.unit = 'm';
sub03.MSMP.st_param.stridelength.unit = 'm';
sub03.PSMP.st_param.stridelength.unit = 'm';
sub04.MFMP.st_param.stridelength.unit = 'm';
sub04.PFMP.st_param.stridelength.unit = 'm';
sub04.MSMP.st_param.stridelength.unit = 'm';
sub04.PSMP.st_param.stridelength.unit = 'm';
sub05.MFMP.st_param.stridelength.unit = 'm';
sub05.PFMP.st_param.stridelength.unit = 'm';
sub05.MSMP.st_param.stridelength.unit = 'm';
sub05.PSMP.st_param.stridelength.unit = 'm';
sub06.MFMP.st_param.stridelength.unit = 'm';
sub06.PFMP.st_param.stridelength.unit = 'm';
sub06.MSMP.st_param.stridelength.unit = 'm';
sub06.PSMP.st_param.stridelength.unit = 'm';
sub07.MFMP.st_param.stridelength.unit = 'm';
sub07.PFMP.st_param.stridelength.unit = 'm';
sub07.MSMP.st_param.stridelength.unit = 'm';
sub07.PSMP.st_param.stridelength.unit = 'm';
%% cad

% sub01.NW.st_param.cadence.raw = s_stp{1,2};
sub01.NW.st_param.cadence.mean = mean(s_stp{1,2});
sub01.NW.st_param.cadence.std = std(s_stp{1,2});
% sub01.MFMP.st_param.cadence.raw = s_stp{2,2};
sub01.MFMP.st_param.cadence.mean = mean(s_stp{2,2});
sub01.MFMP.st_param.cadence.std = std(s_stp{2,2});
% sub01.PFMP.st_param.cadence.raw = s_stp{3,2};
sub01.PFMP.st_param.cadence.mean = mean(s_stp{3,2});
sub01.PFMP.st_param.cadence.std = std(s_stp{3,2});
% sub01.MSMP.st_param.cadence.raw = s_stp{4,2};
sub01.MSMP.st_param.cadence.mean = mean(s_stp{4,2});
sub01.MSMP.st_param.cadence.std = std(s_stp{4,2});
% sub01.PSMP.st_param.cadence.raw = s_stp{5,2};
sub01.PSMP.st_param.cadence.mean = mean(s_stp{5,2});
sub01.PSMP.st_param.cadence.std = std(s_stp{5,2});

% sub02.NW.st_param.cadence.raw = s_stp{6,2};
sub02.NW.st_param.cadence.mean = mean(s_stp{6,2});
sub02.NW.st_param.cadence.std = std(s_stp{6,2});
% sub02.MFMP.st_param.cadence.raw = s_stp{7,2};
sub02.MFMP.st_param.cadence.mean = mean(s_stp{7,2});
sub02.MFMP.st_param.cadence.std = std(s_stp{7,2});
% sub02.PFMP.st_param.cadence.raw = s_stp{8,2};
sub02.PFMP.st_param.cadence.mean = mean(s_stp{8,2});
sub02.PFMP.st_param.cadence.std = std(s_stp{8,2});
% sub02.MSMP.st_param.cadence.raw = s_stp{9,2};
sub02.MSMP.st_param.cadence.mean = mean(s_stp{9,2});
sub02.MSMP.st_param.cadence.std = std(s_stp{9,2});
% sub02.PSMP.st_param.cadence.raw = s_stp{10,2};
sub02.PSMP.st_param.cadence.mean = mean(s_stp{10,2});
sub02.PSMP.st_param.cadence.std = std(s_stp{10,2});

% sub03.NW.st_param.cadence.raw = s_stp{11,2};
sub03.NW.st_param.cadence.mean = mean(s_stp{1,2});
sub03.NW.st_param.cadence.std = std(s_stp{11,2});
% sub03.MFMP.st_param.cadence.raw = s_stp{12,2};
sub03.MFMP.st_param.cadence.mean = mean(s_stp{12,2});
sub03.MFMP.st_param.cadence.std = std(s_stp{12,2});
% sub03.PFMP.st_param.cadence.raw = s_stp{13,2};
sub03.PFMP.st_param.cadence.mean = mean(s_stp{13,2});
sub03.PFMP.st_param.cadence.std = std(s_stp{13,2});
% sub03.MSMP.st_param.cadence.raw = s_stp{14,2};
sub03.MSMP.st_param.cadence.mean = mean(s_stp{14,2});
sub03.MSMP.st_param.cadence.std = std(s_stp{14,2});
% sub03.PSMP.st_param.cadence.raw = s_stp{15,2};
sub03.PSMP.st_param.cadence.mean = mean(s_stp{15,2});
sub03.PSMP.st_param.cadence.std = std(s_stp{15,2});

% sub04.NW.st_param.cadence.raw = s_stp{16,2};
sub04.NW.st_param.cadence.mean = mean(s_stp{16,2});
sub04.NW.st_param.cadence.std = std(s_stp{16,2});
% sub04.MFMP.st_param.cadence.raw = s_stp{17,2};
sub04.MFMP.st_param.cadence.mean = mean(s_stp{17,2});
sub04.MFMP.st_param.cadence.std = std(s_stp{17,2});
% sub04.PFMP.st_param.cadence.raw = s_stp{18,2};
sub04.PFMP.st_param.cadence.mean = mean(s_stp{18,2});
sub04.PFMP.st_param.cadence.std = std(s_stp{18,2});
% sub04.MSMP.st_param.cadence.raw = s_stp{19,2};
sub04.MSMP.st_param.cadence.mean = mean(s_stp{19,2});
sub04.MSMP.st_param.cadence.std = std(s_stp{19,2});
% sub04.PSMP.st_param.cadence.raw = s_stp{20,2};
sub04.PSMP.st_param.cadence.mean = mean(s_stp{20,2});
sub04.PSMP.st_param.cadence.std = std(s_stp{20,2});


% sub05.NW.st_param.cadence.raw = s_stp{21,2};
sub05.NW.st_param.cadence.mean = mean(s_stp{21,2});
sub05.NW.st_param.cadence.std = std(s_stp{21,2});
% sub05.MFMP.st_param.cadence.raw = s_stp{22,2};
sub05.MFMP.st_param.cadence.mean = mean(s_stp{22,2});
sub05.MFMP.st_param.cadence.std = std(s_stp{22,2});
% sub05.PFMP.st_param.cadence.raw = s_stp{23,2};
sub05.PFMP.st_param.cadence.mean = mean(s_stp{23,2});
sub05.PFMP.st_param.cadence.std = std(s_stp{23,2});
% sub05.MSMP.st_param.cadence.raw = s_stp{24,2};
sub05.MSMP.st_param.cadence.mean = mean(s_stp{24,2});
sub05.MSMP.st_param.cadence.std = std(s_stp{24,2});
% sub05.PSMP.st_param.cadence.raw = s_stp{25,2};
sub05.PSMP.st_param.cadence.mean = mean(s_stp{25,2});
sub05.PSMP.st_param.cadence.std = std(s_stp{25,2});


% sub06.NW.st_param.cadence.raw = s_stp{26,2};
sub06.NW.st_param.cadence.mean = mean(s_stp{26,2});
sub06.NW.st_param.cadence.std = std(s_stp{26,2});
% sub06.MFMP.st_param.cadence.raw = s_stp{27,2};
sub06.MFMP.st_param.cadence.mean = mean(s_stp{27,2});
sub06.MFMP.st_param.cadence.std = std(s_stp{27,2});
% sub06.PFMP.st_param.cadence.raw = s_stp{28,2};
sub06.PFMP.st_param.cadence.mean = mean(s_stp{28,2});
sub06.PFMP.st_param.cadence.std = std(s_stp{28,2});
% sub06.MSMP.st_param.cadence.raw = s_stp{29,2};
sub06.MSMP.st_param.cadence.mean = mean(s_stp{29,2});
sub06.MSMP.st_param.cadence.std = std(s_stp{29,2});
% sub06.PSMP.st_param.cadence.raw = s_stp{30,2};
sub06.PSMP.st_param.cadence.mean = mean(s_stp{30,2});
sub06.PSMP.st_param.cadence.std = std(s_stp{30,2});


% sub07.NW.st_param.cadence.raw = s_stp{31,2};
sub07.NW.st_param.cadence.mean = mean(s_stp{31,2});
sub07.NW.st_param.cadence.std = std(s_stp{31,2});
% sub07.MFMP.st_param.cadence.raw = s_stp{32,2};
sub07.MFMP.st_param.cadence.mean = mean(s_stp{32,2});
sub07.MFMP.st_param.cadence.std = std(s_stp{32,2});
% sub07.PFMP.st_param.cadence.raw = s_stp{33,2};
sub07.PFMP.st_param.cadence.mean = mean(s_stp{33,2});
sub07.PFMP.st_param.cadence.std = std(s_stp{33,2});
% sub07.MSMP.st_param.cadence.raw = s_stp{34,2};
sub07.MSMP.st_param.cadence.mean = mean(s_stp{34,2});
sub07.MSMP.st_param.cadence.std = std(s_stp{34,2});
% sub07.PSMP.st_param.cadence.raw = s_stp{35,2};
sub07.PSMP.st_param.cadence.mean = mean(s_stp{35,2});
sub07.PSMP.st_param.cadence.std = std(s_stp{35,2});

sub01.NW.st_param.cadence.unit = 'sec^-1';
sub02.NW.st_param.cadence.unit = 'sec^-1';
sub03.NW.st_param.cadence.unit = 'sec^-1';
sub04.NW.st_param.cadence.unit = 'sec^-1';
sub05.NW.st_param.cadence.unit = 'sec^-1';
sub06.NW.st_param.cadence.unit = 'sec^-1';
sub07.NW.st_param.cadence.unit = 'sec^-1';
sub01.MFMP.st_param.cadence.unit = 'sec^-1';
sub01.PFMP.st_param.cadence.unit = 'sec^-1';
sub01.MSMP.st_param.cadence.unit = 'sec^-1';
sub01.PSMP.st_param.cadence.unit = 'sec^-1';
sub02.MFMP.st_param.cadence.unit = 'sec^-1';
sub02.PFMP.st_param.cadence.unit = 'sec^-1';
sub02.MSMP.st_param.cadence.unit = 'sec^-1';
sub02.PSMP.st_param.cadence.unit = 'sec^-1';
sub03.MFMP.st_param.cadence.unit = 'sec^-1';
sub03.PFMP.st_param.cadence.unit = 'sec^-1';
sub03.MSMP.st_param.cadence.unit = 'sec^-1';
sub03.PSMP.st_param.cadence.unit = 'sec^-1';
sub04.MFMP.st_param.cadence.unit = 'sec^-1';
sub04.PFMP.st_param.cadence.unit = 'sec^-1';
sub04.MSMP.st_param.cadence.unit = 'sec^-1';
sub04.PSMP.st_param.cadence.unit = 'sec^-1';
sub05.MFMP.st_param.cadence.unit = 'sec^-1';
sub05.PFMP.st_param.cadence.unit = 'sec^-1';
sub05.MSMP.st_param.cadence.unit = 'sec^-1';
sub05.PSMP.st_param.cadence.unit = 'sec^-1';
sub06.MFMP.st_param.cadence.unit = 'sec^-1';
sub06.PFMP.st_param.cadence.unit = 'sec^-1';
sub06.MSMP.st_param.cadence.unit = 'sec^-1';
sub06.PSMP.st_param.cadence.unit = 'sec^-1';
sub07.MFMP.st_param.cadence.unit = 'sec^-1';
sub07.PFMP.st_param.cadence.unit = 'sec^-1';
sub07.MSMP.st_param.cadence.unit = 'sec^-1';
sub07.PSMP.st_param.cadence.unit = 'sec^-1';
%% stance

% sub01.NW.st_param.stancetime.raw = s_stp{1,3};
sub01.NW.st_param.stancetime.mean = mean(s_stp{1,3});
sub01.NW.st_param.stancetime.std = std(s_stp{1,3});
% sub01.MFMP.st_param.stancetime.raw = s_stp{2,3};
sub01.MFMP.st_param.stancetime.mean = mean(s_stp{2,3});
sub01.MFMP.st_param.stancetime.std = std(s_stp{2,3});
% sub01.PFMP.st_param.stancetime.raw = s_stp{3,3};
sub01.PFMP.st_param.stancetime.mean = mean(s_stp{3,3});
sub01.PFMP.st_param.stancetime.std = std(s_stp{3,3});
% sub01.MSMP.st_param.stancetime.raw = s_stp{4,3};
sub01.MSMP.st_param.stancetime.mean = mean(s_stp{4,3});
sub01.MSMP.st_param.stancetime.std = std(s_stp{4,3});
% sub01.PSMP.st_param.stancetime.raw = s_stp{5,3};
sub01.PSMP.st_param.stancetime.mean = mean(s_stp{5,3});
sub01.PSMP.st_param.stancetime.std = std(s_stp{5,3});

% sub02.NW.st_param.stancetime.raw = s_stp{6,3};
sub02.NW.st_param.stancetime.mean = mean(s_stp{6,3});
sub02.NW.st_param.stancetime.std = std(s_stp{6,3});
% sub02.MFMP.st_param.stancetime.raw = s_stp{7,3};
sub02.MFMP.st_param.stancetime.mean = mean(s_stp{7,3});
sub02.MFMP.st_param.stancetime.std = std(s_stp{7,3});
% sub02.PFMP.st_param.stancetime.raw = s_stp{8,3};
sub02.PFMP.st_param.stancetime.mean = mean(s_stp{8,3});
sub02.PFMP.st_param.stancetime.std = std(s_stp{8,3});
% sub02.MSMP.st_param.stancetime.raw = s_stp{9,3};
sub02.MSMP.st_param.stancetime.mean = mean(s_stp{9,3});
sub02.MSMP.st_param.stancetime.std = std(s_stp{9,3});
% sub02.PSMP.st_param.stancetime.raw = s_stp{10,3};
sub02.PSMP.st_param.stancetime.mean = mean(s_stp{10,3});
sub02.PSMP.st_param.stancetime.std = std(s_stp{10,3});

% sub03.NW.st_param.stancetime.raw = s_stp{11,3};
sub03.NW.st_param.stancetime.mean = mean(s_stp{1,3});
sub03.NW.st_param.stancetime.std = std(s_stp{11,3});
% sub03.MFMP.st_param.stancetime.raw = s_stp{12,3};
sub03.MFMP.st_param.stancetime.mean = mean(s_stp{12,3});
sub03.MFMP.st_param.stancetime.std = std(s_stp{12,3});
% sub03.PFMP.st_param.stancetime.raw = s_stp{13,3};
sub03.PFMP.st_param.stancetime.mean = mean(s_stp{13,3});
sub03.PFMP.st_param.stancetime.std = std(s_stp{13,3});
% sub03.MSMP.st_param.stancetime.raw = s_stp{14,3};
sub03.MSMP.st_param.stancetime.mean = mean(s_stp{14,3});
sub03.MSMP.st_param.stancetime.std = std(s_stp{14,3});
% sub03.PSMP.st_param.stancetime.raw = s_stp{15,3};
sub03.PSMP.st_param.stancetime.mean = mean(s_stp{15,3});
sub03.PSMP.st_param.stancetime.std = std(s_stp{15,3});

% sub04.NW.st_param.stancetime.raw = s_stp{16,3};
sub04.NW.st_param.stancetime.mean = mean(s_stp{16,3});
sub04.NW.st_param.stancetime.std = std(s_stp{16,3});
% sub04.MFMP.st_param.stancetime.raw = s_stp{17,3};
sub04.MFMP.st_param.stancetime.mean = mean(s_stp{17,3});
sub04.MFMP.st_param.stancetime.std = std(s_stp{17,3});
% sub04.PFMP.st_param.stancetime.raw = s_stp{18,3};
sub04.PFMP.st_param.stancetime.mean = mean(s_stp{18,3});
sub04.PFMP.st_param.stancetime.std = std(s_stp{18,3});
% sub04.MSMP.st_param.stancetime.raw = s_stp{19,3};
sub04.MSMP.st_param.stancetime.mean = mean(s_stp{19,3});
sub04.MSMP.st_param.stancetime.std = std(s_stp{19,3});
% sub04.PSMP.st_param.stancetime.raw = s_stp{20,3};
sub04.PSMP.st_param.stancetime.mean = mean(s_stp{20,3});
sub04.PSMP.st_param.stancetime.std = std(s_stp{20,3});


% sub05.NW.st_param.stancetime.raw = s_stp{21,3};
sub05.NW.st_param.stancetime.mean = mean(s_stp{21,3});
sub05.NW.st_param.stancetime.std = std(s_stp{21,3});
% sub05.MFMP.st_param.stancetime.raw = s_stp{22,3};
sub05.MFMP.st_param.stancetime.mean = mean(s_stp{22,3});
sub05.MFMP.st_param.stancetime.std = std(s_stp{22,3});
% sub05.PFMP.st_param.stancetime.raw = s_stp{23,3};
sub05.PFMP.st_param.stancetime.mean = mean(s_stp{23,3});
sub05.PFMP.st_param.stancetime.std = std(s_stp{23,3});
% sub05.MSMP.st_param.stancetime.raw = s_stp{24,3};
sub05.MSMP.st_param.stancetime.mean = mean(s_stp{24,3});
sub05.MSMP.st_param.stancetime.std = std(s_stp{24,3});
% sub05.PSMP.st_param.stancetime.raw = s_stp{25,3};
sub05.PSMP.st_param.stancetime.mean = mean(s_stp{25,3});
sub05.PSMP.st_param.stancetime.std = std(s_stp{25,3});


% sub06.NW.st_param.stancetime.raw = s_stp{26,3};
sub06.NW.st_param.stancetime.mean = mean(s_stp{26,3});
sub06.NW.st_param.stancetime.std = std(s_stp{26,3});
% sub06.MFMP.st_param.stancetime.raw = s_stp{27,3};
sub06.MFMP.st_param.stancetime.mean = mean(s_stp{27,3});
sub06.MFMP.st_param.stancetime.std = std(s_stp{27,3});
% sub06.PFMP.st_param.stancetime.raw = s_stp{28,3};
sub06.PFMP.st_param.stancetime.mean = mean(s_stp{28,3});
sub06.PFMP.st_param.stancetime.std = std(s_stp{28,3});
% sub06.MSMP.st_param.stancetime.raw = s_stp{29,3};
sub06.MSMP.st_param.stancetime.mean = mean(s_stp{29,3});
sub06.MSMP.st_param.stancetime.std = std(s_stp{29,3});
% sub06.PSMP.st_param.stancetime.raw = s_stp{30,3};
sub06.PSMP.st_param.stancetime.mean = mean(s_stp{30,3});
sub06.PSMP.st_param.stancetime.std = std(s_stp{30,3});


% sub07.NW.st_param.stancetime.raw = s_stp{31,3};
sub07.NW.st_param.stancetime.mean = mean(s_stp{31,3});
sub07.NW.st_param.stancetime.std = std(s_stp{31,3});
% sub07.MFMP.st_param.stancetime.raw = s_stp{32,3};
sub07.MFMP.st_param.stancetime.mean = mean(s_stp{32,3});
sub07.MFMP.st_param.stancetime.std = std(s_stp{32,3});
% sub07.PFMP.st_param.stancetime.raw = s_stp{33,3};
sub07.PFMP.st_param.stancetime.mean = mean(s_stp{33,3});
sub07.PFMP.st_param.stancetime.std = std(s_stp{33,3});
% sub07.MSMP.st_param.stancetime.raw = s_stp{34,3};
sub07.MSMP.st_param.stancetime.mean = mean(s_stp{34,3});
sub07.MSMP.st_param.stancetime.std = std(s_stp{34,3});
% sub07.PSMP.st_param.stancetime.raw = s_stp{35,3};
sub07.PSMP.st_param.stancetime.mean = mean(s_stp{35,3});
sub07.PSMP.st_param.stancetime.std = std(s_stp{35,3});

sub01.NW.st_param.stancetime.unit = 's';
sub02.NW.st_param.stancetime.unit = 's';
sub03.NW.st_param.stancetime.unit = 's';
sub04.NW.st_param.stancetime.unit = 's';
sub05.NW.st_param.stancetime.unit = 's';
sub06.NW.st_param.stancetime.unit = 's';
sub07.NW.st_param.stancetime.unit = 's';
sub01.MFMP.st_param.stancetime.unit = 's';
sub01.PFMP.st_param.stancetime.unit = 's';
sub01.MSMP.st_param.stancetime.unit = 's';
sub01.PSMP.st_param.stancetime.unit = 's';
sub02.MFMP.st_param.stancetime.unit = 's';
sub02.PFMP.st_param.stancetime.unit = 's';
sub02.MSMP.st_param.stancetime.unit = 's';
sub02.PSMP.st_param.stancetime.unit = 's';
sub03.MFMP.st_param.stancetime.unit = 's';
sub03.PFMP.st_param.stancetime.unit = 's';
sub03.MSMP.st_param.stancetime.unit = 's';
sub03.PSMP.st_param.stancetime.unit = 's';
sub04.MFMP.st_param.stancetime.unit = 's';
sub04.PFMP.st_param.stancetime.unit = 's';
sub04.MSMP.st_param.stancetime.unit = 's';
sub04.PSMP.st_param.stancetime.unit = 's';
sub05.MFMP.st_param.stancetime.unit = 's';
sub05.PFMP.st_param.stancetime.unit = 's';
sub05.MSMP.st_param.stancetime.unit = 's';
sub05.PSMP.st_param.stancetime.unit = 's';
sub06.MFMP.st_param.stancetime.unit = 's';
sub06.PFMP.st_param.stancetime.unit = 's';
sub06.MSMP.st_param.stancetime.unit = 's';
sub06.PSMP.st_param.stancetime.unit = 's';
sub07.MFMP.st_param.stancetime.unit = 's';
sub07.PFMP.st_param.stancetime.unit = 's';
sub07.MSMP.st_param.stancetime.unit = 's';
sub07.PSMP.st_param.stancetime.unit = 's';
%% double

% sub01.NW.st_param.double_stancetime.raw = s_stp{1,4};
sub01.NW.st_param.double_stancetime.mean = mean(s_stp{1,4});
sub01.NW.st_param.double_stancetime.std = std(s_stp{1,4});
% sub01.MFMP.st_param.double_stancetime.raw = s_stp{2,4};
sub01.MFMP.st_param.double_stancetime.mean = mean(s_stp{2,4});
sub01.MFMP.st_param.double_stancetime.std = std(s_stp{2,4});
% sub01.PFMP.st_param.double_stancetime.raw = s_stp{3,4};
sub01.PFMP.st_param.double_stancetime.mean = mean(s_stp{3,4});
sub01.PFMP.st_param.double_stancetime.std = std(s_stp{3,4});
% sub01.MSMP.st_param.double_stancetime.raw = s_stp{4,4};
sub01.MSMP.st_param.double_stancetime.mean = mean(s_stp{4,4});
sub01.MSMP.st_param.double_stancetime.std = std(s_stp{4,4});
% sub01.PSMP.st_param.double_stancetime.raw = s_stp{5,4};
sub01.PSMP.st_param.double_stancetime.mean = mean(s_stp{5,4});
sub01.PSMP.st_param.double_stancetime.std = std(s_stp{5,4});

% sub02.NW.st_param.double_stancetime.raw = s_stp{6,4};
sub02.NW.st_param.double_stancetime.mean = mean(s_stp{6,4});
sub02.NW.st_param.double_stancetime.std = std(s_stp{6,4});
% sub02.MFMP.st_param.double_stancetime.raw = s_stp{7,4};
sub02.MFMP.st_param.double_stancetime.mean = mean(s_stp{7,4});
sub02.MFMP.st_param.double_stancetime.std = std(s_stp{7,4});
% sub02.PFMP.st_param.double_stancetime.raw = s_stp{8,4};
sub02.PFMP.st_param.double_stancetime.mean = mean(s_stp{8,4});
sub02.PFMP.st_param.double_stancetime.std = std(s_stp{8,4});
% sub02.MSMP.st_param.double_stancetime.raw = s_stp{9,4};
sub02.MSMP.st_param.double_stancetime.mean = mean(s_stp{9,4});
sub02.MSMP.st_param.double_stancetime.std = std(s_stp{9,4});
% sub02.PSMP.st_param.double_stancetime.raw = s_stp{10,4};
sub02.PSMP.st_param.double_stancetime.mean = mean(s_stp{10,4});
sub02.PSMP.st_param.double_stancetime.std = std(s_stp{10,4});

% sub03.NW.st_param.double_stancetime.raw = s_stp{11,4};
sub03.NW.st_param.double_stancetime.mean = mean(s_stp{1,4});
sub03.NW.st_param.double_stancetime.std = std(s_stp{11,4});
% sub03.MFMP.st_param.double_stancetime.raw = s_stp{12,4};
sub03.MFMP.st_param.double_stancetime.mean = mean(s_stp{12,4});
sub03.MFMP.st_param.double_stancetime.std = std(s_stp{12,4});
% sub03.PFMP.st_param.double_stancetime.raw = s_stp{13,4};
sub03.PFMP.st_param.double_stancetime.mean = mean(s_stp{13,4});
sub03.PFMP.st_param.double_stancetime.std = std(s_stp{13,4});
% sub03.MSMP.st_param.double_stancetime.raw = s_stp{14,4};
sub03.MSMP.st_param.double_stancetime.mean = mean(s_stp{14,4});
sub03.MSMP.st_param.double_stancetime.std = std(s_stp{14,4});
% sub03.PSMP.st_param.double_stancetime.raw = s_stp{15,4};
sub03.PSMP.st_param.double_stancetime.mean = mean(s_stp{15,4});
sub03.PSMP.st_param.double_stancetime.std = std(s_stp{15,4});

% sub04.NW.st_param.double_stancetime.raw = s_stp{16,4};
sub04.NW.st_param.double_stancetime.mean = mean(s_stp{16,4});
sub04.NW.st_param.double_stancetime.std = std(s_stp{16,4});
% sub04.MFMP.st_param.double_stancetime.raw = s_stp{17,4};
sub04.MFMP.st_param.double_stancetime.mean = mean(s_stp{17,4});
sub04.MFMP.st_param.double_stancetime.std = std(s_stp{17,4});
% sub04.PFMP.st_param.double_stancetime.raw = s_stp{18,4};
sub04.PFMP.st_param.double_stancetime.mean = mean(s_stp{18,4});
sub04.PFMP.st_param.double_stancetime.std = std(s_stp{18,4});
% sub04.MSMP.st_param.double_stancetime.raw = s_stp{19,4};
sub04.MSMP.st_param.double_stancetime.mean = mean(s_stp{19,4});
sub04.MSMP.st_param.double_stancetime.std = std(s_stp{19,4});
% sub04.PSMP.st_param.double_stancetime.raw = s_stp{20,4};
sub04.PSMP.st_param.double_stancetime.mean = mean(s_stp{20,4});
sub04.PSMP.st_param.double_stancetime.std = std(s_stp{20,4});


% sub05.NW.st_param.double_stancetime.raw = s_stp{21,4};
sub05.NW.st_param.double_stancetime.mean = mean(s_stp{21,4});
sub05.NW.st_param.double_stancetime.std = std(s_stp{21,4});
% sub05.MFMP.st_param.double_stancetime.raw = s_stp{22,4};
sub05.MFMP.st_param.double_stancetime.mean = mean(s_stp{22,4});
sub05.MFMP.st_param.double_stancetime.std = std(s_stp{22,4});
% sub05.PFMP.st_param.double_stancetime.raw = s_stp{23,4};
sub05.PFMP.st_param.double_stancetime.mean = mean(s_stp{23,4});
sub05.PFMP.st_param.double_stancetime.std = std(s_stp{23,4});
% sub05.MSMP.st_param.double_stancetime.raw = s_stp{24,4};
sub05.MSMP.st_param.double_stancetime.mean = mean(s_stp{24,4});
sub05.MSMP.st_param.double_stancetime.std = std(s_stp{24,4});
% sub05.PSMP.st_param.double_stancetime.raw = s_stp{25,4};
sub05.PSMP.st_param.double_stancetime.mean = mean(s_stp{25,4});
sub05.PSMP.st_param.double_stancetime.std = std(s_stp{25,4});


% sub06.NW.st_param.double_stancetime.raw = s_stp{26,4};
sub06.NW.st_param.double_stancetime.mean = mean(s_stp{26,4});
sub06.NW.st_param.double_stancetime.std = std(s_stp{26,4});
% sub06.MFMP.st_param.double_stancetime.raw = s_stp{27,4};
sub06.MFMP.st_param.double_stancetime.mean = mean(s_stp{27,4});
sub06.MFMP.st_param.double_stancetime.std = std(s_stp{27,4});
% sub06.PFMP.st_param.double_stancetime.raw = s_stp{28,4};
sub06.PFMP.st_param.double_stancetime.mean = mean(s_stp{28,4});
sub06.PFMP.st_param.double_stancetime.std = std(s_stp{28,4});
% sub06.MSMP.st_param.double_stancetime.raw = s_stp{29,4};
sub06.MSMP.st_param.double_stancetime.mean = mean(s_stp{29,4});
sub06.MSMP.st_param.double_stancetime.std = std(s_stp{29,4});
% sub06.PSMP.st_param.double_stancetime.raw = s_stp{30,4};
sub06.PSMP.st_param.double_stancetime.mean = mean(s_stp{30,4});
sub06.PSMP.st_param.double_stancetime.std = std(s_stp{30,4});


% sub07.NW.st_param.double_stancetime.raw = s_stp{31,4};
sub07.NW.st_param.double_stancetime.mean = mean(s_stp{31,4});
sub07.NW.st_param.double_stancetime.std = std(s_stp{31,4});
% sub07.MFMP.st_param.double_stancetime.raw = s_stp{32,4};
sub07.MFMP.st_param.double_stancetime.mean = mean(s_stp{32,4});
sub07.MFMP.st_param.double_stancetime.std = std(s_stp{32,4});
% sub07.PFMP.st_param.double_stancetime.raw = s_stp{33,4};
sub07.PFMP.st_param.double_stancetime.mean = mean(s_stp{33,4});
sub07.PFMP.st_param.double_stancetime.std = std(s_stp{33,4});
% sub07.MSMP.st_param.double_stancetime.raw = s_stp{34,4};
sub07.MSMP.st_param.double_stancetime.mean = mean(s_stp{34,4});
sub07.MSMP.st_param.double_stancetime.std = std(s_stp{34,4});
% sub07.PSMP.st_param.double_stancetime.raw = s_stp{35,4};
sub07.PSMP.st_param.double_stancetime.mean = mean(s_stp{35,4});
sub07.PSMP.st_param.double_stancetime.std = std(s_stp{35,4});

sub01.NW.st_param.double_stancetime.unit = '%gcp';
sub02.NW.st_param.double_stancetime.unit = '%gcp';
sub03.NW.st_param.double_stancetime.unit = '%gcp';
sub04.NW.st_param.double_stancetime.unit = '%gcp';
sub05.NW.st_param.double_stancetime.unit = '%gcp';
sub06.NW.st_param.double_stancetime.unit = '%gcp';
sub07.NW.st_param.double_stancetime.unit = '%gcp';
sub01.MFMP.st_param.double_stancetime.unit = '%gcp';
sub01.PFMP.st_param.double_stancetime.unit = '%gcp';
sub01.MSMP.st_param.double_stancetime.unit = '%gcp';
sub01.PSMP.st_param.double_stancetime.unit = '%gcp';
sub02.MFMP.st_param.double_stancetime.unit = '%gcp';
sub02.PFMP.st_param.double_stancetime.unit = '%gcp';
sub02.MSMP.st_param.double_stancetime.unit = '%gcp';
sub02.PSMP.st_param.double_stancetime.unit = '%gcp';
sub03.MFMP.st_param.double_stancetime.unit = '%gcp';
sub03.PFMP.st_param.double_stancetime.unit = '%gcp';
sub03.MSMP.st_param.double_stancetime.unit = '%gcp';
sub03.PSMP.st_param.double_stancetime.unit = '%gcp';
sub04.MFMP.st_param.double_stancetime.unit = '%gcp';
sub04.PFMP.st_param.double_stancetime.unit = '%gcp';
sub04.MSMP.st_param.double_stancetime.unit = '%gcp';
sub04.PSMP.st_param.double_stancetime.unit = '%gcp';
sub05.MFMP.st_param.double_stancetime.unit = '%gcp';
sub05.PFMP.st_param.double_stancetime.unit = '%gcp';
sub05.MSMP.st_param.double_stancetime.unit = '%gcp';
sub05.PSMP.st_param.double_stancetime.unit = '%gcp';
sub06.MFMP.st_param.double_stancetime.unit = '%gcp';
sub06.PFMP.st_param.double_stancetime.unit = '%gcp';
sub06.MSMP.st_param.double_stancetime.unit = '%gcp';
sub06.PSMP.st_param.double_stancetime.unit = '%gcp';
sub07.MFMP.st_param.double_stancetime.unit = '%gcp';
sub07.PFMP.st_param.double_stancetime.unit = '%gcp';
sub07.MSMP.st_param.double_stancetime.unit = '%gcp';
sub07.PSMP.st_param.double_stancetime.unit = '%gcp';
%% swing

% sub01.NW.st_param.swingtime.raw = s_stp{1,5};
sub01.NW.st_param.swingtime.mean = mean(s_stp{1,5});
sub01.NW.st_param.swingtime.std = std(s_stp{1,5});
% sub01.MFMP.st_param.swingtime.raw = s_stp{2,5};
sub01.MFMP.st_param.swingtime.mean = mean(s_stp{2,5});
sub01.MFMP.st_param.swingtime.std = std(s_stp{2,5});
% sub01.PFMP.st_param.swingtime.raw = s_stp{3,5};
sub01.PFMP.st_param.swingtime.mean = mean(s_stp{3,5});
sub01.PFMP.st_param.swingtime.std = std(s_stp{3,5});
% sub01.MSMP.st_param.swingtime.raw = s_stp{4,5};
sub01.MSMP.st_param.swingtime.mean = mean(s_stp{4,5});
sub01.MSMP.st_param.swingtime.std = std(s_stp{4,5});
% sub01.PSMP.st_param.swingtime.raw = s_stp{5,5};
sub01.PSMP.st_param.swingtime.mean = mean(s_stp{5,5});
sub01.PSMP.st_param.swingtime.std = std(s_stp{5,5});

% sub02.NW.st_param.swingtime.raw = s_stp{6,5};
sub02.NW.st_param.swingtime.mean = mean(s_stp{6,5});
sub02.NW.st_param.swingtime.std = std(s_stp{6,5});
% sub02.MFMP.st_param.swingtime.raw = s_stp{7,5};
sub02.MFMP.st_param.swingtime.mean = mean(s_stp{7,5});
sub02.MFMP.st_param.swingtime.std = std(s_stp{7,5});
% sub02.PFMP.st_param.swingtime.raw = s_stp{8,5};
sub02.PFMP.st_param.swingtime.mean = mean(s_stp{8,5});
sub02.PFMP.st_param.swingtime.std = std(s_stp{8,5});
% sub02.MSMP.st_param.swingtime.raw = s_stp{9,5};
sub02.MSMP.st_param.swingtime.mean = mean(s_stp{9,5});
sub02.MSMP.st_param.swingtime.std = std(s_stp{9,5});
% sub02.PSMP.st_param.swingtime.raw = s_stp{10,5};
sub02.PSMP.st_param.swingtime.mean = mean(s_stp{10,5});
sub02.PSMP.st_param.swingtime.std = std(s_stp{10,5});

% sub03.NW.st_param.swingtime.raw = s_stp{11,5};
sub03.NW.st_param.swingtime.mean = mean(s_stp{1,5});
sub03.NW.st_param.swingtime.std = std(s_stp{11,5});
% sub03.MFMP.st_param.swingtime.raw = s_stp{12,5};
sub03.MFMP.st_param.swingtime.mean = mean(s_stp{12,5});
sub03.MFMP.st_param.swingtime.std = std(s_stp{12,5});
% sub03.PFMP.st_param.swingtime.raw = s_stp{13,5};
sub03.PFMP.st_param.swingtime.mean = mean(s_stp{13,5});
sub03.PFMP.st_param.swingtime.std = std(s_stp{13,5});
% sub03.MSMP.st_param.swingtime.raw = s_stp{14,5};
sub03.MSMP.st_param.swingtime.mean = mean(s_stp{14,5});
sub03.MSMP.st_param.swingtime.std = std(s_stp{14,5});
% sub03.PSMP.st_param.swingtime.raw = s_stp{15,5};
sub03.PSMP.st_param.swingtime.mean = mean(s_stp{15,5});
sub03.PSMP.st_param.swingtime.std = std(s_stp{15,5});

% sub04.NW.st_param.swingtime.raw = s_stp{16,5};
sub04.NW.st_param.swingtime.mean = mean(s_stp{16,5});
sub04.NW.st_param.swingtime.std = std(s_stp{16,5});
% sub04.MFMP.st_param.swingtime.raw = s_stp{17,5};
sub04.MFMP.st_param.swingtime.mean = mean(s_stp{17,5});
sub04.MFMP.st_param.swingtime.std = std(s_stp{17,5});
% sub04.PFMP.st_param.swingtime.raw = s_stp{18,5};
sub04.PFMP.st_param.swingtime.mean = mean(s_stp{18,5});
sub04.PFMP.st_param.swingtime.std = std(s_stp{18,5});
% sub04.MSMP.st_param.swingtime.raw = s_stp{19,5};
sub04.MSMP.st_param.swingtime.mean = mean(s_stp{19,5});
sub04.MSMP.st_param.swingtime.std = std(s_stp{19,5});
% sub04.PSMP.st_param.swingtime.raw = s_stp{20,5};
sub04.PSMP.st_param.swingtime.mean = mean(s_stp{20,5});
sub04.PSMP.st_param.swingtime.std = std(s_stp{20,5});


% sub05.NW.st_param.swingtime.raw = s_stp{21,5};
sub05.NW.st_param.swingtime.mean = mean(s_stp{21,5});
sub05.NW.st_param.swingtime.std = std(s_stp{21,5});
% sub05.MFMP.st_param.swingtime.raw = s_stp{22,5};
sub05.MFMP.st_param.swingtime.mean = mean(s_stp{22,5});
sub05.MFMP.st_param.swingtime.std = std(s_stp{22,5});
% sub05.PFMP.st_param.swingtime.raw = s_stp{23,5};
sub05.PFMP.st_param.swingtime.mean = mean(s_stp{23,5});
sub05.PFMP.st_param.swingtime.std = std(s_stp{23,5});
% sub05.MSMP.st_param.swingtime.raw = s_stp{24,5};
sub05.MSMP.st_param.swingtime.mean = mean(s_stp{24,5});
sub05.MSMP.st_param.swingtime.std = std(s_stp{24,5});
% sub05.PSMP.st_param.swingtime.raw = s_stp{25,5};
sub05.PSMP.st_param.swingtime.mean = mean(s_stp{25,5});
sub05.PSMP.st_param.swingtime.std = std(s_stp{25,5});


% sub06.NW.st_param.swingtime.raw = s_stp{26,5};
sub06.NW.st_param.swingtime.mean = mean(s_stp{26,5});
sub06.NW.st_param.swingtime.std = std(s_stp{26,5});
% sub06.MFMP.st_param.swingtime.raw = s_stp{27,5};
sub06.MFMP.st_param.swingtime.mean = mean(s_stp{27,5});
sub06.MFMP.st_param.swingtime.std = std(s_stp{27,5});
% sub06.PFMP.st_param.swingtime.raw = s_stp{28,5};
sub06.PFMP.st_param.swingtime.mean = mean(s_stp{28,5});
sub06.PFMP.st_param.swingtime.std = std(s_stp{28,5});
% sub06.MSMP.st_param.swingtime.raw = s_stp{29,5};
sub06.MSMP.st_param.swingtime.mean = mean(s_stp{29,5});
sub06.MSMP.st_param.swingtime.std = std(s_stp{29,5});
% sub06.PSMP.st_param.swingtime.raw = s_stp{30,5};
sub06.PSMP.st_param.swingtime.mean = mean(s_stp{30,5});
sub06.PSMP.st_param.swingtime.std = std(s_stp{30,5});


% sub07.NW.st_param.swingtime.raw = s_stp{31,5};
sub07.NW.st_param.swingtime.mean = mean(s_stp{31,5});
sub07.NW.st_param.swingtime.std = std(s_stp{31,5});
% sub07.MFMP.st_param.swingtime.raw = s_stp{32,5};
sub07.MFMP.st_param.swingtime.mean = mean(s_stp{32,5});
sub07.MFMP.st_param.swingtime.std = std(s_stp{32,5});
% sub07.PFMP.st_param.swingtime.raw = s_stp{33,5};
sub07.PFMP.st_param.swingtime.mean = mean(s_stp{33,5});
sub07.PFMP.st_param.swingtime.std = std(s_stp{33,5});
% sub07.MSMP.st_param.swingtime.raw = s_stp{34,5};
sub07.MSMP.st_param.swingtime.mean = mean(s_stp{34,5});
sub07.MSMP.st_param.swingtime.std = std(s_stp{34,5});
% sub07.PSMP.st_param.swingtime.raw = s_stp{35,5};
sub07.PSMP.st_param.swingtime.mean = mean(s_stp{35,5});
sub07.PSMP.st_param.swingtime.std = std(s_stp{35,5});

sub01.NW.st_param.swingtime.unit = 's';
sub02.NW.st_param.swingtime.unit = 's';
sub03.NW.st_param.swingtime.unit = 's';
sub04.NW.st_param.swingtime.unit = 's';
sub05.NW.st_param.swingtime.unit = 's';
sub06.NW.st_param.swingtime.unit = 's';
sub07.NW.st_param.swingtime.unit = 's';
sub01.MFMP.st_param.swingtime.unit = 's';
sub01.PFMP.st_param.swingtime.unit = 's';
sub01.MSMP.st_param.swingtime.unit = 's';
sub01.PSMP.st_param.swingtime.unit = 's';
sub02.MFMP.st_param.swingtime.unit = 's';
sub02.PFMP.st_param.swingtime.unit = 's';
sub02.MSMP.st_param.swingtime.unit = 's';
sub02.PSMP.st_param.swingtime.unit = 's';
sub03.MFMP.st_param.swingtime.unit = 's';
sub03.PFMP.st_param.swingtime.unit = 's';
sub03.MSMP.st_param.swingtime.unit = 's';
sub03.PSMP.st_param.swingtime.unit = 's';
sub04.MFMP.st_param.swingtime.unit = 's';
sub04.PFMP.st_param.swingtime.unit = 's';
sub04.MSMP.st_param.swingtime.unit = 's';
sub04.PSMP.st_param.swingtime.unit = 's';
sub05.MFMP.st_param.swingtime.unit = 's';
sub05.PFMP.st_param.swingtime.unit = 's';
sub05.MSMP.st_param.swingtime.unit = 's';
sub05.PSMP.st_param.swingtime.unit = 's';
sub06.MFMP.st_param.swingtime.unit = 's';
sub06.PFMP.st_param.swingtime.unit = 's';
sub06.MSMP.st_param.swingtime.unit = 's';
sub06.PSMP.st_param.swingtime.unit = 's';
sub07.MFMP.st_param.swingtime.unit = 's';
sub07.PFMP.st_param.swingtime.unit = 's';
sub07.MSMP.st_param.swingtime.unit = 's';
sub07.PSMP.st_param.swingtime.unit = 's';


%% met

sub01.NW.metdata = results(1,2);
sub01.MFMP.metdata = results(1,3);
sub01.PFMP.metdata = results(1,4);
sub01.MSMP.metdata = results(1,5);
sub01.PSMP.metdata = results(1,6);

sub02.NW.metdata = results(2,2);
sub02.MFMP.metdata = results(2,3);
sub02.PFMP.metdata = results(2,4);
sub02.MSMP.metdata = results(2,5);
sub02.PSMP.metdata = results(2,6);

sub03.NW.metdata = results(3,2);
sub03.MFMP.metdata = results(3,3);
sub03.PFMP.metdata = results(3,4);
sub03.MSMP.metdata = results(3,5);
sub03.PSMP.metdata = results(3,6);

sub04.NW.metdata = results(4,2);
sub04.MFMP.metdata = results(4,3);
sub04.PFMP.metdata = results(4,4);
sub04.MSMP.metdata = results(4,5);
sub04.PSMP.metdata = results(4,6);

sub05.NW.metdata = results(5,2);
sub05.MFMP.metdata = results(5,3);
sub05.PFMP.metdata = results(5,4);
sub05.MSMP.metdata = results(5,5);
sub05.PSMP.metdata = results(5,6);

sub06.NW.metdata = results(6,2);
sub06.MFMP.metdata = results(6,3);
sub06.PFMP.metdata = results(6,4);
sub06.MSMP.metdata = results(6,5);
sub06.PSMP.metdata = results(6,6);

sub07.NW.metdata = results(7,2);
sub07.MFMP.metdata = results(7,3);
sub07.PFMP.metdata = results(7,4);
sub07.MSMP.metdata = results(7,5);
sub07.PSMP.metdata = results(7,6);

sub01.NW.metdata.unit = 'W/kg';
sub02.NW.metdata.unit = 'W/kg';
sub03.NW.metdata.unit = 'W/kg';
sub04.NW.metdata.unit = 'W/kg';
sub05.NW.metdata.unit = 'W/kg';
sub06.NW.metdata.unit = 'W/kg';
sub07.NW.metdata.unit = 'W/kg';
sub01.MFMP.metdata.unit = 'W/kg';
sub01.PFMP.metdata.unit = 'W/kg';
sub01.MSMP.metdata.unit = 'W/kg';
sub01.PSMP.metdata.unit = 'W/kg';
sub02.MFMP.metdata.unit = 'W/kg';
sub02.PFMP.metdata.unit = 'W/kg';
sub02.MSMP.metdata.unit = 'W/kg';
sub02.PSMP.metdata.unit = 'W/kg';
sub03.MFMP.metdata.unit = 'W/kg';
sub03.PFMP.metdata.unit = 'W/kg';
sub03.MSMP.metdata.unit = 'W/kg';
sub03.PSMP.metdata.unit = 'W/kg';
sub04.MFMP.metdata.unit = 'W/kg';
sub04.PFMP.metdata.unit = 'W/kg';
sub04.MSMP.metdata.unit = 'W/kg';
sub04.PSMP.metdata.unit = 'W/kg';
sub05.MFMP.metdata.unit = 'W/kg';
sub05.PFMP.metdata.unit = 'W/kg';
sub05.MSMP.metdata.unit = 'W/kg';
sub05.PSMP.metdata.unit = 'W/kg';
sub06.MFMP.metdata.unit = 'W/kg';
sub06.PFMP.metdata.unit = 'W/kg';
sub06.MSMP.metdata.unit = 'W/kg';
sub06.PSMP.metdata.unit = 'W/kg';
sub07.MFMP.metdata.unit = 'W/kg';
sub07.PFMP.metdata.unit = 'W/kg';
sub07.MSMP.metdata.unit = 'W/kg';
sub07.PSMP.metdata.unit = 'W/kg';
%% old kinematics

sub01.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{1,4};
sub01.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{1,4},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{1,4},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{1,5};
sub01.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{1,5},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{1,5},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{1,6};
sub01.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{1,6},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{1,6},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{1,4};
sub01.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{1,4},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{1,4},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{1,5};
sub01.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{1,5},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{1,5},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{1,6};
sub01.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{1,6},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{1,6},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{1,4};
sub01.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{1,4},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{1,4},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{1,5};
sub01.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{1,5},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{1,5},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{1,6};
sub01.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{1,6},2,'omitnan');
sub01.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{1,6},0,2,'omitnan');

sub01.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{1,1};
sub01.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{1,1},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{1,1},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{1,2};
sub01.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{1,2},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{1,2},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{1,3};
sub01.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{1,3},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{1,3},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{1,1};
sub01.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{1,1},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{1,1},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{1,2};
sub01.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{1,2},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{1,2},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{1,3};
sub01.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{1,3},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{1,3},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{1,1};
sub01.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{1,1},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{1,1},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{1,2};
sub01.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{1,2},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{1,2},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{1,3};
sub01.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{1,3},2,'omitnan');
sub01.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{1,3},0,2,'omitnan');

sub01.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{1,1};
sub01.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{1,1},2,'omitnan');
sub01.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{1,1},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{1,2};
sub01.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{1,2},2,'omitnan');
sub01.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{1,2},0,2,'omitnan');
sub01.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{1,3};
sub01.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{1,3},2,'omitnan');
sub01.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{1,3},0,2,'omitnan');


sub01.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{2,4};
sub01.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{2,4},2,'omitnan');
sub01.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{2,4},0,2,'omitnan');
sub01.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{2,5};
sub01.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{2,5},2,'omitnan');
sub01.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{2,5},0,2,'omitnan');
sub01.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{2,6};
sub01.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{2,6},2,'omitnan');
sub01.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{2,6},0,2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{2,4};
sub01.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{2,4},2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{2,4},0,2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{2,5};
sub01.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{2,5},2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{2,5},0,2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{2,6};
sub01.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{2,6},2,'omitnan');
sub01.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{2,6},0,2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{2,4};
sub01.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{2,4},2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{2,4},0,2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{2,5};
sub01.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{2,5},2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{2,5},0,2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{2,6};
sub01.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{2,6},2,'omitnan');
sub01.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{2,6},0,2,'omitnan');

sub01.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{2,1};
sub01.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{2,1},2,'omitnan');
sub01.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{2,1},0,2,'omitnan');
sub01.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{2,2};
sub01.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{2,2},2,'omitnan');
sub01.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{2,2},0,2,'omitnan');
sub01.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{2,3};
sub01.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{2,3},2,'omitnan');
sub01.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{2,3},0,2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{2,1};
sub01.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{2,1},2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{2,1},0,2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{2,2};
sub01.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{2,2},2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{2,2},0,2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{2,3};
sub01.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{2,3},2,'omitnan');
sub01.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{2,3},0,2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{2,1};
sub01.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{2,1},2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{2,1},0,2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{2,2};
sub01.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{2,2},2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{2,2},0,2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{2,3};
sub01.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{2,3},2,'omitnan');
sub01.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{2,3},0,2,'omitnan');

sub01.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{2,1};
sub01.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{2,1},2,'omitnan');
sub01.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{2,1},0,2,'omitnan');
sub01.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{2,2};
sub01.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{2,2},2,'omitnan');
sub01.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{2,2},0,2,'omitnan');
sub01.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{2,3};
sub01.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{2,3},2,'omitnan');
sub01.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{2,3},0,2,'omitnan');

sub01.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{3,4};
sub01.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{3,4},2,'omitnan');
sub01.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{3,4},0,2,'omitnan');
sub01.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{3,5};
sub01.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{3,5},2,'omitnan');
sub01.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{3,5},0,2,'omitnan');
sub01.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{3,6};
sub01.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{3,6},2,'omitnan');
sub01.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{3,6},0,2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{3,4};
sub01.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{3,4},2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{3,4},0,2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{3,5};
sub01.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{3,5},2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{3,5},0,2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{3,6};
sub01.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{3,6},2,'omitnan');
sub01.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{3,6},0,2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{3,4};
sub01.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{3,4},2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{3,4},0,2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{3,5};
sub01.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{3,5},2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{3,5},0,2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{3,6};
sub01.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{3,6},2,'omitnan');
sub01.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{3,6},0,2,'omitnan');

sub01.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{3,1};
sub01.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{3,1},2,'omitnan');
sub01.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{3,1},0,2,'omitnan');
sub01.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{3,2};
sub01.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{3,2},2,'omitnan');
sub01.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{3,2},0,2,'omitnan');
sub01.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{3,3};
sub01.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{3,3},2,'omitnan');
sub01.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{3,3},0,2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{3,1};
sub01.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{3,1},2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{3,1},0,2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{3,2};
sub01.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{3,2},2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{3,2},0,2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{3,3};
sub01.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{3,3},2,'omitnan');
sub01.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{3,3},0,2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{3,1};
sub01.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{3,1},2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{3,1},0,2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{3,2};
sub01.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{3,2},2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{3,2},0,2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{3,3};
sub01.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{3,3},2,'omitnan');
sub01.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{3,3},0,2,'omitnan');

sub01.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{3,1};
sub01.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{3,1},2,'omitnan');
sub01.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{3,1},0,2,'omitnan');
sub01.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{3,2};
sub01.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{3,2},2,'omitnan');
sub01.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{3,2},0,2,'omitnan');
sub01.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{3,3};
sub01.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{3,3},2,'omitnan');
sub01.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{3,3},0,2,'omitnan');

sub01.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{4,4};
sub01.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{4,4},2,'omitnan');
sub01.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{4,4},0,2,'omitnan');
sub01.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{4,5};
sub01.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{4,5},2,'omitnan');
sub01.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{4,5},0,2,'omitnan');
sub01.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{4,6};
sub01.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{4,6},2,'omitnan');
sub01.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{4,6},0,2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{4,4};
sub01.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{4,4},2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{4,4},0,2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{4,5};
sub01.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{4,5},2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{4,5},0,2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{4,6};
sub01.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{4,6},2,'omitnan');
sub01.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{4,6},0,2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{4,4};
sub01.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{4,4},2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{4,4},0,2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{4,5};
sub01.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{4,5},2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{4,5},0,2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{4,6};
sub01.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{4,6},2,'omitnan');
sub01.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{4,6},0,2,'omitnan');

sub01.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{4,1};
sub01.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{4,1},2,'omitnan');
sub01.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{4,1},0,2,'omitnan');
sub01.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{4,2};
sub01.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{4,2},2,'omitnan');
sub01.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{4,2},0,2,'omitnan');
sub01.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{4,3};
sub01.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{4,3},2,'omitnan');
sub01.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{4,3},0,2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{4,1};
sub01.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{4,1},2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{4,1},0,2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{4,2};
sub01.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{4,2},2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{4,2},0,2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{4,3};
sub01.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{4,3},2,'omitnan');
sub01.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{4,3},0,2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{4,1};
sub01.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{4,1},2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{4,1},0,2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{4,2};
sub01.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{4,2},2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{4,2},0,2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{4,3};
sub01.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{4,3},2,'omitnan');
sub01.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{4,3},0,2,'omitnan');

sub01.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{4,1};
sub01.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{4,1},2,'omitnan');
sub01.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{4,1},0,2,'omitnan');
sub01.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{4,2};
sub01.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{4,2},2,'omitnan');
sub01.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{4,2},0,2,'omitnan');
sub01.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{4,3};
sub01.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{4,3},2,'omitnan');
sub01.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{4,3},0,2,'omitnan');

sub01.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{5,4};
sub01.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{5,4},2,'omitnan');
sub01.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{5,4},0,2,'omitnan');
sub01.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{5,5};
sub01.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{5,5},2,'omitnan');
sub01.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{5,5},0,2,'omitnan');
sub01.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{5,6};
sub01.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{5,6},2,'omitnan');
sub01.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{5,6},0,2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{5,4};
sub01.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{5,4},2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{5,4},0,2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{5,5};
sub01.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{5,5},2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{5,5},0,2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{5,6};
sub01.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{5,6},2,'omitnan');
sub01.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{5,6},0,2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{5,4};
sub01.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{5,4},2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{5,4},0,2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{5,5};
sub01.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{5,5},2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{5,5},0,2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{5,6};
sub01.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{5,6},2,'omitnan');
sub01.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{5,6},0,2,'omitnan');

sub01.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{5,1};
sub01.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{5,1},2,'omitnan');
sub01.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{5,1},0,2,'omitnan');
sub01.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{5,2};
sub01.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{5,2},2,'omitnan');
sub01.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{5,2},0,2,'omitnan');
sub01.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{5,3};
sub01.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{5,3},2,'omitnan');
sub01.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{5,3},0,2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{5,1};
sub01.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{5,1},2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{5,1},0,2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{5,2};
sub01.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{5,2},2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{5,2},0,2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{5,3};
sub01.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{5,3},2,'omitnan');
sub01.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{5,3},0,2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{5,1};
sub01.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{5,1},2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{5,1},0,2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{5,2};
sub01.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{5,2},2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{5,2},0,2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{5,3};
sub01.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{5,3},2,'omitnan');
sub01.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{5,3},0,2,'omitnan');

sub01.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{5,1};
sub01.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{5,1},2,'omitnan');
sub01.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{5,1},0,2,'omitnan');
sub01.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{5,2};
sub01.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{5,2},2,'omitnan');
sub01.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{5,2},0,2,'omitnan');
sub01.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{5,3};
sub01.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{5,3},2,'omitnan');
sub01.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{5,3},0,2,'omitnan');


sub02.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{6,4};
sub02.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{6,4},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{6,4},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{6,5};
sub02.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{6,5},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{6,5},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{6,6};
sub02.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{6,6},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{6,6},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{6,4};
sub02.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{6,4},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{6,4},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{6,5};
sub02.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{6,5},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{6,5},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{6,6};
sub02.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{6,6},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{6,6},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{6,4};
sub02.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{6,4},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{6,4},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{6,5};
sub02.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{6,5},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{6,5},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{6,6};
sub02.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{6,6},2,'omitnan');
sub02.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{6,6},0,2,'omitnan');

sub02.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{6,1};
sub02.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{6,1},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{6,1},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{6,2};
sub02.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{6,2},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{6,2},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{6,3};
sub02.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{6,3},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{6,3},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{6,1};
sub02.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{6,1},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{6,1},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{6,2};
sub02.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{6,2},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{6,2},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{6,3};
sub02.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{6,3},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{6,3},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{6,1};
sub02.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{6,1},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{6,1},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{6,2};
sub02.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{6,2},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{6,2},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{6,3};
sub02.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{6,3},2,'omitnan');
sub02.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{6,3},0,2,'omitnan');

sub02.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{6,1};
sub02.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{6,1},2,'omitnan');
sub02.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{6,1},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{6,2};
sub02.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{6,2},2,'omitnan');
sub02.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{6,2},0,2,'omitnan');
sub02.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{6,3};
sub02.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{6,3},2,'omitnan');
sub02.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{6,3},0,2,'omitnan');



sub02.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{7,4};
sub02.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{7,4},2,'omitnan');
sub02.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{7,4},0,2,'omitnan');
sub02.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{7,5};
sub02.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{7,5},2,'omitnan');
sub02.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{7,5},0,2,'omitnan');
sub02.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{7,6};
sub02.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{7,6},2,'omitnan');
sub02.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{7,6},0,2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{7,4};
sub02.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{7,4},2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{7,4},0,2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{7,5};
sub02.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{7,5},2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{7,5},0,2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{7,6};
sub02.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{7,6},2,'omitnan');
sub02.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{7,6},0,2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{7,4};
sub02.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{7,4},2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{7,4},0,2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{7,5};
sub02.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{7,5},2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{7,5},0,2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{7,6};
sub02.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{7,6},2,'omitnan');
sub02.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{7,6},0,2,'omitnan');

sub02.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{7,1};
sub02.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{7,1},2,'omitnan');
sub02.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{7,1},0,2,'omitnan');
sub02.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{7,2};
sub02.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{7,2},2,'omitnan');
sub02.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{7,2},0,2,'omitnan');
sub02.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{7,3};
sub02.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{7,3},2,'omitnan');
sub02.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{7,3},0,2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{7,1};
sub02.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{7,1},2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{7,1},0,2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{7,2};
sub02.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{7,2},2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{7,2},0,2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{7,3};
sub02.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{7,3},2,'omitnan');
sub02.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{7,3},0,2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{7,1};
sub02.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{7,1},2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{7,1},0,2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{7,2};
sub02.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{7,2},2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{7,2},0,2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{7,3};
sub02.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{7,3},2,'omitnan');
sub02.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{7,3},0,2,'omitnan');

sub02.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{7,1};
sub02.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{7,1},2,'omitnan');
sub02.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{7,1},0,2,'omitnan');
sub02.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{7,2};
sub02.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{7,2},2,'omitnan');
sub02.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{7,2},0,2,'omitnan');
sub02.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{7,3};
sub02.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{7,3},2,'omitnan');
sub02.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{7,3},0,2,'omitnan');


sub02.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{8,4};
sub02.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{8,4},2,'omitnan');
sub02.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{8,4},0,2,'omitnan');
sub02.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{8,5};
sub02.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{8,5},2,'omitnan');
sub02.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{8,5},0,2,'omitnan');
sub02.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{8,6};
sub02.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{8,6},2,'omitnan');
sub02.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{8,6},0,2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{8,4};
sub02.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{8,4},2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{8,4},0,2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{8,5};
sub02.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{8,5},2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{8,5},0,2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{8,6};
sub02.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{8,6},2,'omitnan');
sub02.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{8,6},0,2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{8,4};
sub02.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{8,4},2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{8,4},0,2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{8,5};
sub02.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{8,5},2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{8,5},0,2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{8,6};
sub02.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{8,6},2,'omitnan');
sub02.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{8,6},0,2,'omitnan');

sub02.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{8,1};
sub02.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{8,1},2,'omitnan');
sub02.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{8,1},0,2,'omitnan');
sub02.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{8,2};
sub02.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{8,2},2,'omitnan');
sub02.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{8,2},0,2,'omitnan');
sub02.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{8,3};
sub02.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{8,3},2,'omitnan');
sub02.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{8,3},0,2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{8,1};
sub02.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{8,1},2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{8,1},0,2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{8,2};
sub02.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{8,2},2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{8,2},0,2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{8,3};
sub02.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{8,3},2,'omitnan');
sub02.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{8,3},0,2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{8,1};
sub02.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{8,1},2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{8,1},0,2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{8,2};
sub02.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{8,2},2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{8,2},0,2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{8,3};
sub02.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{8,3},2,'omitnan');
sub02.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{8,3},0,2,'omitnan');

sub02.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{8,1};
sub02.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{8,1},2,'omitnan');
sub02.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{8,1},0,2,'omitnan');
sub02.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{8,2};
sub02.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{8,2},2,'omitnan');
sub02.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{8,2},0,2,'omitnan');
sub02.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{8,3};
sub02.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{8,3},2,'omitnan');
sub02.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{8,3},0,2,'omitnan');


sub02.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{9,4};
sub02.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{9,4},2,'omitnan');
sub02.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{9,4},0,2,'omitnan');
sub02.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{9,5};
sub02.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{9,5},2,'omitnan');
sub02.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{9,5},0,2,'omitnan');
sub02.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{9,6};
sub02.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{9,6},2,'omitnan');
sub02.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{9,6},0,2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{9,4};
sub02.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{9,4},2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{9,4},0,2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{9,5};
sub02.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{9,5},2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{9,5},0,2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{9,6};
sub02.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{9,6},2,'omitnan');
sub02.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{9,6},0,2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{9,4};
sub02.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{9,4},2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{9,4},0,2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{9,5};
sub02.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{9,5},2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{9,5},0,2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{9,6};
sub02.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{9,6},2,'omitnan');
sub02.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{9,6},0,2,'omitnan');

sub02.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{9,1};
sub02.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{9,1},2,'omitnan');
sub02.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{9,1},0,2,'omitnan');
sub02.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{9,2};
sub02.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{9,2},2,'omitnan');
sub02.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{9,2},0,2,'omitnan');
sub02.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{9,3};
sub02.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{9,3},2,'omitnan');
sub02.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{9,3},0,2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{9,1};
sub02.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{9,1},2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{9,1},0,2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{9,2};
sub02.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{9,2},2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{9,2},0,2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{9,3};
sub02.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{9,3},2,'omitnan');
sub02.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{9,3},0,2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{9,1};
sub02.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{9,1},2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{9,1},0,2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{9,2};
sub02.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{9,2},2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{9,2},0,2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{9,3};
sub02.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{9,3},2,'omitnan');
sub02.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{9,3},0,2,'omitnan');

sub02.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{9,1};
sub02.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{9,1},2,'omitnan');
sub02.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{9,1},0,2,'omitnan');
sub02.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{9,2};
sub02.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{9,2},2,'omitnan');
sub02.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{9,2},0,2,'omitnan');
sub02.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{9,3};
sub02.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{9,3},2,'omitnan');
sub02.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{9,3},0,2,'omitnan');


sub02.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{10,4};
sub02.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{10,4},2,'omitnan');
sub02.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{10,4},0,2,'omitnan');
sub02.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{10,5};
sub02.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{10,5},2,'omitnan');
sub02.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{10,5},0,2,'omitnan');
sub02.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{10,6};
sub02.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{10,6},2,'omitnan');
sub02.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{10,6},0,2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{10,4};
sub02.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{10,4},2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{10,4},0,2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{10,5};
sub02.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{10,5},2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{10,5},0,2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{10,6};
sub02.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{10,6},2,'omitnan');
sub02.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{10,6},0,2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{10,4};
sub02.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{10,4},2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{10,4},0,2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{10,5};
sub02.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{10,5},2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{10,5},0,2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{10,6};
sub02.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{10,6},2,'omitnan');
sub02.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{10,6},0,2,'omitnan');

sub02.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{10,1};
sub02.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{10,1},2,'omitnan');
sub02.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{10,1},0,2,'omitnan');
sub02.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{10,2};
sub02.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{10,2},2,'omitnan');
sub02.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{10,2},0,2,'omitnan');
sub02.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{10,3};
sub02.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{10,3},2,'omitnan');
sub02.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{10,3},0,2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{10,1};
sub02.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{10,1},2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{10,1},0,2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{10,2};
sub02.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{10,2},2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{10,2},0,2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{10,3};
sub02.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{10,3},2,'omitnan');
sub02.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{10,3},0,2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{10,1};
sub02.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{10,1},2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{10,1},0,2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{10,2};
sub02.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{10,2},2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{10,2},0,2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{10,3};
sub02.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{10,3},2,'omitnan');
sub02.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{10,3},0,2,'omitnan');

sub02.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{10,1};
sub02.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{10,1},2,'omitnan');
sub02.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{10,1},0,2,'omitnan');
sub02.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{10,2};
sub02.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{10,2},2,'omitnan');
sub02.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{10,2},0,2,'omitnan');
sub02.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{10,3};
sub02.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{10,3},2,'omitnan');
sub02.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{10,3},0,2,'omitnan');

sub03.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{11,4};
sub03.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{11,4},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{11,4},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{11,5};
sub03.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{11,5},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{11,5},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{11,6};
sub03.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{11,6},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{11,6},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{11,4};
sub03.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{11,4},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{11,4},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{11,5};
sub03.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{11,5},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{11,5},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{11,6};
sub03.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{11,6},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{11,6},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{11,4};
sub03.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{11,4},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{11,4},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{11,5};
sub03.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{11,5},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{11,5},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{11,6};
sub03.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{11,6},2,'omitnan');
sub03.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{11,6},0,2,'omitnan');

sub03.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{11,1};
sub03.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{11,1},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{11,1},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{11,2};
sub03.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{11,2},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{11,2},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{11,3};
sub03.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{11,3},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{11,3},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{11,1};
sub03.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{11,1},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{11,1},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{11,2};
sub03.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{11,2},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{11,2},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{11,3};
sub03.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{11,3},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{11,3},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{11,1};
sub03.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{11,1},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{11,1},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{11,2};
sub03.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{11,2},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{11,2},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{11,3};
sub03.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{11,3},2,'omitnan');
sub03.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{11,3},0,2,'omitnan');

sub03.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{11,1};
sub03.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{11,1},2,'omitnan');
sub03.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{11,1},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{11,2};
sub03.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{11,2},2,'omitnan');
sub03.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{11,2},0,2,'omitnan');
sub03.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{11,3};
sub03.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{11,3},2,'omitnan');
sub03.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{11,3},0,2,'omitnan');



sub03.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{12,4};
sub03.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{12,4},2,'omitnan');
sub03.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{12,4},0,2,'omitnan');
sub03.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{12,5};
sub03.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{12,5},2,'omitnan');
sub03.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{12,5},0,2,'omitnan');
sub03.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{12,6};
sub03.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{12,6},2,'omitnan');
sub03.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{12,6},0,2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{12,4};
sub03.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{12,4},2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{12,4},0,2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{12,5};
sub03.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{12,5},2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{12,5},0,2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{12,6};
sub03.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{12,6},2,'omitnan');
sub03.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{12,6},0,2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{12,4};
sub03.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{12,4},2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{12,4},0,2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{12,5};
sub03.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{12,5},2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{12,5},0,2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{12,6};
sub03.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{12,6},2,'omitnan');
sub03.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{12,6},0,2,'omitnan');

sub03.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{12,1};
sub03.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{12,1},2,'omitnan');
sub03.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{12,1},0,2,'omitnan');
sub03.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{12,2};
sub03.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{12,2},2,'omitnan');
sub03.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{12,2},0,2,'omitnan');
sub03.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{12,3};
sub03.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{12,3},2,'omitnan');
sub03.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{12,3},0,2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{12,1};
sub03.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{12,1},2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{12,1},0,2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{12,2};
sub03.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{12,2},2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{12,2},0,2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{12,3};
sub03.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{12,3},2,'omitnan');
sub03.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{12,3},0,2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{12,1};
sub03.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{12,1},2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{12,1},0,2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{12,2};
sub03.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{12,2},2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{12,2},0,2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{12,3};
sub03.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{12,3},2,'omitnan');
sub03.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{12,3},0,2,'omitnan');

sub03.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{12,1};
sub03.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{12,1},2,'omitnan');
sub03.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{12,1},0,2,'omitnan');
sub03.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{12,2};
sub03.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{12,2},2,'omitnan');
sub03.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{12,2},0,2,'omitnan');
sub03.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{12,3};
sub03.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{12,3},2,'omitnan');
sub03.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{12,3},0,2,'omitnan');


sub03.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{13,4};
sub03.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{13,4},2,'omitnan');
sub03.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{13,4},0,2,'omitnan');
sub03.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{13,5};
sub03.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{13,5},2,'omitnan');
sub03.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{13,5},0,2,'omitnan');
sub03.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{13,6};
sub03.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{13,6},2,'omitnan');
sub03.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{13,6},0,2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{13,4};
sub03.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{13,4},2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{13,4},0,2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{13,5};
sub03.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{13,5},2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{13,5},0,2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{13,6};
sub03.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{13,6},2,'omitnan');
sub03.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{13,6},0,2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{13,4};
sub03.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{13,4},2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{13,4},0,2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{13,5};
sub03.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{13,5},2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{13,5},0,2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{13,6};
sub03.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{13,6},2,'omitnan');
sub03.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{13,6},0,2,'omitnan');

sub03.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{13,1};
sub03.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{13,1},2,'omitnan');
sub03.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{13,1},0,2,'omitnan');
sub03.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{13,2};
sub03.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{13,2},2,'omitnan');
sub03.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{13,2},0,2,'omitnan');
sub03.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{13,3};
sub03.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{13,3},2,'omitnan');
sub03.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{13,3},0,2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{13,1};
sub03.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{13,1},2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{13,1},0,2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{13,2};
sub03.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{13,2},2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{13,2},0,2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{13,3};
sub03.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{13,3},2,'omitnan');
sub03.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{13,3},0,2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{13,1};
sub03.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{13,1},2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{13,1},0,2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{13,2};
sub03.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{13,2},2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{13,2},0,2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{13,3};
sub03.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{13,3},2,'omitnan');
sub03.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{13,3},0,2,'omitnan');

sub03.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{13,1};
sub03.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{13,1},2,'omitnan');
sub03.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{13,1},0,2,'omitnan');
sub03.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{13,2};
sub03.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{13,2},2,'omitnan');
sub03.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{13,2},0,2,'omitnan');
sub03.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{13,3};
sub03.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{13,3},2,'omitnan');
sub03.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{13,3},0,2,'omitnan');


sub07.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{14,4};
sub03.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{14,4},2,'omitnan');
sub03.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{14,4},0,2,'omitnan');
sub03.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{14,5};
sub03.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{14,5},2,'omitnan');
sub03.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{14,5},0,2,'omitnan');
sub03.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{14,6};
sub03.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{14,6},2,'omitnan');
sub03.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{14,6},0,2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{14,4};
sub03.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{14,4},2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{14,4},0,2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{14,5};
sub03.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{14,5},2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{14,5},0,2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{14,6};
sub03.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{14,6},2,'omitnan');
sub03.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{14,6},0,2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{14,4};
sub03.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{14,4},2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{14,4},0,2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{14,5};
sub03.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{14,5},2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{14,5},0,2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{14,6};
sub03.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{14,6},2,'omitnan');
sub03.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{14,6},0,2,'omitnan');

sub03.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{14,1};
sub03.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{14,1},2,'omitnan');
sub03.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{14,1},0,2,'omitnan');
sub03.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{14,2};
sub03.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{14,2},2,'omitnan');
sub03.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{14,2},0,2,'omitnan');
sub03.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{14,3};
sub03.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{14,3},2,'omitnan');
sub03.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{14,3},0,2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{14,1};
sub03.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{14,1},2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{14,1},0,2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{14,2};
sub03.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{14,2},2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{14,2},0,2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{14,3};
sub03.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{14,3},2,'omitnan');
sub03.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{14,3},0,2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{14,1};
sub03.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{14,1},2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{14,1},0,2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{14,2};
sub03.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{14,2},2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{14,2},0,2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{14,3};
sub03.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{14,3},2,'omitnan');
sub03.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{14,3},0,2,'omitnan');

sub03.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{14,1};
sub03.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{14,1},2,'omitnan');
sub03.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{14,1},0,2,'omitnan');
sub03.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{14,2};
sub03.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{14,2},2,'omitnan');
sub03.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{14,2},0,2,'omitnan');
sub03.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{14,3};
sub03.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{14,3},2,'omitnan');
sub03.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{14,3},0,2,'omitnan');


sub03.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{15,4};
sub03.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{15,4},2,'omitnan');
sub03.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{15,4},0,2,'omitnan');
sub03.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{15,5};
sub03.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{15,5},2,'omitnan');
sub03.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{15,5},0,2,'omitnan');
sub03.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{15,6};
sub03.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{15,6},2,'omitnan');
sub03.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{15,6},0,2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{15,4};
sub03.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{15,4},2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{15,4},0,2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{15,5};
sub03.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{15,5},2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{15,5},0,2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{15,6};
sub03.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{15,6},2,'omitnan');
sub03.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{15,6},0,2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{15,4};
sub03.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{15,4},2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{15,4},0,2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{15,5};
sub03.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{15,5},2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{15,5},0,2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{15,6};
sub03.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{15,6},2,'omitnan');
sub03.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{15,6},0,2,'omitnan');

sub03.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{15,1};
sub03.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{15,1},2,'omitnan');
sub03.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{15,1},0,2,'omitnan');
sub03.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{15,2};
sub03.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{15,2},2,'omitnan');
sub03.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{15,2},0,2,'omitnan');
sub03.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{15,3};
sub03.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{15,3},2,'omitnan');
sub03.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{15,3},0,2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{15,1};
sub03.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{15,1},2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{15,1},0,2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{15,2};
sub03.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{15,2},2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{15,2},0,2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{15,3};
sub03.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{15,3},2,'omitnan');
sub03.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{15,3},0,2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{15,1};
sub03.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{15,1},2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{15,1},0,2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{15,2};
sub03.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{15,2},2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{15,2},0,2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{15,3};
sub03.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{15,3},2,'omitnan');
sub03.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{15,3},0,2,'omitnan');

sub03.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{15,1};
sub03.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{15,1},2,'omitnan');
sub03.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{15,1},0,2,'omitnan');
sub03.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{15,2};
sub03.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{15,2},2,'omitnan');
sub03.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{15,2},0,2,'omitnan');
sub03.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{15,3};
sub03.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{15,3},2,'omitnan');
sub03.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{15,3},0,2,'omitnan');


sub04.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{16,4};
sub04.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{16,4},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{16,4},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{16,5};
sub04.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{16,5},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{16,5},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{16,6};
sub04.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{16,6},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{16,6},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{16,4};
sub04.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{16,4},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{16,4},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{16,5};
sub04.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{16,5},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{16,5},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{16,6};
sub04.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{16,6},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{16,6},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{16,4};
sub04.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{16,4},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{16,4},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{16,5};
sub04.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{16,5},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{16,5},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{16,6};
sub04.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{16,6},2,'omitnan');
sub04.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{16,6},0,2,'omitnan');

sub04.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{16,1};
sub04.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{16,1},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{16,1},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{16,2};
sub04.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{16,2},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{16,2},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{16,3};
sub04.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{16,3},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{16,3},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{16,1};
sub04.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{16,1},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{16,1},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{16,2};
sub04.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{16,2},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{16,2},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{16,3};
sub04.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{16,3},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{16,3},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{16,1};
sub04.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{16,1},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{16,1},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{16,2};
sub04.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{16,2},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{16,2},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{16,3};
sub04.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{16,3},2,'omitnan');
sub04.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{16,3},0,2,'omitnan');

sub04.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{16,1};
sub04.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{16,1},2,'omitnan');
sub04.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{16,1},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{16,2};
sub04.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{16,2},2,'omitnan');
sub04.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{16,2},0,2,'omitnan');
sub04.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{16,3};
sub04.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{16,3},2,'omitnan');
sub04.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{16,3},0,2,'omitnan');

sub04.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{17,4};
sub04.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{17,4},2,'omitnan');
sub04.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{17,4},0,2,'omitnan');
sub04.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{17,5};
sub04.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{17,5},2,'omitnan');
sub04.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{17,5},0,2,'omitnan');
sub04.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{17,6};
sub04.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{17,6},2,'omitnan');
sub04.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{17,6},0,2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{17,4};
sub04.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{17,4},2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{17,4},0,2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{17,5};
sub04.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{17,5},2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{17,5},0,2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{17,6};
sub04.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{17,6},2,'omitnan');
sub04.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{17,6},0,2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{17,4};
sub04.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{17,4},2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{17,4},0,2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{17,5};
sub04.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{17,5},2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{17,5},0,2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{17,6};
sub04.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{17,6},2,'omitnan');
sub04.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{17,6},0,2,'omitnan');

sub04.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{17,1};
sub04.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{17,1},2,'omitnan');
sub04.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{17,1},0,2,'omitnan');
sub04.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{17,2};
sub04.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{17,2},2,'omitnan');
sub04.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{17,2},0,2,'omitnan');
sub04.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{17,3};
sub04.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{17,3},2,'omitnan');
sub04.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{17,3},0,2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{17,1};
sub04.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{17,1},2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{17,1},0,2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{17,2};
sub04.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{17,2},2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{17,2},0,2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{17,3};
sub04.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{17,3},2,'omitnan');
sub04.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{17,3},0,2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{17,1};
sub04.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{17,1},2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{17,1},0,2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{17,2};
sub04.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{17,2},2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{17,2},0,2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{17,3};
sub04.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{17,3},2,'omitnan');
sub04.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{17,3},0,2,'omitnan');

sub04.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{17,1};
sub04.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{17,1},2,'omitnan');
sub04.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{17,1},0,2,'omitnan');
sub04.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{17,2};
sub04.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{17,2},2,'omitnan');
sub04.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{17,2},0,2,'omitnan');
sub04.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{17,3};
sub04.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{17,3},2,'omitnan');
sub04.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{17,3},0,2,'omitnan');



sub04.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{18,4};
sub04.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{18,4},2,'omitnan');
sub04.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{18,4},0,2,'omitnan');
sub04.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{18,5};
sub04.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{18,5},2,'omitnan');
sub04.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{18,5},0,2,'omitnan');
sub04.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{18,6};
sub04.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{18,6},2,'omitnan');
sub04.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{18,6},0,2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{18,4};
sub04.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{18,4},2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{18,4},0,2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{18,5};
sub04.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{18,5},2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{18,5},0,2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{18,6};
sub04.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{18,6},2,'omitnan');
sub04.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{18,6},0,2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{18,4};
sub04.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{18,4},2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{18,4},0,2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{18,5};
sub04.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{18,5},2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{18,5},0,2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{18,6};
sub04.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{18,6},2,'omitnan');
sub04.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{18,6},0,2,'omitnan');

sub04.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{18,1};
sub04.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{18,1},2,'omitnan');
sub04.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{18,1},0,2,'omitnan');
sub04.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{18,2};
sub04.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{18,2},2,'omitnan');
sub04.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{18,2},0,2,'omitnan');
sub04.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{18,3};
sub04.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{18,3},2,'omitnan');
sub04.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{18,3},0,2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{18,1};
sub04.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{18,1},2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{18,1},0,2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{18,2};
sub04.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{18,2},2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{18,2},0,2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{18,3};
sub04.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{18,3},2,'omitnan');
sub04.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{18,3},0,2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{18,1};
sub04.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{18,1},2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{18,1},0,2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{18,2};
sub04.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{18,2},2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{18,2},0,2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{18,3};
sub04.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{18,3},2,'omitnan');
sub04.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{18,3},0,2,'omitnan');

sub04.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{18,1};
sub04.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{18,1},2,'omitnan');
sub04.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{18,1},0,2,'omitnan');
sub04.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{18,2};
sub04.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{18,2},2,'omitnan');
sub04.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{18,2},0,2,'omitnan');
sub04.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{18,3};
sub04.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{18,3},2,'omitnan');
sub04.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{18,3},0,2,'omitnan');


sub04.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{19,4};
sub04.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{19,4},2,'omitnan');
sub04.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{19,4},0,2,'omitnan');
sub04.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{19,5};
sub04.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{19,5},2,'omitnan');
sub04.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{19,5},0,2,'omitnan');
sub04.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{19,6};
sub04.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{19,6},2,'omitnan');
sub04.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{19,6},0,2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{19,4};
sub04.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{19,4},2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{19,4},0,2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{19,5};
sub04.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{19,5},2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{19,5},0,2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{19,6};
sub04.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{19,6},2,'omitnan');
sub04.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{19,6},0,2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{19,4};
sub04.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{19,4},2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{19,4},0,2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{19,5};
sub04.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{19,5},2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{19,5},0,2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{19,6};
sub04.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{19,6},2,'omitnan');
sub04.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{19,6},0,2,'omitnan');

sub04.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{19,1};
sub04.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{19,1},2,'omitnan');
sub04.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{19,1},0,2,'omitnan');
sub04.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{19,2};
sub04.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{19,2},2,'omitnan');
sub04.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{19,2},0,2,'omitnan');
sub04.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{19,3};
sub04.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{19,3},2,'omitnan');
sub04.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{19,3},0,2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{19,1};
sub04.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{19,1},2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{19,1},0,2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{19,2};
sub04.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{19,2},2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{19,2},0,2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{19,3};
sub04.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{19,3},2,'omitnan');
sub04.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{19,3},0,2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{19,1};
sub04.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{19,1},2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{19,1},0,2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{19,2};
sub04.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{19,2},2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{19,2},0,2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{19,3};
sub04.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{19,3},2,'omitnan');
sub04.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{19,3},0,2,'omitnan');

sub04.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{19,1};
sub04.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{19,1},2,'omitnan');
sub04.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{19,1},0,2,'omitnan');
sub04.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{19,2};
sub04.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{19,2},2,'omitnan');
sub04.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{19,2},0,2,'omitnan');
sub04.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{19,3};
sub04.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{19,3},2,'omitnan');
sub04.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{19,3},0,2,'omitnan');



sub04.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{20,4};
sub04.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{20,4},2,'omitnan');
sub04.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{20,4},0,2,'omitnan');
sub04.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{20,5};
sub04.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{20,5},2,'omitnan');
sub04.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{20,5},0,2,'omitnan');
sub04.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{20,6};
sub04.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{20,6},2,'omitnan');
sub04.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{20,6},0,2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{20,4};
sub04.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{20,4},2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{20,4},0,2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{20,5};
sub04.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{20,5},2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{20,5},0,2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{20,6};
sub04.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{20,6},2,'omitnan');
sub04.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{20,6},0,2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{20,4};
sub04.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{20,4},2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{20,4},0,2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{20,5};
sub04.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{20,5},2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{20,5},0,2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{20,6};
sub04.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{20,6},2,'omitnan');
sub04.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{20,6},0,2,'omitnan');

sub04.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{20,1};
sub04.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{20,1},2,'omitnan');
sub04.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{20,1},0,2,'omitnan');
sub04.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{20,2};
sub04.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{20,2},2,'omitnan');
sub04.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{20,2},0,2,'omitnan');
sub04.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{20,3};
sub04.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{20,3},2,'omitnan');
sub04.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{20,3},0,2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{20,1};
sub04.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{20,1},2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{20,1},0,2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{20,2};
sub04.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{20,2},2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{20,2},0,2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{20,3};
sub04.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{20,3},2,'omitnan');
sub04.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{20,3},0,2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{20,1};
sub04.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{20,1},2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{20,1},0,2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{20,2};
sub04.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{20,2},2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{20,2},0,2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{20,3};
sub04.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{20,3},2,'omitnan');
sub04.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{20,3},0,2,'omitnan');

sub04.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{20,1};
sub04.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{20,1},2,'omitnan');
sub04.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{20,1},0,2,'omitnan');
sub04.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{20,2};
sub04.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{20,2},2,'omitnan');
sub04.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{20,2},0,2,'omitnan');
sub04.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{20,3};
sub04.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{20,3},2,'omitnan');
sub04.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{20,3},0,2,'omitnan');



sub05.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{21,4};
sub05.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{21,4},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{21,4},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{21,5};
sub05.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{21,5},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{21,5},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{21,6};
sub05.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{21,6},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{21,6},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{21,4};
sub05.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{21,4},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{21,4},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{21,5};
sub05.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{21,5},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{21,5},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{21,6};
sub05.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{21,6},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{21,6},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{21,4};
sub05.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{21,4},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{21,4},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{21,5};
sub05.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{21,5},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{21,5},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{21,6};
sub05.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{21,6},2,'omitnan');
sub05.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{21,6},0,2,'omitnan');

sub05.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{21,1};
sub05.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{21,1},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{21,1},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{21,2};
sub05.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{21,2},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{21,2},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{21,3};
sub05.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{21,3},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{21,3},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{21,1};
sub05.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{21,1},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{21,1},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{21,2};
sub05.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{21,2},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{21,2},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{21,3};
sub05.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{21,3},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{21,3},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{21,1};
sub05.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{21,1},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{21,1},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{21,2};
sub05.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{21,2},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{21,2},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{21,3};
sub05.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{21,3},2,'omitnan');
sub05.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{21,3},0,2,'omitnan');

sub05.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{21,1};
sub05.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{21,1},2,'omitnan');
sub05.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{21,1},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{21,2};
sub05.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{21,2},2,'omitnan');
sub05.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{21,2},0,2,'omitnan');
sub05.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{21,3};
sub05.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{21,3},2,'omitnan');
sub05.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{21,3},0,2,'omitnan');



sub05.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{22,4};
sub05.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{22,4},2,'omitnan');
sub05.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{22,4},0,2,'omitnan');
sub05.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{22,5};
sub05.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{22,5},2,'omitnan');
sub05.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{22,5},0,2,'omitnan');
sub05.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{22,6};
sub05.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{22,6},2,'omitnan');
sub05.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{22,6},0,2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{22,4};
sub05.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{22,4},2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{22,4},0,2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{22,5};
sub05.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{22,5},2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{22,5},0,2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{22,6};
sub05.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{22,6},2,'omitnan');
sub05.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{22,6},0,2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{22,4};
sub05.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{22,4},2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{22,4},0,2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{22,5};
sub05.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{22,5},2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{22,5},0,2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{22,6};
sub05.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{22,6},2,'omitnan');
sub05.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{22,6},0,2,'omitnan');

sub05.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{22,1};
sub05.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{22,1},2,'omitnan');
sub05.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{22,1},0,2,'omitnan');
sub05.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{22,2};
sub05.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{22,2},2,'omitnan');
sub05.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{22,2},0,2,'omitnan');
sub05.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{22,3};
sub05.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{22,3},2,'omitnan');
sub05.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{22,3},0,2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{22,1};
sub05.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{22,1},2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{22,1},0,2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{22,2};
sub05.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{22,2},2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{22,2},0,2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{22,3};
sub05.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{22,3},2,'omitnan');
sub05.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{22,3},0,2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{22,1};
sub05.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{22,1},2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{22,1},0,2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{22,2};
sub05.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{22,2},2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{22,2},0,2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{22,3};
sub05.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{22,3},2,'omitnan');
sub05.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{22,3},0,2,'omitnan');

sub05.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{22,1};
sub05.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{22,1},2,'omitnan');
sub05.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{22,1},0,2,'omitnan');
sub05.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{22,2};
sub05.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{22,2},2,'omitnan');
sub05.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{22,2},0,2,'omitnan');
sub05.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{22,3};
sub05.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{22,3},2,'omitnan');
sub05.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{22,3},0,2,'omitnan');



sub05.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{23,4};
sub05.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{23,4},2,'omitnan');
sub05.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{23,4},0,2,'omitnan');
sub05.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{23,5};
sub05.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{23,5},2,'omitnan');
sub05.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{23,5},0,2,'omitnan');
sub05.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{23,6};
sub05.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{23,6},2,'omitnan');
sub05.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{23,6},0,2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{23,4};
sub05.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{23,4},2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{23,4},0,2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{23,5};
sub05.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{23,5},2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{23,5},0,2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{23,6};
sub05.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{23,6},2,'omitnan');
sub05.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{23,6},0,2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{23,4};
sub05.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{23,4},2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{23,4},0,2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{23,5};
sub05.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{23,5},2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{23,5},0,2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{23,6};
sub05.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{23,6},2,'omitnan');
sub05.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{23,6},0,2,'omitnan');

sub05.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{23,1};
sub05.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{23,1},2,'omitnan');
sub05.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{23,1},0,2,'omitnan');
sub05.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{23,2};
sub05.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{23,2},2,'omitnan');
sub05.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{23,2},0,2,'omitnan');
sub05.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{23,3};
sub05.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{23,3},2,'omitnan');
sub05.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{23,3},0,2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{23,1};
sub05.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{23,1},2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{23,1},0,2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{23,2};
sub05.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{23,2},2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{23,2},0,2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{23,3};
sub05.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{23,3},2,'omitnan');
sub05.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{23,3},0,2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{23,1};
sub05.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{23,1},2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{23,1},0,2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{23,2};
sub05.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{23,2},2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{23,2},0,2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{23,3};
sub05.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{23,3},2,'omitnan');
sub05.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{23,3},0,2,'omitnan');

sub05.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{23,1};
sub05.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{23,1},2,'omitnan');
sub05.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{23,1},0,2,'omitnan');
sub05.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{23,2};
sub05.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{23,2},2,'omitnan');
sub05.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{23,2},0,2,'omitnan');
sub05.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{23,3};
sub05.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{23,3},2,'omitnan');
sub05.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{23,3},0,2,'omitnan');



sub05.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{24,4};
sub05.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{24,4},2,'omitnan');
sub05.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{24,4},0,2,'omitnan');
sub05.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{24,5};
sub05.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{24,5},2,'omitnan');
sub05.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{24,5},0,2,'omitnan');
sub05.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{24,6};
sub05.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{24,6},2,'omitnan');
sub05.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{24,6},0,2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{24,4};
sub05.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{24,4},2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{24,4},0,2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{24,5};
sub05.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{24,5},2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{24,5},0,2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{24,6};
sub05.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{24,6},2,'omitnan');
sub05.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{24,6},0,2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{24,4};
sub05.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{24,4},2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{24,4},0,2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{24,5};
sub05.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{24,5},2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{24,5},0,2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{24,6};
sub05.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{24,6},2,'omitnan');
sub05.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{24,6},0,2,'omitnan');

sub05.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{24,1};
sub05.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{24,1},2,'omitnan');
sub05.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{24,1},0,2,'omitnan');
sub05.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{24,2};
sub05.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{24,2},2,'omitnan');
sub05.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{24,2},0,2,'omitnan');
sub05.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{24,3};
sub05.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{24,3},2,'omitnan');
sub05.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{24,3},0,2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{24,1};
sub05.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{24,1},2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{24,1},0,2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{24,2};
sub05.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{24,2},2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{24,2},0,2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{24,3};
sub05.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{24,3},2,'omitnan');
sub05.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{24,3},0,2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{24,1};
sub05.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{24,1},2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{24,1},0,2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{24,2};
sub05.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{24,2},2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{24,2},0,2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{24,3};
sub05.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{24,3},2,'omitnan');
sub05.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{24,3},0,2,'omitnan');

sub05.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{24,1};
sub05.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{24,1},2,'omitnan');
sub05.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{24,1},0,2,'omitnan');
sub05.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{24,2};
sub05.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{24,2},2,'omitnan');
sub05.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{24,2},0,2,'omitnan');
sub05.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{24,3};
sub05.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{24,3},2,'omitnan');
sub05.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{24,3},0,2,'omitnan');



sub05.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{25,4};
sub05.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{25,4},2,'omitnan');
sub05.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{25,4},0,2,'omitnan');
sub05.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{25,5};
sub05.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{25,5},2,'omitnan');
sub05.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{25,5},0,2,'omitnan');
sub05.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{25,6};
sub05.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{25,6},2,'omitnan');
sub05.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{25,6},0,2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{25,4};
sub05.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{25,4},2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{25,4},0,2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{25,5};
sub05.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{25,5},2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{25,5},0,2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{25,6};
sub05.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{25,6},2,'omitnan');
sub05.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{25,6},0,2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{25,4};
sub05.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{25,4},2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{25,4},0,2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{25,5};
sub05.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{25,5},2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{25,5},0,2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{25,6};
sub05.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{25,6},2,'omitnan');
sub05.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{25,6},0,2,'omitnan');

sub05.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{25,1};
sub05.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{25,1},2,'omitnan');
sub05.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{25,1},0,2,'omitnan');
sub05.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{25,2};
sub05.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{25,2},2,'omitnan');
sub05.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{25,2},0,2,'omitnan');
sub05.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{25,3};
sub05.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{25,3},2,'omitnan');
sub05.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{25,3},0,2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{25,1};
sub05.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{25,1},2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{25,1},0,2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{25,2};
sub05.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{25,2},2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{25,2},0,2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{25,3};
sub05.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{25,3},2,'omitnan');
sub05.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{25,3},0,2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{25,1};
sub05.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{25,1},2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{25,1},0,2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{25,2};
sub05.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{25,2},2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{25,2},0,2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{25,3};
sub05.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{25,3},2,'omitnan');
sub05.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{25,3},0,2,'omitnan');

sub05.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{25,1};
sub05.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{25,1},2,'omitnan');
sub05.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{25,1},0,2,'omitnan');
sub05.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{25,2};
sub05.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{25,2},2,'omitnan');
sub05.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{25,2},0,2,'omitnan');
sub05.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{25,3};
sub05.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{25,3},2,'omitnan');
sub05.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{25,3},0,2,'omitnan');



sub06.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{26,4};
sub06.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{26,4},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{26,4},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{26,5};
sub06.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{26,5},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{26,5},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{26,6};
sub06.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{26,6},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{26,6},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{26,4};
sub06.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{26,4},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{26,4},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{26,5};
sub06.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{26,5},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{26,5},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{26,6};
sub06.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{26,6},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{26,6},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{26,4};
sub06.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{26,4},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{26,4},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{26,5};
sub06.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{26,5},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{26,5},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{26,6};
sub06.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{26,6},2,'omitnan');
sub06.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{26,6},0,2,'omitnan');

sub06.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{26,1};
sub06.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{26,1},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{26,1},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{26,2};
sub06.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{26,2},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{26,2},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{26,3};
sub06.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{26,3},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{26,3},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{26,1};
sub06.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{26,1},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{26,1},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{26,2};
sub06.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{26,2},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{26,2},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{26,3};
sub06.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{26,3},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{26,3},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{26,1};
sub06.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{26,1},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{26,1},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{26,2};
sub06.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{26,2},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{26,2},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{26,3};
sub06.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{26,3},2,'omitnan');
sub06.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{26,3},0,2,'omitnan');

sub06.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{26,1};
sub06.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{26,1},2,'omitnan');
sub06.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{26,1},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{26,2};
sub06.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{26,2},2,'omitnan');
sub06.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{26,2},0,2,'omitnan');
sub06.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{26,3};
sub06.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{26,3},2,'omitnan');
sub06.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{26,3},0,2,'omitnan');



sub07.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{27,4};
sub06.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{27,4},2,'omitnan');
sub06.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{27,4},0,2,'omitnan');
sub06.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{27,5};
sub06.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{27,5},2,'omitnan');
sub06.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{27,5},0,2,'omitnan');
sub06.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{27,6};
sub06.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{27,6},2,'omitnan');
sub06.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{27,6},0,2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{27,4};
sub06.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{27,4},2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{27,4},0,2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{27,5};
sub06.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{27,5},2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{27,5},0,2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{27,6};
sub06.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{27,6},2,'omitnan');
sub06.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{27,6},0,2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{27,4};
sub06.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{27,4},2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{27,4},0,2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{27,5};
sub06.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{27,5},2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{27,5},0,2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{27,6};
sub06.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{27,6},2,'omitnan');
sub06.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{27,6},0,2,'omitnan');

sub06.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{27,1};
sub06.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{27,1},2,'omitnan');
sub06.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{27,1},0,2,'omitnan');
sub06.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{27,2};
sub06.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{27,2},2,'omitnan');
sub06.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{27,2},0,2,'omitnan');
sub06.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{27,3};
sub06.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{27,3},2,'omitnan');
sub06.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{27,3},0,2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{27,1};
sub06.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{27,1},2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{27,1},0,2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{27,2};
sub06.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{27,2},2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{27,2},0,2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{27,3};
sub06.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{27,3},2,'omitnan');
sub06.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{27,3},0,2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{27,1};
sub06.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{27,1},2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{27,1},0,2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{27,2};
sub06.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{27,2},2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{27,2},0,2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{27,3};
sub06.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{27,3},2,'omitnan');
sub06.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{27,3},0,2,'omitnan');

sub06.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{27,1};
sub06.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{27,1},2,'omitnan');
sub06.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{27,1},0,2,'omitnan');
sub06.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{27,2};
sub06.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{27,2},2,'omitnan');
sub06.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{27,2},0,2,'omitnan');
sub06.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{27,3};
sub06.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{27,3},2,'omitnan');
sub06.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{27,3},0,2,'omitnan');



sub06.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{28,4};
sub06.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{28,4},2,'omitnan');
sub06.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{28,4},0,2,'omitnan');
sub06.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{28,5};
sub06.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{28,5},2,'omitnan');
sub06.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{28,5},0,2,'omitnan');
sub06.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{28,6};
sub06.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{28,6},2,'omitnan');
sub06.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{28,6},0,2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{28,4};
sub06.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{28,4},2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{28,4},0,2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{28,5};
sub06.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{28,5},2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{28,5},0,2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{28,6};
sub06.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{28,6},2,'omitnan');
sub06.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{28,6},0,2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{28,4};
sub06.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{28,4},2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{28,4},0,2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{28,5};
sub06.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{28,5},2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{28,5},0,2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{28,6};
sub06.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{28,6},2,'omitnan');
sub06.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{28,6},0,2,'omitnan');

sub06.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{28,1};
sub06.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{28,1},2,'omitnan');
sub06.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{28,1},0,2,'omitnan');
sub06.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{28,2};
sub06.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{28,2},2,'omitnan');
sub06.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{28,2},0,2,'omitnan');
sub06.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{28,3};
sub06.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{28,3},2,'omitnan');
sub06.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{28,3},0,2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{28,1};
sub06.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{28,1},2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{28,1},0,2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{28,2};
sub06.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{28,2},2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{28,2},0,2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{28,3};
sub06.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{28,3},2,'omitnan');
sub06.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{28,3},0,2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{28,1};
sub06.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{28,1},2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{28,1},0,2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{28,2};
sub06.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{28,2},2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{28,2},0,2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{28,3};
sub06.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{28,3},2,'omitnan');
sub06.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{28,3},0,2,'omitnan');

sub06.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{28,1};
sub06.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{28,1},2,'omitnan');
sub06.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{28,1},0,2,'omitnan');
sub06.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{28,2};
sub06.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{28,2},2,'omitnan');
sub06.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{28,2},0,2,'omitnan');
sub06.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{28,3};
sub06.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{28,3},2,'omitnan');
sub06.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{28,3},0,2,'omitnan');



sub06.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{29,4};
sub06.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{29,4},2,'omitnan');
sub06.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{29,4},0,2,'omitnan');
sub06.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{29,5};
sub06.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{29,5},2,'omitnan');
sub06.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{29,5},0,2,'omitnan');
sub06.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{29,6};
sub06.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{29,6},2,'omitnan');
sub06.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{29,6},0,2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{29,4};
sub06.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{29,4},2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{29,4},0,2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{29,5};
sub06.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{29,5},2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{29,5},0,2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{29,6};
sub06.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{29,6},2,'omitnan');
sub06.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{29,6},0,2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{29,4};
sub06.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{29,4},2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{29,4},0,2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{29,5};
sub06.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{29,5},2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{29,5},0,2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{29,6};
sub06.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{29,6},2,'omitnan');
sub06.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{29,6},0,2,'omitnan');

sub06.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{29,1};
sub06.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{29,1},2,'omitnan');
sub06.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{29,1},0,2,'omitnan');
sub06.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{29,2};
sub06.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{29,2},2,'omitnan');
sub06.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{29,2},0,2,'omitnan');
sub06.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{29,3};
sub06.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{29,3},2,'omitnan');
sub06.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{29,3},0,2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{29,1};
sub06.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{29,1},2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{29,1},0,2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{29,2};
sub06.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{29,2},2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{29,2},0,2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{29,3};
sub06.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{29,3},2,'omitnan');
sub06.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{29,3},0,2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{29,1};
sub06.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{29,1},2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{29,1},0,2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{29,2};
sub06.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{29,2},2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{29,2},0,2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{29,3};
sub06.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{29,3},2,'omitnan');
sub06.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{29,3},0,2,'omitnan');

sub06.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{29,1};
sub06.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{29,1},2,'omitnan');
sub06.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{29,1},0,2,'omitnan');
sub06.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{29,2};
sub06.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{29,2},2,'omitnan');
sub06.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{29,2},0,2,'omitnan');
sub06.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{29,3};
sub06.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{29,3},2,'omitnan');
sub06.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{29,3},0,2,'omitnan');



sub06.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{30,4};
sub06.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{30,4},2,'omitnan');
sub06.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{30,4},0,2,'omitnan');
sub06.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{30,5};
sub06.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{30,5},2,'omitnan');
sub06.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{30,5},0,2,'omitnan');
sub06.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{30,6};
sub06.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{30,6},2,'omitnan');
sub06.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{30,6},0,2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{30,4};
sub06.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{30,4},2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{30,4},0,2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{30,5};
sub06.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{30,5},2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{30,5},0,2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{30,6};
sub06.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{30,6},2,'omitnan');
sub06.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{30,6},0,2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{30,4};
sub06.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{30,4},2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{30,4},0,2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{30,5};
sub06.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{30,5},2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{30,5},0,2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{30,6};
sub06.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{30,6},2,'omitnan');
sub06.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{30,6},0,2,'omitnan');

sub06.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{30,1};
sub06.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{30,1},2,'omitnan');
sub06.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{30,1},0,2,'omitnan');
sub06.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{30,2};
sub06.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{30,2},2,'omitnan');
sub06.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{30,2},0,2,'omitnan');
sub06.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{30,3};
sub06.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{30,3},2,'omitnan');
sub06.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{30,3},0,2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{30,1};
sub06.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{30,1},2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{30,1},0,2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{30,2};
sub06.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{30,2},2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{30,2},0,2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{30,3};
sub06.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{30,3},2,'omitnan');
sub06.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{30,3},0,2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{30,1};
sub06.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{30,1},2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{30,1},0,2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{30,2};
sub06.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{30,2},2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{30,2},0,2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{30,3};
sub06.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{30,3},2,'omitnan');
sub06.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{30,3},0,2,'omitnan');

sub06.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{30,1};
sub06.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{30,1},2,'omitnan');
sub06.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{30,1},0,2,'omitnan');
sub06.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{30,2};
sub06.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{30,2},2,'omitnan');
sub06.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{30,2},0,2,'omitnan');
sub06.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{30,3};
sub06.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{30,3},2,'omitnan');
sub06.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{30,3},0,2,'omitnan');



sub07.Normalwalkig.kinematics.Left.ANK_angle.x.raw = A_ANK{31,4};
sub07.Normalwalkig.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{31,4},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.ANK_angle.x.std = std(A_ANK{31,4},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.ANK_angle.y.raw = A_ANK{31,5};
sub07.Normalwalkig.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{31,5},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.ANK_angle.y.std = std(A_ANK{31,5},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.ANK_angle.z.raw = A_ANK{31,6};
sub07.Normalwalkig.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{31,6},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.ANK_angle.z.std = std(A_ANK{31,6},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.x.raw = A_KNE{31,4};
sub07.Normalwalkig.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{31,4},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.x.std = std(A_KNE{31,4},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.y.raw = A_KNE{31,5};
sub07.Normalwalkig.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{31,5},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.y.std = std(A_KNE{31,5},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.z.raw = A_KNE{31,6};
sub07.Normalwalkig.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{31,6},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.KNE_angle.z.std = std(A_KNE{31,6},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.x.raw = A_HIP{31,4};
sub07.Normalwalkig.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{31,4},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.x.std = std(A_HIP{31,4},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.y.raw = A_HIP{31,5};
sub07.Normalwalkig.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{31,5},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.y.std = std(A_HIP{31,5},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.z.raw = A_HIP{31,6};
sub07.Normalwalkig.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{31,6},2,'omitnan');
sub07.Normalwalkig.kinematics.Left.HIP_angle.z.std = std(A_HIP{31,6},0,2,'omitnan');

sub07.Normalwalkig.kinematics.Right.ANK_angle.x.raw = A_ANK{31,1};
sub07.Normalwalkig.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{31,1},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.ANK_angle.x.std = std(A_ANK{31,1},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.ANK_angle.y.raw = A_ANK{31,2};
sub07.Normalwalkig.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{31,2},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.ANK_angle.y.std = std(A_ANK{31,2},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.ANK_angle.z.raw = A_ANK{31,3};
sub07.Normalwalkig.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{31,3},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.ANK_angle.z.std = std(A_ANK{31,3},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.x.raw = A_KNE{31,1};
sub07.Normalwalkig.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{31,1},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.x.std = std(A_KNE{31,1},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.y.raw = A_KNE{31,2};
sub07.Normalwalkig.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{31,2},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.y.std = std(A_KNE{31,2},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.z.raw = A_KNE{31,3};
sub07.Normalwalkig.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{31,3},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.KNE_angle.z.std = std(A_KNE{31,3},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.x.raw = A_HIP{31,1};
sub07.Normalwalkig.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{31,1},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.x.std = std(A_HIP{31,1},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.y.raw = A_HIP{31,2};
sub07.Normalwalkig.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{31,2},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.y.std = std(A_HIP{31,2},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.z.raw = A_HIP{31,3};
sub07.Normalwalkig.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{31,3},2,'omitnan');
sub07.Normalwalkig.kinematics.Right.HIP_angle.z.std = std(A_HIP{31,3},0,2,'omitnan');

sub07.Normalwalkig.kinematics.Pelvis_tilt.x.raw = A_PEL{31,1};
sub07.Normalwalkig.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{31,1},2,'omitnan');
sub07.Normalwalkig.kinematics.Pelvis_tilt.x.std = std(A_PEL{31,1},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Pelvis_tilt.y.raw = A_PEL{31,2};
sub07.Normalwalkig.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{31,2},2,'omitnan');
sub07.Normalwalkig.kinematics.Pelvis_tilt.y.std = std(A_PEL{31,2},0,2,'omitnan');
sub07.Normalwalkig.kinematics.Pelvis_tilt.z.raw = A_PEL{31,3};
sub07.Normalwalkig.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{31,3},2,'omitnan');
sub07.Normalwalkig.kinematics.Pelvis_tilt.z.std = std(A_PEL{31,3},0,2,'omitnan');



sub07.Profile1.kinematics.Left.ANK_angle.x.raw = A_ANK{32,4};
sub07.Profile1.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{32,4},2,'omitnan');
sub07.Profile1.kinematics.Left.ANK_angle.x.std = std(A_ANK{32,4},0,2,'omitnan');
sub07.Profile1.kinematics.Left.ANK_angle.y.raw = A_ANK{32,5};
sub07.Profile1.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{32,5},2,'omitnan');
sub07.Profile1.kinematics.Left.ANK_angle.y.std = std(A_ANK{32,5},0,2,'omitnan');
sub07.Profile1.kinematics.Left.ANK_angle.z.raw = A_ANK{32,6};
sub07.Profile1.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{32,6},2,'omitnan');
sub07.Profile1.kinematics.Left.ANK_angle.z.std = std(A_ANK{32,6},0,2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.x.raw = A_KNE{32,4};
sub07.Profile1.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{32,4},2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.x.std = std(A_KNE{32,4},0,2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.y.raw = A_KNE{32,5};
sub07.Profile1.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{32,5},2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.y.std = std(A_KNE{32,5},0,2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.z.raw = A_KNE{32,6};
sub07.Profile1.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{32,6},2,'omitnan');
sub07.Profile1.kinematics.Left.KNE_angle.z.std = std(A_KNE{32,6},0,2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.x.raw = A_HIP{32,4};
sub07.Profile1.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{32,4},2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.x.std = std(A_HIP{32,4},0,2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.y.raw = A_HIP{32,5};
sub07.Profile1.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{32,5},2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.y.std = std(A_HIP{32,5},0,2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.z.raw = A_HIP{32,6};
sub07.Profile1.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{32,6},2,'omitnan');
sub07.Profile1.kinematics.Left.HIP_angle.z.std = std(A_HIP{32,6},0,2,'omitnan');

sub07.Profile1.kinematics.Right.ANK_angle.x.raw = A_ANK{32,1};
sub07.Profile1.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{32,1},2,'omitnan');
sub07.Profile1.kinematics.Right.ANK_angle.x.std = std(A_ANK{32,1},0,2,'omitnan');
sub07.Profile1.kinematics.Right.ANK_angle.y.raw = A_ANK{32,2};
sub07.Profile1.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{32,2},2,'omitnan');
sub07.Profile1.kinematics.Right.ANK_angle.y.std = std(A_ANK{32,2},0,2,'omitnan');
sub07.Profile1.kinematics.Right.ANK_angle.z.raw = A_ANK{32,3};
sub07.Profile1.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{32,3},2,'omitnan');
sub07.Profile1.kinematics.Right.ANK_angle.z.std = std(A_ANK{32,3},0,2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.x.raw = A_KNE{32,1};
sub07.Profile1.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{32,1},2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.x.std = std(A_KNE{32,1},0,2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.y.raw = A_KNE{32,2};
sub07.Profile1.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{32,2},2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.y.std = std(A_KNE{32,2},0,2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.z.raw = A_KNE{32,3};
sub07.Profile1.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{32,3},2,'omitnan');
sub07.Profile1.kinematics.Right.KNE_angle.z.std = std(A_KNE{32,3},0,2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.x.raw = A_HIP{32,1};
sub07.Profile1.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{32,1},2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.x.std = std(A_HIP{32,1},0,2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.y.raw = A_HIP{32,2};
sub07.Profile1.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{32,2},2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.y.std = std(A_HIP{32,2},0,2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.z.raw = A_HIP{32,3};
sub07.Profile1.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{32,3},2,'omitnan');
sub07.Profile1.kinematics.Right.HIP_angle.z.std = std(A_HIP{32,3},0,2,'omitnan');

sub07.Profile1.kinematics.Pelvis_tilt.x.raw = A_PEL{32,1};
sub07.Profile1.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{32,1},2,'omitnan');
sub07.Profile1.kinematics.Pelvis_tilt.x.std = std(A_PEL{32,1},0,2,'omitnan');
sub07.Profile1.kinematics.Pelvis_tilt.y.raw = A_PEL{32,2};
sub07.Profile1.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{32,2},2,'omitnan');
sub07.Profile1.kinematics.Pelvis_tilt.y.std = std(A_PEL{32,2},0,2,'omitnan');
sub07.Profile1.kinematics.Pelvis_tilt.z.raw = A_PEL{32,3};
sub07.Profile1.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{32,3},2,'omitnan');
sub07.Profile1.kinematics.Pelvis_tilt.z.std = std(A_PEL{32,3},0,2,'omitnan');



sub07.Profile2.kinematics.Left.ANK_angle.x.raw = A_ANK{33,4};
sub07.Profile2.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{33,4},2,'omitnan');
sub07.Profile2.kinematics.Left.ANK_angle.x.std = std(A_ANK{33,4},0,2,'omitnan');
sub07.Profile2.kinematics.Left.ANK_angle.y.raw = A_ANK{33,5};
sub07.Profile2.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{33,5},2,'omitnan');
sub07.Profile2.kinematics.Left.ANK_angle.y.std = std(A_ANK{33,5},0,2,'omitnan');
sub07.Profile2.kinematics.Left.ANK_angle.z.raw = A_ANK{33,6};
sub07.Profile2.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{33,6},2,'omitnan');
sub07.Profile2.kinematics.Left.ANK_angle.z.std = std(A_ANK{33,6},0,2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.x.raw = A_KNE{33,4};
sub07.Profile2.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{33,4},2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.x.std = std(A_KNE{33,4},0,2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.y.raw = A_KNE{33,5};
sub07.Profile2.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{33,5},2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.y.std = std(A_KNE{33,5},0,2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.z.raw = A_KNE{33,6};
sub07.Profile2.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{33,6},2,'omitnan');
sub07.Profile2.kinematics.Left.KNE_angle.z.std = std(A_KNE{33,6},0,2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.x.raw = A_HIP{33,4};
sub07.Profile2.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{33,4},2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.x.std = std(A_HIP{33,4},0,2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.y.raw = A_HIP{33,5};
sub07.Profile2.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{33,5},2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.y.std = std(A_HIP{33,5},0,2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.z.raw = A_HIP{33,6};
sub07.Profile2.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{33,6},2,'omitnan');
sub07.Profile2.kinematics.Left.HIP_angle.z.std = std(A_HIP{33,6},0,2,'omitnan');

sub07.Profile2.kinematics.Right.ANK_angle.x.raw = A_ANK{33,1};
sub07.Profile2.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{33,1},2,'omitnan');
sub07.Profile2.kinematics.Right.ANK_angle.x.std = std(A_ANK{33,1},0,2,'omitnan');
sub07.Profile2.kinematics.Right.ANK_angle.y.raw = A_ANK{33,2};
sub07.Profile2.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{33,2},2,'omitnan');
sub07.Profile2.kinematics.Right.ANK_angle.y.std = std(A_ANK{33,2},0,2,'omitnan');
sub07.Profile2.kinematics.Right.ANK_angle.z.raw = A_ANK{33,3};
sub07.Profile2.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{33,3},2,'omitnan');
sub07.Profile2.kinematics.Right.ANK_angle.z.std = std(A_ANK{33,3},0,2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.x.raw = A_KNE{33,1};
sub07.Profile2.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{33,1},2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.x.std = std(A_KNE{33,1},0,2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.y.raw = A_KNE{33,2};
sub07.Profile2.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{33,2},2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.y.std = std(A_KNE{33,2},0,2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.z.raw = A_KNE{33,3};
sub07.Profile2.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{33,3},2,'omitnan');
sub07.Profile2.kinematics.Right.KNE_angle.z.std = std(A_KNE{33,3},0,2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.x.raw = A_HIP{33,1};
sub07.Profile2.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{33,1},2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.x.std = std(A_HIP{33,1},0,2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.y.raw = A_HIP{33,2};
sub07.Profile2.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{33,2},2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.y.std = std(A_HIP{33,2},0,2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.z.raw = A_HIP{33,3};
sub07.Profile2.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{33,3},2,'omitnan');
sub07.Profile2.kinematics.Right.HIP_angle.z.std = std(A_HIP{33,3},0,2,'omitnan');

sub07.Profile2.kinematics.Pelvis_tilt.x.raw = A_PEL{33,1};
sub07.Profile2.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{33,1},2,'omitnan');
sub07.Profile2.kinematics.Pelvis_tilt.x.std = std(A_PEL{33,1},0,2,'omitnan');
sub07.Profile2.kinematics.Pelvis_tilt.y.raw = A_PEL{33,2};
sub07.Profile2.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{33,2},2,'omitnan');
sub07.Profile2.kinematics.Pelvis_tilt.y.std = std(A_PEL{33,2},0,2,'omitnan');
sub07.Profile2.kinematics.Pelvis_tilt.z.raw = A_PEL{33,3};
sub07.Profile2.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{33,3},2,'omitnan');
sub07.Profile2.kinematics.Pelvis_tilt.z.std = std(A_PEL{33,3},0,2,'omitnan');



sub07.Profile3.kinematics.Left.ANK_angle.x.raw = A_ANK{34,4};
sub07.Profile3.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{34,4},2,'omitnan');
sub07.Profile3.kinematics.Left.ANK_angle.x.std = std(A_ANK{34,4},0,2,'omitnan');
sub07.Profile3.kinematics.Left.ANK_angle.y.raw = A_ANK{34,5};
sub07.Profile3.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{34,5},2,'omitnan');
sub07.Profile3.kinematics.Left.ANK_angle.y.std = std(A_ANK{34,5},0,2,'omitnan');
sub07.Profile3.kinematics.Left.ANK_angle.z.raw = A_ANK{34,6};
sub07.Profile3.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{34,6},2,'omitnan');
sub07.Profile3.kinematics.Left.ANK_angle.z.std = std(A_ANK{34,6},0,2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.x.raw = A_KNE{34,4};
sub07.Profile3.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{34,4},2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.x.std = std(A_KNE{34,4},0,2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.y.raw = A_KNE{34,5};
sub07.Profile3.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{34,5},2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.y.std = std(A_KNE{34,5},0,2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.z.raw = A_KNE{34,6};
sub07.Profile3.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{34,6},2,'omitnan');
sub07.Profile3.kinematics.Left.KNE_angle.z.std = std(A_KNE{34,6},0,2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.x.raw = A_HIP{34,4};
sub07.Profile3.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{34,4},2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.x.std = std(A_HIP{34,4},0,2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.y.raw = A_HIP{34,5};
sub07.Profile3.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{34,5},2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.y.std = std(A_HIP{34,5},0,2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.z.raw = A_HIP{34,6};
sub07.Profile3.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{34,6},2,'omitnan');
sub07.Profile3.kinematics.Left.HIP_angle.z.std = std(A_HIP{34,6},0,2,'omitnan');

sub07.Profile3.kinematics.Right.ANK_angle.x.raw = A_ANK{34,1};
sub07.Profile3.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{34,1},2,'omitnan');
sub07.Profile3.kinematics.Right.ANK_angle.x.std = std(A_ANK{34,1},0,2,'omitnan');
sub07.Profile3.kinematics.Right.ANK_angle.y.raw = A_ANK{34,2};
sub07.Profile3.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{34,2},2,'omitnan');
sub07.Profile3.kinematics.Right.ANK_angle.y.std = std(A_ANK{34,2},0,2,'omitnan');
sub07.Profile3.kinematics.Right.ANK_angle.z.raw = A_ANK{34,3};
sub07.Profile3.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{34,3},2,'omitnan');
sub07.Profile3.kinematics.Right.ANK_angle.z.std = std(A_ANK{34,3},0,2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.x.raw = A_KNE{34,1};
sub07.Profile3.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{34,1},2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.x.std = std(A_KNE{34,1},0,2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.y.raw = A_KNE{34,2};
sub07.Profile3.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{34,2},2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.y.std = std(A_KNE{34,2},0,2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.z.raw = A_KNE{34,3};
sub07.Profile3.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{34,3},2,'omitnan');
sub07.Profile3.kinematics.Right.KNE_angle.z.std = std(A_KNE{34,3},0,2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.x.raw = A_HIP{34,1};
sub07.Profile3.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{34,1},2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.x.std = std(A_HIP{34,1},0,2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.y.raw = A_HIP{34,2};
sub07.Profile3.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{34,2},2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.y.std = std(A_HIP{34,2},0,2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.z.raw = A_HIP{34,3};
sub07.Profile3.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{34,3},2,'omitnan');
sub07.Profile3.kinematics.Right.HIP_angle.z.std = std(A_HIP{34,3},0,2,'omitnan');

sub07.Profile3.kinematics.Pelvis_tilt.x.raw = A_PEL{34,1};
sub07.Profile3.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{34,1},2,'omitnan');
sub07.Profile3.kinematics.Pelvis_tilt.x.std = std(A_PEL{34,1},0,2,'omitnan');
sub07.Profile3.kinematics.Pelvis_tilt.y.raw = A_PEL{34,2};
sub07.Profile3.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{34,2},2,'omitnan');
sub07.Profile3.kinematics.Pelvis_tilt.y.std = std(A_PEL{34,2},0,2,'omitnan');
sub07.Profile3.kinematics.Pelvis_tilt.z.raw = A_PEL{34,3};
sub07.Profile3.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{34,3},2,'omitnan');
sub07.Profile3.kinematics.Pelvis_tilt.z.std = std(A_PEL{34,3},0,2,'omitnan');



sub07.Profile4.kinematics.Left.ANK_angle.x.raw = A_ANK{35,4};
sub07.Profile4.kinematics.Left.ANK_angle.x.mean = mean(A_ANK{35,4},2,'omitnan');
sub07.Profile4.kinematics.Left.ANK_angle.x.std = std(A_ANK{35,4},0,2,'omitnan');
sub07.Profile4.kinematics.Left.ANK_angle.y.raw = A_ANK{35,5};
sub07.Profile4.kinematics.Left.ANK_angle.y.mean = mean(A_ANK{35,5},2,'omitnan');
sub07.Profile4.kinematics.Left.ANK_angle.y.std = std(A_ANK{35,5},0,2,'omitnan');
sub07.Profile4.kinematics.Left.ANK_angle.z.raw = A_ANK{35,6};
sub07.Profile4.kinematics.Left.ANK_angle.z.mean = mean(A_ANK{35,6},2,'omitnan');
sub07.Profile4.kinematics.Left.ANK_angle.z.std = std(A_ANK{35,6},0,2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.x.raw = A_KNE{35,4};
sub07.Profile4.kinematics.Left.KNE_angle.x.mean = mean(A_KNE{35,4},2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.x.std = std(A_KNE{35,4},0,2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.y.raw = A_KNE{35,5};
sub07.Profile4.kinematics.Left.KNE_angle.y.mean = mean(A_KNE{35,5},2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.y.std = std(A_KNE{35,5},0,2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.z.raw = A_KNE{35,6};
sub07.Profile4.kinematics.Left.KNE_angle.z.mean = mean(A_KNE{35,6},2,'omitnan');
sub07.Profile4.kinematics.Left.KNE_angle.z.std = std(A_KNE{35,6},0,2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.x.raw = A_HIP{35,4};
sub07.Profile4.kinematics.Left.HIP_angle.x.mean = mean(A_HIP{35,4},2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.x.std = std(A_HIP{35,4},0,2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.y.raw = A_HIP{35,5};
sub07.Profile4.kinematics.Left.HIP_angle.y.mean = mean(A_HIP{35,5},2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.y.std = std(A_HIP{35,5},0,2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.z.raw = A_HIP{35,6};
sub07.Profile4.kinematics.Left.HIP_angle.z.mean = mean(A_HIP{35,6},2,'omitnan');
sub07.Profile4.kinematics.Left.HIP_angle.z.std = std(A_HIP{35,6},0,2,'omitnan');

sub07.Profile4.kinematics.Right.ANK_angle.x.raw = A_ANK{35,1};
sub07.Profile4.kinematics.Right.ANK_angle.x.mean = mean(A_ANK{35,1},2,'omitnan');
sub07.Profile4.kinematics.Right.ANK_angle.x.std = std(A_ANK{35,1},0,2,'omitnan');
sub07.Profile4.kinematics.Right.ANK_angle.y.raw = A_ANK{35,2};
sub07.Profile4.kinematics.Right.ANK_angle.y.mean = mean(A_ANK{35,2},2,'omitnan');
sub07.Profile4.kinematics.Right.ANK_angle.y.std = std(A_ANK{35,2},0,2,'omitnan');
sub07.Profile4.kinematics.Right.ANK_angle.z.raw = A_ANK{35,3};
sub07.Profile4.kinematics.Right.ANK_angle.z.mean = mean(A_ANK{35,3},2,'omitnan');
sub07.Profile4.kinematics.Right.ANK_angle.z.std = std(A_ANK{35,3},0,2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.x.raw = A_KNE{35,1};
sub07.Profile4.kinematics.Right.KNE_angle.x.mean = mean(A_KNE{35,1},2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.x.std = std(A_KNE{35,1},0,2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.y.raw = A_KNE{35,2};
sub07.Profile4.kinematics.Right.KNE_angle.y.mean = mean(A_KNE{35,2},2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.y.std = std(A_KNE{35,2},0,2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.z.raw = A_KNE{35,3};
sub07.Profile4.kinematics.Right.KNE_angle.z.mean = mean(A_KNE{35,3},2,'omitnan');
sub07.Profile4.kinematics.Right.KNE_angle.z.std = std(A_KNE{35,3},0,2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.x.raw = A_HIP{35,1};
sub07.Profile4.kinematics.Right.HIP_angle.x.mean = mean(A_HIP{35,1},2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.x.std = std(A_HIP{35,1},0,2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.y.raw = A_HIP{35,2};
sub07.Profile4.kinematics.Right.HIP_angle.y.mean = mean(A_HIP{35,2},2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.y.std = std(A_HIP{35,2},0,2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.z.raw = A_HIP{35,3};
sub07.Profile4.kinematics.Right.HIP_angle.z.mean = mean(A_HIP{35,3},2,'omitnan');
sub07.Profile4.kinematics.Right.HIP_angle.z.std = std(A_HIP{35,3},0,2,'omitnan');

sub07.Profile4.kinematics.Pelvis_tilt.x.raw = A_PEL{35,1};
sub07.Profile4.kinematics.Pelvis_tilt.x.mean = mean(A_PEL{35,1},2,'omitnan');
sub07.Profile4.kinematics.Pelvis_tilt.x.std = std(A_PEL{35,1},0,2,'omitnan');
sub07.Profile4.kinematics.Pelvis_tilt.y.raw = A_PEL{35,2};
sub07.Profile4.kinematics.Pelvis_tilt.y.mean = mean(A_PEL{35,2},2,'omitnan');
sub07.Profile4.kinematics.Pelvis_tilt.y.std = std(A_PEL{35,2},0,2,'omitnan');
sub07.Profile4.kinematics.Pelvis_tilt.z.raw = A_PEL{35,3};
sub07.Profile4.kinematics.Pelvis_tilt.z.mean = mean(A_PEL{35,3},2,'omitnan');
sub07.Profile4.kinematics.Pelvis_tilt.z.std = std(A_PEL{35,3},0,2,'omitnan');


%% Xsensor 

for j=36:50
% j=5;
close all
clear -regexp ^yy_ ^PSD_ ^temp ^L_ ^R_ ^COM_ ^Pelvis_ ^MoS_ ^XCOM_ ^ind ^Q_sync
clear LHS LTO RHS RTO time_mo time_mos i k data7 aa aaa g leg pp qq nn step_mf_x xspa xx leng ans data2 COP_related contact_pressure data4...
    kk num1 hh e formal RHS_X time_xs T_cop_sss

data4 = raw4{j};
time_xs = data4(:,1);
L_heel = data4(:,33);
R_heel = data4(:,42);

num1 = 1;
hh = 0;
e = 5;
formal = 0;
for i = 2:length(R_heel)
    if hh == 0 && R_heel(i-1,1) < e && R_heel(i,1) >= e && i > formal+60
        RHS_X(num1,1) = i;
        formal = i;
        hh = 1 ;
    elseif hh == 1 && R_heel(i-1,1) > e && R_heel(i,1) <= e 
        hh = 0 ;
        num1 = num1+1;
    end
end
% 
% 
% figure(886)
% hold on
% plot(time_xs,R_heel)
% plot(time_xs(RHS_X),R_heel(RHS_X),'*r')

% size = [0.265 0.265 0.255 0.255 0.255 0.275 0.265];
L_cop = data4(:,17:18);
R_cop = data4(:,30:31);
L_pressure = data4(:,9);
R_pressure = data4(:,22);
% L_cop_ps = [-L_cop(:,1)+31,L_cop(:,2)]; %% 이상하게 나오면 이걸로
L_cop_ps = [-L_cop(:,1)+31,-L_cop(:,2)+11];
R_cop_ps = [-R_cop(:,1)+31,R_cop(:,2)+11];

L_pressure_map = L_pressure.*data4(:,7).*data4(:,8).*data4(:,14);
R_pressure_map = R_pressure.*data4(:,20).*data4(:,21).*data4(:,27);
contact_pressure = [L_pressure_map R_pressure_map];
COP_related= abs(L_cop_ps-R_cop_ps); %%% Cop의 상대적 위치 (:,2)
temp1=[COP_related, contact_pressure(:,1)./(contact_pressure(:,1)+contact_pressure(:,2)), ...
    contact_pressure(:,2)./(contact_pressure(:,1)+contact_pressure(:,2))]; %%% [cop상대적위치 , L비율 , R 비율] (:,4)
% temp2=[temp1 temp1(:,1).*temp1(:,3) temp1(:,1).*temp1(:,4) temp1(:,2).*temp1(:,3) temp1(:,2).*temp1(:,4)];

for kk=1:length(time_xs)
T_cop_sss(kk,1) = dot ([contact_pressure(kk,1)./(contact_pressure(kk,1)+contact_pressure(kk,2)), ...
    contact_pressure(kk,2)./(contact_pressure(kk,1)+contact_pressure(kk,2))],[L_cop_ps(kk,1),R_cop_ps(kk,1)]);
T_cop_sss(kk,2) = dot ([contact_pressure(kk,1)./(contact_pressure(kk,1)+contact_pressure(kk,2)), ...
    contact_pressure(kk,2)./(contact_pressure(kk,1)+contact_pressure(kk,2))],[L_cop_ps(kk,2),R_cop_ps(kk,2)]);
end

for k = 1:2
nn =  isnan(T_cop_sss(:,k));
for i=2:length(T_cop_sss(:,k))-1
    if nn(i,1) == 1 
       T_cop_sss(i,k) = 0;
    end
end
end

T_cop{j,1} = T_cop_sss(:,1);
T_cop{j,2} = T_cop_sss(:,2);

for pp= 1:120 
    T_cop_var(pp,1) = std(T_cop_sss(RHS_X(end-pp,1):RHS_X(end-pp+1,1),1));
    T_cop_var(pp,2) = std(T_cop_sss(RHS_X(end-pp,1):RHS_X(end-pp+1,1),2));
%     MoS_y_R_min(pp,1) = min(MoS_y_R(RHS(end-pp,1):RHS(end-pp+1,1)));
end
T_cop{j,3} = T_cop_var;

end
T_cop_v = [mean(T_cop{1,3}(:,2)),mean(T_cop{1,3}(:,2)),mean(T_cop{1,3}(:,2)),mean(T_cop{1,3}(:,2)),mean(T_cop{1,3}(:,2));...
    mean(T_cop{6,3}(:,2)),mean(T_cop{7,3}(:,2)),mean(T_cop{8,3}(:,2)),mean(T_cop{9,3}(:,2)),mean(T_cop{10,3}(:,2));...
    mean(T_cop{11,3}(:,2)),mean(T_cop{12,3}(:,2)),mean(T_cop{13,3}(:,2)),mean(T_cop{14,3}(:,2)),mean(T_cop{15,3}(:,2));...
    mean(T_cop{16,3}(:,2)),mean(T_cop{17,3}(:,2)),mean(T_cop{18,3}(:,2)),mean(T_cop{19,3}(:,2)),mean(T_cop{20,3}(:,2));...
    mean(T_cop{21,3}(:,2)),mean(T_cop{22,3}(:,2)),mean(T_cop{23,3}(:,2)),mean(T_cop{24,3}(:,2)),mean(T_cop{25,3}(:,2));...
    mean(T_cop{26,3}(:,2)),mean(T_cop{27,3}(:,2)),mean(T_cop{28,3}(:,2)),mean(T_cop{29,3}(:,2)),mean(T_cop{30,3}(:,2));...
    mean(T_cop{31,3}(:,2)),mean(T_cop{32,3}(:,2)),mean(T_cop{33,3}(:,2)),mean(T_cop{34,3}(:,2)),mean(T_cop{35,3}(:,2));...
    mean(T_cop{36,3}(:,2)),mean(T_cop{37,3}(:,2)),mean(T_cop{38,3}(:,2)),mean(T_cop{39,3}(:,2)),mean(T_cop{40,3}(:,2));...
    mean(T_cop{41,3}(:,2)),mean(T_cop{42,3}(:,2)),mean(T_cop{43,3}(:,2)),mean(T_cop{44,3}(:,2)),mean(T_cop{45,3}(:,2));...
    mean(T_cop{46,3}(:,2)),mean(T_cop{47,3}(:,2)),mean(T_cop{48,3}(:,2)),mean(T_cop{49,3}(:,2)),mean(T_cop{50,3}(:,2));...
    ];

sub07.Profile4.balance_related.COP.ML.raw = T_cop{1,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{1,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{1,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{1,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{1,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{2,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{2,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{2,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{2,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{2,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{3,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{3,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{3,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{3,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{3,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{4,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{4,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{4,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{4,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{4,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{5,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{5,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{5,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{5,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{5,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{6,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{6,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{6,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{6,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{6,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{7,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{7,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{7,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{7,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{7,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{8,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{8,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{8,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{8,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{8,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{9,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{9,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{9,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{9,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{9,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{10,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{10,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{10,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{10,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{10,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{11,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{11,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{11,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{11,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{11,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{12,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{12,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{12,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{12,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{12,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{13,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{13,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{13,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{13,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{13,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{14,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{14,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{14,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{14,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{14,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{15,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{15,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{15,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{15,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{15,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{16,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{16,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{16,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{16,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{16,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{17,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{17,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{17,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{17,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{17,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{18,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{18,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{18,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{18,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{18,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{19,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{19,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{19,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{19,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{19,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{20,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{20,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{20,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{20,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{20,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{21,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{21,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{21,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{21,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{21,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{22,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{22,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{22,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{22,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{22,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{23,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{23,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{23,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{23,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{23,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{24,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{24,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{24,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{24,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{24,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{25,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{25,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{25,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{25,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{25,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{26,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{26,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{26,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{26,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{26,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{27,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{27,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{27,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{27,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{27,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{28,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{28,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{28,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{28,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{28,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{29,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{29,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{29,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{29,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{29,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{30,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{30,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{30,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{30,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{30,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{31,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{31,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{31,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{31,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{31,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{32,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{32,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{32,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{32,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{32,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{33,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{33,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{33,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{33,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{33,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{34,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{34,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{34,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{34,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{34,3}(:,2));
sub07.Profile4.balance_related.COP.ML.raw = T_cop{35,2};
sub07.Profile4.balance_related.COP.AP.raw = T_cop{35,1};
sub07.Profile4.balance_related.COP.ML.variability.raw = T_cop{35,3}(:,2);
sub07.Profile4.balance_related.COP.ML.variability.mean = mean(T_cop{35,3}(:,2));
sub07.Profile4.balance_related.COP.ML.variability.std = std(T_cop{35,3}(:,2));


% sub07.Profile4.balance_related.COP.std = std(MoS_x_R_min);







xxx = [sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean;...
    sub07.Profile4.balance_related.COP.ML.variability.mean , sub07.Profile4.balance_related.COP.ML.variability.mean ,...
    sub07.Profile4.balance_related.COP.ML.variability.mean,sub07.Profile4.balance_related.COP.ML.variability.mean,...
    sub07.Profile4.balance_related.COP.ML.variability.mean];

% 
% figure(3)
% set(gcf,'Position',[0 0 440 620],'Renderer', 'painters')
% plot(T_cop_sss(:,2),T_cop_sss(:,1),'k');%hold on
% ylim([0 31])
% xlim([0 22])
% % title('COP sub8 verification','Fontsize',30)
% title('COP sub8 normal2','Fontsize',30)
% set(gcf, 'Color', 'w');
% 
% figure(44)
% set(gcf,'Position',[0 0 440 620],'Renderer', 'painters')
% ylim([0 31])
% xlim([0 22])
% hold on
% scatter(L_cop_ps(:,2),L_cop_ps(:,1),25,[38/255,70/255,83/255],'filled')
% scatter(R_cop_ps(:,2),R_cop_ps(:,1),25,[231/255,111/255,81/255],'filled')
% % title('COP sub8 verification','Fontsize',20)
% title('COP sub8 normal2','Fontsize',20)
% set(gcf, 'Color', 'w');
% legend('Left cop','Right cop') 


%% time sync cheak


ss2 = index2-index1;   %% control
ss1 = ind22-ind11;       %% motion
% ss1 = ind2-ind1;       %% motion
ss(1,1) = ss2;
ss(1,2) = ss1;
ss(1,3) = ss2-ss1;

% ss3 = (Time_integrated(end,1)-Time_integrated(1,1));

Q_stack = [mean(Q_PSD),mean(Q_MOTOR),mean(Q_GCP)];
figure(7888)
yyaxis left
hold on
plot(time,Q_PSD,'-','color',[0 0.4470 0.7410])
plot(time,Q_MOTOR,'-','color',[0.8500 0.3250 0.0980])
plot(time,Q_GCP,'-','color',[0.9290 0.6940 0.1250])
yyaxis right
plot(time,Time_PSD_loop,'-','color',[0 0.4470 0.7410])
plot(time,Time_MOTOR_loop,'-','color',[0.8500 0.3250 0.0980])
plot(time,Time_GCP_loop,'-','color',[0.9290 0.6940 0.1250])
plot(time,Time_SAVE_loop,'k-')
legend('PSD','MOTOR','GCP','PDS time','MOTOR time','GCP time','SAVE time')

figure(429034234)
hold on
plot(time,R_angvel_raw)
plot(time,R_angvel_filt)
plot(time,R_angvel_filt_62)


%% HS from motion
%%%%%%%%% sync 맞는지 체크하는 plot %%%%%%%%%%%%%%%%%%%
figure(2234)
yyaxis left
hold on
plot(Q_sync_ds(ind11:ind22,1),'-','color',[0 0.4470 0.7410])
plot(Q_sync,'-','color',[0.8500 0.3250 0.0980])

yyaxis right
hold on
plot(R_mf_lp*100,'b-')
plot(R_angvel_filt_62,'r-')



%% Metabolic 
results = [];
flag = [];
for j=1:7
data2 = raw2{j};
data3 = raw3{j};
kg = [75;73;70;67;65;86;94;66]; 
Vo2 = data2(4:end,15);
Vco2 = data2(4:end,16);
met_power = 16.58*(Vo2/60)+4.51*(Vco2/60);
met_power_kg = met_power/kg(j,1); 
time_const = 1.15741E-05;
time = data2(4:end,10)/(time_const);
phase = table2array(data3(3:end,36));
num1 = 1;
for i = 1:length(time)-1
    if phase{i}(1) == 'E' && phase{i+1}(1) == 'R'
       flag(num1,1) = i;
       [~,flag(num1,2)] = min(abs(time-(time(i)-120)));
       num1 = num1+1;
    end
end

ST = mean(met_power_kg(flag(1,2):flag(1,1)));
N1 = mean(met_power_kg(flag(2,2):flag(2,1)))-ST;
T1 = mean(met_power_kg(flag(3,2):flag(3,1)))-ST;
T2 = mean(met_power_kg(flag(4,2):flag(4,1)))-ST;
T3 = mean(met_power_kg(flag(5,2):flag(5,1)))-ST;
T4 = mean(met_power_kg(flag(6,2):flag(6,1)))-ST;
result = [ST N1 T1 T2 T3 T4];
results = [results ;result];

end

%%


B_mos = [0.157	0.116	0.101	0.119	0.103
0.151	0.116	0.123	0.112	0.079
0.140	0.119	0.127	0.127	0.115
0.158	0.137	0.118	0.140	0.107
0.150	0.119	0.144	0.129	0.105
0.160	0.138	0.148	0.136	0.112
0.190	0.171	0.175	0.147	0.128
0.170	0.160	0.147	0.151	0.140
0.187	0.176	0.168	0.197	0.178
0.202	0.202	0.197	0.209	0.183
];
B_sw =[0.131	0.141	0.150	0.144	0.137
0.136	0.149	0.153	0.145	0.154
0.132	0.164	0.141	0.141	0.147
0.128	0.134	0.132	0.130	0.130
0.135	0.143	0.146	0.138	0.127
0.131	0.124	0.125	0.137	0.132
0.167	0.174	0.160	0.149	0.155
0.128	0.144	0.140	0.106	0.130
0.149	0.159	0.154	0.157	0.152
0.164	0.168	0.165	0.168	0.163
];
B_swv =[0.010	0.015	0.018	0.015	0.018
0.009	0.010	0.013	0.012	0.013
0.008	0.009	0.010	0.012	0.009
0.016	0.018	0.020	0.018	0.015
0.009	0.010	0.012	0.012	0.013
0.011	0.012	0.014	0.018	0.015
0.016	0.018	0.019	0.019	0.017
0.008	0.009	0.014	0.016	0.013
0.010	0.009	0.012	0.010	0.010
0.009	0.010	0.013	0.012	0.011
];

B_cov =[7.793	10.343	12.136	10.364	13.365
6.795	6.660	8.184	8.528	8.508
5.751	5.612	7.147	8.205	6.246
12.415	13.362	14.796	13.619	11.762
6.670	7.086	8.156	9.034	9.998
8.000	10.012	11.362	13.382	11.688
9.424	10.542	11.763	12.413	10.700
5.996	6.360	10.228	15.006	9.930
6.508	5.761	7.525	6.238	6.832
5.499	6.171	7.749	7.243	6.867
];

B_cop = [6.718	6.426	6.484	6.654	6.508
6.805	6.799	6.791	6.970	6.799
5.505	5.601	5.714	5.712	5.679
6.554	6.164	6.282	6.509	6.483
5.962	5.728	5.547	5.617	4.559
6.912	6.706	6.706	6.980	6.691
6.556	6.616	6.625	6.791	6.986
];

B_sl = [0.688	0.737	0.712	0.686	0.701
0.614	0.616	0.621	0.617	0.624
0.630	0.630	0.628	0.609	0.627
0.693	0.725	0.707	0.687	0.698
0.643	0.662	0.629	0.615	0.592
0.677	0.667	0.664	0.640	0.643
0.669	0.659	0.644	0.625	0.604
0.671	0.685	0.671	0.660	0.657
0.653	0.670	0.663	0.657	0.651
0.688	0.688	0.691	0.700	0.721
];

B_ca =[0.955	0.898	0.926	0.947	0.928
1.015	1.008	1.002	1.011	1.001
1.002	1.005	1.012	1.045	1.013
0.950	0.899	0.925	0.949	0.926
1.007	0.982	1.023	1.051	1.095
0.938	0.941	0.935	0.982	0.972
0.964	0.963	0.991	1.006	1.040
0.964	0.930	0.948	0.967	0.963
0.987	0.974	0.974	1.001	0.998
0.921	0.918	0.908	0.905	0.880
];

B_sts =[0.625	0.660	0.640	0.626	0.640
0.583	0.591	0.597	0.581	0.591
0.593	0.588	0.589	0.557	0.575
0.624	0.661	0.645	0.623	0.638
0.579	0.595	0.571	0.551	0.532
0.640	0.644	0.645	0.607	0.613
0.629	0.630	0.610	0.595	0.573
0.619	0.640	0.629	0.609	0.609
0.608	0.623	0.622	0.599	0.601
0.655	0.665	0.673	0.662	0.691
];

B_dst =[10.027	10.073	10.060	9.554	9.570
9.660	9.935	10.208	9.498	9.998
9.408	9.695	10.464	8.852	9.083
9.427	9.216	9.190	8.998	8.893
8.396	8.882	8.821	8.068	7.792
10.520	11.035	10.702	10.571	10.434
11.198	10.971	11.078	9.753	9.592
10.107	10.010	9.849	9.249	8.806
9.468	10.037	9.825	9.138	8.890
10.623	11.082	11.187	10.275	10.839
];

mean(B_mos)


figure(3)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_mos),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,0.2])
yticks([0 0.05 0.1 0.15 0.2])
xlabel('MoS ML')
hold on
er = errorbar(sizes,mean(B_mos),-std(B_mos),std(B_mos));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;


figure(4)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_sw),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,0.2])
yticks([0 0.05 0.1 0.15 0.2])
xlabel('Average Stepwidth')
hold on
er = errorbar(sizes,mean(B_sw),-std(B_sw),std(B_sw));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(5)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_swv),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,0.02])
yticks([0 0.005 0.01 0.015 0.02])
xlabel('Stepwidth Variability')
hold on
er = errorbar(sizes,mean(B_swv),-std(B_swv),std(B_swv));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(6)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_cop),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,8])
yticks([0 2 4 6 8])
xlabel('CoP Variability')
hold on
er = errorbar(sizes,mean(B_cop),-std(B_cop),std(B_cop));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(7)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_sl),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0.55,0.75])
yticks([0.55 0.6 0.65 0.7 0.75])
xlabel('Stride length')
hold on
er = errorbar(sizes,mean(B_sl),-std(B_sl),std(B_sl));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(8)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_ca),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0.8,1.1])
yticks([0.8 0.9 1 1.1])
xlabel('Cadence')
hold on
er = errorbar(sizes,mean(B_ca),-std(B_ca),std(B_ca));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;


figure(9)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_cov),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([5,15])
% yticks([0 0.05 0.1 0.15 0.2])
xlabel('Step width variability (C.V)')
hold on
er = errorbar(sizes,mean(B_cov),-std(B_cov),std(B_cov));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

B_ds_init = [9.515	9.469	9.394	9.579	9.633
8.898	9.095	9.373	8.053	8.470
9.108	8.774	9.361	8.166	8.079
8.890	9.208	9.518	8.148	8.482
7.507	7.974	7.506	6.808	7.362
9.295	10.315	10.026	8.445	8.469
9.493	9.919	9.628	9.050	8.831
];
B_ds_term=[10.024	10.044	10.052	9.544	9.559
9.655	9.917	10.177	9.491	9.980
9.431	9.669	10.459	8.829	9.049
9.405	9.203	9.179	8.973	8.879
8.394	8.850	8.802	8.053	7.792
10.514	11.033	10.696	10.536	10.439
11.223	10.968	11.036	9.732	9.581
];

B_met = [4.128	4.697	4.697	3.877	4.108
3.966	4.681	4.520	4.277	4.556
4.237	4.719	3.006	3.753	4.121
3.841	4.580	3.992	3.513	3.171
4.452	3.837	3.539	3.026	3.637
4.793	4.763	4.363	3.927	3.946
3.292	3.403	3.296	2.824	2.818
3.760	4.191	3.531	3.178	3.911
4.382	4.537	4.634	4.064	3.875
4.016	4.141	4.331	3.684	3.892
];


figure(171)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
% ax.OuterPosition =[0,0,8.89,4];
% ax.Position =[1,1,8,3.5];

% ax.OuterPosition =[0,0,12.7,6];
ax.Position =[1,1,11,5];

% xticks([0 10 20 30 40 50 60 70 80 90 100])
ylim([2 5])
yticks([2 3 4 5])
hold on
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_met),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off')
% ylim([0.8,1.1])
% yticks([0.8 0.9 1 1.1])
% xlabel('V_pr')
hold on
er = errorbar(sizes,mean(B_met),-std(B_met),std(B_met));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(10)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_ds_init),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([5,12])
% yticks([0 0.05 0.1 0.15 0.2])
xlabel('Double stance time (initial)')
hold on
er = errorbar(sizes,mean(B_ds_init),-std(B_ds_init),std(B_ds_init));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(11)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_ds_term),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255]; 
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([5,12])
% yticks([0 0.05 0.1 0.15 0.2])
xlabel('Double stance time (terminal)')
hold on
er = errorbar(sizes,mean(B_ds_term),-std(B_ds_term),std(B_ds_term));    
er.Color = [0 0 0]; er.LineStyle = 'none'; er.LineWidth = 0.7;


% 
% % a = get(gca,'XTickLabel');
% % set(gca,'XTickLabel',a,'FontName','Arial','fontsize',10)
% 
% % a = get(gca,'YTickLabel');
% % set(gca,'YTickLabel',a,'FontName','Arial','fontsize',10)
% 
% REG_CA = [];
% REG_MOS = [];
% REG_Power_mean = [];
% REG_MET = [];
% 
% REG = [REG_MOS , REG_Power_mean ,REG_MET];
% 
% X =[ones(size(REG_MOS)), REG_MOS, REG_Power_mean ,REG_MOS.*REG_Power_mean];
%     
% [b,bint,r,rint,stats] = regress(REG_MET,X);
% 
% scatter3(REG_MOS,REG_Power_mean,REG_MET,'filled')
% hold on
% 
% x1fit = min(REG_MOS):(max(REG_MOS)-min(REG_MOS))/100:max(REG_MOS);
% x2fit = min(REG_Power_mean):(max(REG_Power_mean)-min(REG_Power_mean))/100:max(REG_Power_mean);
% [X1FIT,X2FIT] = meshgrid(x1fit,x2fit);
% YFIT = b(1) + b(2)*X1FIT + b(3)*X2FIT + b(4)*X1FIT.*X2FIT;
% mesh(X1FIT,X2FIT,YFIT)
% xlabel('Change in MoS ML')
% ylabel('Average positive power')
% zlabel('Change in Metabolic rate')
% view(50,10)
% hold off
% 
% 
% X = [REG_MOS,REG_Power_mean];
% mdl = fitlm(X,REG_MET);

figure(17)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(W_total),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Myraid pro','FontSize',10)
% ylim([0.8,1.1])
% yticks([0.8 0.9 1 1.1])
% xlabel('V_pr')
hold on
er = errorbar(sizes,mean(W_total),-std(W_total),std(W_total));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;


figure(18)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(W_total2),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
% ylim([0.8,1.1])
% yticks([0.8 0.9 1 1.1])
% xlabel('V_pr')
hold on
er = errorbar(sizes,mean(W_total2),-std(W_total2),std(W_total2));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

% B_stance = [0.625	0.660	0.640	0.626	0.640
% 0.583	0.591	0.597	0.581	0.591
% 0.593	0.588	0.589	0.557	0.575
% 0.624	0.661	0.645	0.623	0.638
% 0.579	0.595	0.571	0.551	0.532
% 0.640	0.644	0.645	0.607	0.613
% 0.629	0.630	0.610	0.595	0.573
% ];
    

figure(18)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_sts),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0.5,0.7])
yticks([0.4 0.5 0.6 0.7])
% xlabel('Stance time')
hold on
er = errorbar(sizes,mean(B_sts),-std(B_sts),std(B_sts));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

figure(181)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_dst),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([6,12])
yticks([6 8 10 12])
% xlabel('Stance time')
hold on
er = errorbar(sizes,mean(B_dst),-std(B_dst),std(B_dst));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

B_swing = [0.422	0.454	0.441	0.430	0.437
0.403	0.401	0.401	0.408	0.408
0.410	0.407	0.400	0.401	0.413
0.429	0.451	0.436	0.432	0.441
0.414	0.423	0.407	0.401	0.383
0.425	0.419	0.425	0.412	0.416
0.409	0.409	0.399	0.400	0.389
];
    

figure(19)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_swing),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,0.5])
yticks([0 0.5])
% xlabel('Swing time')
hold on
er = errorbar(sizes,mean(B_swing),-std(B_swing),std(B_swing));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

B_mos = [0.1568	0.1159	0.1010	0.1191	0.1035
0.1507	0.1160	0.1227	0.1123	0.0789
0.1397	0.1191	0.1272	0.1275	0.1146
0.1584	0.1371	0.1180	0.1405	0.1065
0.1502	0.1190	0.1442	0.1287	0.1049
0.1601	0.1380	0.1480	0.1360	0.1125
0.1899	0.1707	0.1751	0.1473	0.1280
];

figure(20)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
valueset = {'Normal walking','MFMP','PFMP','MSMP','PSMP'};
x1 = {'Normal walking','MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(B_mos),0.8,'FaceColor','flat') ;
ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(2,:) = [194/255 81/255 79/255] ; 
ba.CData(3,:) = [248/255 153/255 55/255] ; 
ba.CData(4,:) = [153/255 204/255 103/255] ;
ba.CData(5,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Arial','FontSize',15)
ylim([0,0.2])
yticks([0 0.05 0.10 0.15 0.20])
xlabel('MoS ML')
hold on
er = errorbar(sizes,mean(B_mos),-std(B_mos),std(B_mos));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;


figure(20112)
set(gcf,'Renderer', 'painters')
colororder({'k','k'})
set(gcf, 'Color', 'w');
hold on
ax = gca ;
ax.FontSize = 10 ;
ax.FontName = 'Myraid pro';
ax.Units  ='centimeters';
ax.OuterPosition =[0,0,7.5,6];
% ax.Position =[1,1,8,3.5];
% ax.OuterPosition =[0,0,12.7,9];
% ax.Position =[1.2,12.5,5];
valueset = {'MFMP','PFMP','MSMP','PSMP'};
x1 = {'MFMP','PFMP','MSMP','PSMP'}; %001
sizes = categorical(x1,valueset,'Ordinal',true);
ba = bar(mean(max_force),0.8,'FaceColor','flat') ;
% ba.CData(1,:) = [100/255 100/255 102/255];
ba.CData(1,:) = [194/255 81/255 79/255] ; 
ba.CData(2,:) = [248/255 153/255 55/255] ; 
ba.CData(3,:) = [153/255 204/255 103/255] ;
ba.CData(4,:) = [67/255 172/255 198/255] ;
set(gca,'XTick',[],'Box','off','FontName','Myraid pro','FontSize',10)
ylim([200 300])
yticks([200 225 250 275 300])
% xlabel('MoS ML')
hold on
er = errorbar(sizes,mean(max_force),-std(max_force),std(max_force));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;

B_power = [0.047	0.060	0.081	0.102
0.007	0.062	0.071	0.096
0.019	0.057	0.041	0.085
0.047	0.067	0.065	0.124
0.064	0.085	0.069	0.183
0.025	0.068	0.033	0.074
0.006	0.041	0.063	0.091
0.067	0.103	0.086	0.145
0.000	0.030	0.054	0.096
0.008	0.061	0.046	0.093
];

B_red = [-13.792	-13.781	6.068	0.488
-18.041	-13.959	-7.842	-14.882
-11.354	29.051	11.441	2.758
-19.240	-3.921	8.548	17.438
13.801	20.501	32.027	18.295
0.628	8.970	18.073	17.679
-3.363	-0.101	14.229	14.409
-11.447	6.103	15.492	-4.011
-3.542	-5.755	7.242	11.558
-3.102	-7.837	8.286	3.094
];


figure(211)
set(gcf,'Position',[0 0 300 200],'Renderer', 'painters')
hold on
colororder({'k','k'})
set(gcf, 'Color', 'w');
% hold on
% ylim([0.4 0.5])
% yticks([0.4 0.45 0.5])
% xticks([0 10 20 30 40 50 60 70 80 90 100])
ax = gca ;
ax.FontSize = 20 ;
% plot(B_power(:,1),B_red(:,1),'o','MarkerSize',2,'MarkerFaceColor',[213/255 137/255 135/255])
% plot(B_power(:,2),B_red(:,2),'o','MarkerSize',2,'MarkerFaceColor',[251/255 197/255 143/255],'MarkerEdgeColor','k')
% plot(B_power(:,3),B_red(:,3),'o','MarkerSize',2,'MarkerFaceColor',[188/255 221/255 155/255],'MarkerEdgeColor','k')
% plot(B_power(:,4),B_red(:,4),'o','MarkerSize',2,'MarkerFaceColor',[155/255 212/255 225/255],'MarkerEdgeColor','k')
plot(mean(B_power(:,1)),mean(B_red(:,1)),'o','MarkerSize',20,'MarkerFaceColor',[194/255 81/255 79/255],'MarkerEdgeColor','k')
er = errorbar(mean(B_red(:,1)),-std(B_red(:,1)),std(B_red(:,1)));    
er.Color = [0 0 0];                            
er.LineStyle = 'none'; 
er.LineWidth = 0.7;
plot(mean(B_power(:,2)),mean(B_red(:,2)),'o','MarkerSize',20,'MarkerFaceColor',[248/255 153/255 55/255],'MarkerEdgeColor','k')
plot(mean(B_power(:,3)),mean(B_red(:,3)),'o','MarkerSize',20,'MarkerFaceColor',[153/255 204/255 103/255],'MarkerEdgeColor','k')
plot(mean(B_power(:,4)),mean(B_red(:,4)),'o','MarkerSize',20,'MarkerFaceColor',[67/255 172/255 198/255],'MarkerEdgeColor','k')
plot(B_power(:,1),B_red(:,1),'o','MarkerSize',10,'MarkerFaceColor',[194/255 81/255 79/255])
plot(B_power(:,2),B_red(:,2),'o','MarkerSize',10,'MarkerFaceColor',[248/255 153/255 55/255])
plot(B_power(:,3),B_red(:,3),'o','MarkerSize',10,'MarkerFaceColor',[153/255 204/255 103/255])
plot(B_power(:,4),B_red(:,4),'o','MarkerSize',10,'MarkerFaceColor',[67/255 172/255 198/255])


figure(26)
set(gcf,'Renderer', 'painters')
% title('Delivered power','Fontsize',30)
set(gcf, 'Color', 'w');
hold on
grid on
% ylim([0 350])
ax = gca ;
ax.FontSize = 20 ;
% xlim([0 0.5])
% ylim([-10 20])
hold on
errorbar(mean(B_power(:,1)),mean(B_red(:,1)),-9.422,9.422,-0.024,0.024,'.','Markersize',60,'color',[194/255 81/255 79/255])
errorbar(mean(B_power(:,2)),mean(B_red(:,2)),-13.563,13.563,-0.019,0.019,'.','Markersize',60,'color',[248/255 153/255 55/255])
errorbar(mean(B_power(:,3)),mean(B_red(:,3)),-9.636,9.636,-0.016,0.016,'.','Markersize',60,'color',[153/255 204/255 103/255])
errorbar(mean(B_power(:,4)),mean(B_red(:,4)),-10.493,10.493,-0.031,0.031,'.','Markersize',60,'color',[67/255 172/255 198/255])
yticks([-20 -10 0 10 20])
ylim([-20 25])
xticks([0 0.1 0.2 0.3 0.4 0.5])
xlim([0 0.15])
a = get(gca,'XTickLabel');
set(gca,'XTickLabel',a,'FontName','Arial','fontsize',28)