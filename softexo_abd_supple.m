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
directory3 = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\motion_C3D_2\');
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


directory6 = strcat('Exp_data\SOFT_EXOS\Abduction\HIP_ABD_main\motion_static\');
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


% j=30;

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
% % 
% save HIP_ABD_supp_load '-v7.3'
% save HIP_ABD
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

L_Heel_x = data7(:,5);
L_Heel_y = data7(:,6);
L_Heel_z = data7(:,7);
L_Met1_x = data7(:,17);
L_Met1_y = data7(:,18);
L_Met1_z = data7(:,19);
L_Met5_x = data7(:,20);
L_Met5_y = data7(:,21);
L_Met5_z = data7(:,22);
L_ANK_angle_x = data7(:,92)-mean(data8(:,92),'omitnan');
L_ANK_angle_y = data7(:,93)-mean(data8(:,93),'omitnan');
L_ANK_angle_z = data7(:,94)-mean(data8(:,94),'omitnan');
L_KNE_angle_x = data7(:,98)-mean(data8(:,98),'omitnan');
L_KNE_angle_y = data7(:,99)-mean(data8(:,99),'omitnan');
L_KNE_angle_z = data7(:,100)-mean(data8(:,100),'omitnan');
L_HIP_angle_x = data7(:,95)-mean(data8(:,95),'omitnan');
L_HIP_angle_y = data7(:,96)-mean(data8(:,96),'omitnan');
L_HIP_angle_z = data7(:,97)-mean(data8(:,97),'omitnan');
L_mf_x = data7(:,77);
L_mf_y = data7(:,78);
L_mf_z = data7(:,79);
L_mf_v = data7(:,124);


R_Heel_x = data7(:,41);
R_Heel_y = data7(:,42);
R_Heel_z = data7(:,43);
R_Met1_x = data7(:,53);
R_Met1_y = data7(:,54);
R_Met1_z = data7(:,55);
R_Met5_x = data7(:,56);
% R_Met5_x = data7(:,17);-mean(data8(:,17),'omitnan');
R_Met5_y = data7(:,57);
R_Met5_z = data7(:,58);
R_ANK_angle_x = data7(:,104)-mean(data8(:,104),'omitnan');
R_ANK_angle_y = data7(:,105)-mean(data8(:,105),'omitnan');
R_ANK_angle_z = data7(:,106)-mean(data8(:,106),'omitnan');
R_KNE_angle_s = data7(:,110)-mean(data8(:,110),'omitnan');
R_KNE_angle_f = data7(:,111)-mean(data8(:,111),'omitnan');
R_KNE_angle_z = data7(:,112)-mean(data8(:,112),'omitnan');
R_HIP_angle_s = data7(:,107)-mean(data8(:,107),'omitnan'); %% x : SAGITTAL
R_HIP_angle_f = data7(:,108)-mean(data8(:,108),'omitnan'); %% y : Forntal
R_HIP_angle_z = data7(:,109)-mean(data8(:,109),'omitnan'); %% z : transverse
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

R_mf_x = data7(:,83);
R_mf_y = data7(:,84);
R_mf_z = data7(:,85);

R_mf_v = data7(:,121);

COM_x  = data7(:,113);
COM_y  = data7(:,114);
COM_z = data7(:,115);
COM_vel_x = data7(:,116);
COM_vel_y = data7(:,117);
COM_vel_z = data7(:,118);
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

Pelvis_tilt_x = data7(:,101)-mean(data8(:,101),'omitnan');
Pelvis_tilt_y = data7(:,102)-mean(data8(:,102),'omitnan');
Pelvis_tilt_z = data7(:,103)-mean(data8(:,103),'omitnan');

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

kg = [75;75;75;75;75;73;73;73;73;73;70;70;70;70;70;67;67;67;67;67;65;65;65;65;65;86;86;86;86;86;94;94;94;94;94;66;66;66;66;66]; 

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
R_power = R_force.*B_angvel_f_lp*(pi/180).*0.06/kg(j,1);
% plot(R_power)
% hold on
% plot(-R_power_mo)
R_power_mo = R_moment.*(R_HIP_angvel_f_3)*(pi/180);
R_power_mo_s = R_moment.*(R_HIP_angvel_s_3)*(pi/180);
PSD_R_force = [];
PSD_R_force_r = [];
PSD_L_force_r = [];
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
    %     tempp36 = interp1(xx,yy_R_HIP_angvel_s_rad,xspa);
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
    %     PSD_R_HIP_angvel_s_rad = [PSD_R_HIP_angvel_s_rad tempp36'];
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
CON{j,11} = PSD_L_force_r;




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


%% Metabolic 
results = [];
flag = [];
% for j=1:4
j=3;
data2 = raw2{j};
data3 = raw3{j};
kg = [63;75;90]; 
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
T5 = mean(met_power_kg(flag(7,2):flag(7,1)))-ST;
result = [ST N1 T1 T2 T3 T4 T5];
% result = [ST N1 T1 T2 T3 T4];
results = [results ;result];

end
% 
met_power2 = 16.58*(xxss(124:162,1)/60)+4.51*(xxss(124:162,2)/60);
T3 = mean(met_power2)/kg(j,1)-ST;
met_power3 = 16.58*(xxss(499:529,1)/60)+4.51*(xxss(499:529,2)/60);
T4 = mean(met_power3)/kg(j,1)-ST;