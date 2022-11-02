%%
clc; clear all; close all;
%%
% directory = strcat('EXP_data\SOFT_EXOS\Abduction\ABD_force_ctl\');
% directory = strcat('Exp_data\FLEX\IMU\Day6\');
directory = strcat('Exp_data\FLEX\IMU_total\');
files1 = dir(directory);  
num_files1 = size(files1,1)-2;
for i = 1:num_files1
    file_name1 = files1(i+2).name;
    name1 = [directory file_name1];
%     raw{i} = csvread(name1,1,0) ;
%     raw{i} = xlsread(name1) ; 
    raw_imu{i} = readmatrix(name1); 
%     raw{i} =load(name1) ;
end

% directory2 = strcat('Exp_data\FLEX\FSR\Day6\');
directory2 = strcat('Exp_data\FLEX\FSR_total\');
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
% for j = 1:12
% raw_motions{1,j} = raw_motion{1,1}(:,24*(j-1)+2:24*j+1);
% end

%%
j=1; %% subject
q=23; %% trial
close all
clear -regexp ^L_ ^R_ ^ind ^B_ ^Q_ ^raw_EMG
clear i k time LHS RHS sample_EMG
%% variables - analog

data2 = raw_sync{24*(j-1)+q}(3:end,:);
% data2_timing = raw_sync{j}(3:end,1);
ind = find(~isnan(data2(:,1)),1,'last');

raw_EMG = data2(1:ind-1,[1,2,10]); %% [time , left foor , right foot]
sample_EMG =1/(raw_EMG(2,1)-raw_EMG(1,1));
raw_EMG_lp=lopass_butterworth(raw_EMG(:,2:3),5,sample_EMG,2);
raw_EMG_lp=[raw_EMG(:,1),raw_EMG_lp];

ind1 = raw_EMG(1,1):1/sample_EMG:raw_EMG(end,1);
% ind1 = 1/sample_EMG:1/sample_EMG:raw_EMG(end,1);
ind2 = 0:0.0025:raw_EMG(end,1);

raw_EMG_2 = interp1(ind1,raw_EMG_lp,ind2);
% raw_EMG_2 = interp1(ind1,raw_EMG_lp(1:end-1,:),ind2);
figure(1)
hold on
plot(raw_EMG(:,1),raw_EMG(:,2))
plot(raw_EMG_lp(:,1),raw_EMG_lp(:,2))
plot(raw_EMG_2(:,1),raw_EMG_2(:,2))

figure(2)
hold on
plot(raw_EMG(:,1),raw_EMG(:,3))
plot(raw_EMG_lp(:,1),raw_EMG_lp(:,3))
plot(raw_EMG_2(:,1),raw_EMG_2(:,3))

%%
data1 = raw_imu{1,24*(j-1)+q};
Q_sync = data1(:,34);
for k=1:length(Q_sync)-1
    if Q_sync(k,1) == 0 && Q_sync(k+1,1) == 1
       index1 = k+1;
    elseif Q_sync(k,1) == 1 && Q_sync(k+1,1) == 0 
       index2 = k; 
       break
    end
end

data = data1(index1:end-6000,:);   
time = (1:length(data))'*0.0025; 

L_ang = data(:,6);
L_ang_h = data(:,7);
L_ang_f = data(:,8);
L_acc_x = data(:,9);
L_acc_y = data(:,10);
L_acc_z = data(:,11);
L_angvel_s = data(:,24);
L_angvel_h = data(:,25);
L_angvel_f = data(:,26);

R_ang = data(:,12);
R_ang_h = data(:,13);
R_ang_f = data(:,14);
R_acc_x = data(:,15);
R_acc_y = data(:,16);
R_acc_z = data(:,17);
R_angvel_s = data(:,27);
R_angvel_h = data(:,28);
R_angvel_f = data(:,29);

B_ang = data(:,18);
B_ang_f = data(:,19);
B_ang_h = data(:,20);
B_acc_x = data(:,21);
B_acc_y = data(:,22);
B_acc_z = data(:,23);
B_angvel_s = data(:,30);
B_angvel_f = data(:,31);
B_angvel_h = data(:,32);


LHS = Gait_fsr(raw_EMG_2(:,1),raw_EMG_2(:,2),10);
RHS = Gait_fsr(raw_EMG_2(:,1),raw_EMG_2(:,3),10);

while LHS(end,1) > length(L_ang)
   LHS = LHS(1:end-1,1);
end
while RHS(end,1) > length(R_ang)
   RHS = RHS(1:end-1,1);
end

% [MHF,MHE,class] = find_MHFE(time,data,acc_z,c)
[LMHF,~,~] = find_MHFE(time,L_ang,B_acc_z,[85;100;85;100]);
[RMHF,~,~] = find_MHFE(time,R_ang,B_acc_z,[85;100;85;100]);



%%
figure(3)
hold on
plot(raw_EMG_2(:,1),raw_EMG_2(:,2))
plot(raw_EMG_2(LHS,1),raw_EMG_2(LHS,2),'r*')
plot(raw_EMG_2(:,1),raw_EMG_2(:,3))
plot(raw_EMG_2(RHS,1),raw_EMG_2(RHS,3),'b*')


figure(5)
% subplot(2,1,1)
hold on
plot(time,L_ang(:,1))
plot(time(LHS,1),L_ang(LHS,1),'r*')
plot(time(LMHF(:,3),1),L_ang(LMHF(:,3),1),'ro')
% subplot(2,1,2)
figure(51)
hold on
plot(time(:,1),R_ang(:,1))
plot(time(RHS,1),R_ang(RHS,1),'b*')
plot(time(RMHF(:,3),1),R_ang(RMHF(:,3),1),'bo')
figure(6)
hold on
ylim([0 ,2])
xlim([0 ,300])
plot((LHS(2:end,1)-LHS(1:end-1,1))*0.0025)
plot((RHS(2:end,1)-RHS(1:end-1,1))*0.0025)

    





