clc; clear all; close all;
GT.right=[];GT.left=[];
%%
a=dir('D:\test\1_220412\FSR\Day1'); a(1:2)=[];a=struct2cell(a)';a=a(:,1);
b=dir('D:\test\1_220412\IMU\Day1'); b(1:2)=[];b=struct2cell(b)';b=b(:,1);
name={'angle_00_1';'angle_00_2';'angle_00_3';'angle_00_4';'angle_02_1';'angle_02_2';'angle_02_3';'angle_02_4';
    'angle_04_1';'angle_04_2';'angle_04_3';'angle_04_4';'angle_06_1';'angle_06_2';'angle_06_3';'angle_06_4';
    'angle_08_1';'angle_08_2';'angle_08_3';'angle_08_4';'angle_10_1';'angle_10_2';'angle_10_3';'angle_10_4'};
for n=1:length(a)
    n
    raw.EMG=readmatrix(['D:\test\1_220412\FSR\Day1\' a{n,1}]);
    raw.EMG=raw.EMG(:,[1 2 10]);
    TF=isnan(raw.EMG(:,1));
    sample.EMG=1/raw.EMG(2,1); sample.IMU=400;
    raw.EMG(TF,:)=[]; 
    outlier.right=find(raw.EMG(:,3)==0); outlier.left=find(raw.EMG(:,2)==0);
    y=find(raw.EMG(:,2)>0); z=find(raw.EMG(:,3)>0);
    for nn=1:length(outlier.left)
        if outlier.left(nn)>y(1)
        raw.EMG(outlier.left(nn),2)=raw.EMG(outlier.left(nn)-1,2);
        end
    end
    for nn=1:length(outlier.right)
        if outlier.right(nn)>y(1)
        raw.EMG(outlier.right(nn),3)=raw.EMG(outlier.right(nn)-1,3);
        end
    end
    raw.EMG=js_Filter(raw.EMG(:,2:3),sample.EMG,5,4,2);
    raw.EMG=resample(raw.EMG,4000,5185);
    raw.EMG=raw.EMG(12001:(400*180),:);
    for nn=2:length(raw.EMG(:,1))
        if raw.EMG(nn-1,2)<10 && raw.EMG(nn,2)>=10 && raw.EMG(nn-1,2)~=0
            GT.right=[GT.right; nn-1];
        end
        if raw.EMG(nn-1,1)<10 && raw.EMG(nn,1)>=10 && raw.EMG(nn-1,1)~=0
            GT.left=[GT.left; nn-1];
        end
    end
    nnn=2;
    while GT.right(nnn)~=GT.right(end)
        if GT.right(nnn)-GT.right(nnn-1)<300
            GT.right(nnn)=[];
        end
        nnn=nnn+1;
    end
    nnn=2;
    while GT.left(nnn)~=GT.left(end)
        if GT.left(nnn)-GT.left(nnn-1)<300
            GT.left(nnn)=[];
        end
        nnn=nnn+1;
    end
    
%     figure;
%     plot(raw.EMG(:,1),'b'); hold on; plot(GT.left,raw.EMG(GT.left,1),'bo','linewidth',3);
%     plot(raw.EMG(:,2),'r'); hold on; plot(GT.right,raw.EMG(GT.right,2),'ro','linewidth',3);
%     legend('Left','Left HS','Right','Right HS');
%         
    raw.IMU=readmatrix(['D:\test\1_220412\IMU\Day1\' b{n,1}]);
    raw.IMU=raw.IMU(1:end-1,3:end);
    find(raw.IMU(:,31)==1);
    raw.IMU=raw.IMU(ans(1)+12000:(ans(1)+400*180-1),:);
    raw.IMU=js_Filter(raw.IMU,sample.IMU,5,4,2);
%     figure;
%     plot(raw.IMU(:,4)); hold on; plot(GT.left,raw.IMU(GT.left,4),'bo','linewidth',3);
%     plot(raw.IMU(:,10),'r'); hold on; plot(GT.right,raw.IMU(GT.right,10),'ro','linewidth',3);
%     legend('Left','Left HS','Right','Right HS');

%   L_Roll L_Pitch L_Yaw L_accX L_accY L_accZ R_Roll R_Pitch R_Yaw R_accX R_accY R_accZ	B_Roll B_Pitch B_Yaw B_accX	B_accY B_accZ L_angvel_X L_angvel_Y L_angvel_Z R_angvel_X R_angvel_Y R_angvel_Z	B_angvel_X B_angvel_Y B_angvel_Z
    assignin('base', [name{n},'_Angle_left'], [raw.IMU(:,4:6)]);
    assignin('base', [name{n},'_Angle_right'], [raw.IMU(:,10:12)]);
    assignin('base', [name{n},'_Angle_belly'], [raw.IMU(:,16:18)]);
    assignin('base', [name{n},'_AngularVelocity_left'], [raw.IMU(:,22:24)]);
    assignin('base', [name{n},'_AngularVelocity_right'], [raw.IMU(:,25:27)]);
    assignin('base', [name{n},'_AngularVelocity_belly'], [raw.IMU(:,28:30)]);
    assignin('base', [name{n},'_FSR_left'], [raw.EMG(:,1)]);
    assignin('base', [name{n},'_FSR_right'], [raw.EMG(:,2)]);
    assignin('base', [name{n},'_HS_left'], [GT.left]);
    assignin('base', [name{n},'_HS_right'], [GT.right]);
    GT.right=[];GT.left=[];
end