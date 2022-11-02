%%
clc; clear all; close all;
directory2 = strcat('EXP_data\SOFT_EXOS\FLEX\20211029_1st_year\');
files2 = dir(directory2);  
num_files2 = size(files2,1)-2;
for i = 1:num_files2
    file_name2 = files2(i+2).name;
    name2 = [directory2 file_name2];
    raw2{i} = readmatrix(name2);
    raw3{i} = readtable(name2);
end
data2 = raw2{2};
data3 = raw3{2};
kg = 73 ; 
Vo2 = data2(4:end,15);
Vco2 = data2(4:end,16);
met_power = 16.58*(Vo2/60)+4.51*(Vco2/60);
met_power_kg = met_power/kg; 
time_const = 1.15741E-05;

time = data2(4:end,10)/(time_const);
phase = table2array(data3(3:end,34));
num1 = 1;
for i = 1:length(time)-1
    if phase{i}(1) == 'E' && phase{i+1}(1) == 'R'
       flag(num1,1) = i;
       [~,flag(num1,2)] = min(abs(time-(time(i)-120)));
       num1 = num1+1;
    end
end

ST = mean(met_power_kg(flag(1,2):flag(1,1)));
% POFF = mean(met_power_kg(flag(2,2):flag(2,1)))-ST;
POFF = mean(met_power_kg(flag(2,2):flag(2,1)));
POFF_raw = met_power_kg(flag(2,2):flag(2,1));
POFF_raw(:,2) = Vo2(flag(2,2):flag(2,1))/60;
POFF_raw(:,3) = Vco2(flag(2,2):flag(2,1))/60;
PON = [];
j=0;

k = flag(1,1);
[~,start] = min(abs(time-(time(k)-360)));
while true
    PON = [PON ; mean(met_power_kg(start+j:start+20+j))];
    if start+20+j == k
        break
    end
    j=j+1;
end

PON(:,2) = ((POFF-PON(:,1))/POFF)*100;


PON = mean(met_power_kg(start+10:start+20+10));
PON_raw = met_power_kg(start+10:start+20+10);
PON_raw(:,2) = Vo2(start+10:start+20+10)/60;
PON_raw(:,3) = Vco2(start+10:start+20+10)/60;
