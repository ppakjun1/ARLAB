function [HS] = Gait_fsr(time,h,e)
% [HS,TO] = Gait_fsr(time,h,t,e)
% This code is gait event detection from fsr sensor
% time : temporal information
% a/b/c/d : fsr signal of each spot
% e : treshold (%), 5% is recommended

% flag = 0 ; 
% num1 = 1 ; 
% cheakp = [] ; 
% temp = [0,0] ; 
% flag2 = 0 ; 
% f = 50 ; 
% m1=0;
% m5=0;

% mx_h = max(h);
% mn_h = min(h);
% mx_t = max(t);
% mn_t = min(t);
% ave_psd = ave/mx;
% h1 = ((h)/mx_h)*100 ;
% t1 = ((t-mn_t)/(mx_t-mn_t))*100 ;

num1=1;
num2=1;

%%%%%%%%%% Find LHS RHS 
for i = 2:length(time)
    if h(i,1) > e 
        flag_h = 1;
    else
        flag_h = 0 ;
    end
%      if mt(i,1) > e1
%         flag_mt = 1;
%     else
%         flag_mt = 0 ; 
%      end
%      if mf(i,1) > e1
%         flag_mf = 1;
%     else
%         flag_mf = 0 ; 
%     end
%     if t1(i,1) > e
%         flag_t = 1;
%     else
%         flag_t = 0 ; 
%     end
%     flag_sum = flag_h + flag_t ;
    
    if num1 == 1 
    if h(i-1,1) < e && h(i,1) >= e 
       HS(num1,1) = i ;
%        L_HS(num1,2) = L_heel_lp(i,1);
       num1=num1+1;
    end
    else 
    if h(i-1,1) < e && h(i,1) >= e 
       if i - HS(num1-1,1) < 320
       else
       HS(num1,1) = i ;
%        L_HS(num1,2) = L_heel_lp(i,1);
       num1=num1+1;
       end
    end
    end  
%     
%     if num2 ==1 
%        if t1(i-1,1) > e && t1(i,1) <= e && flag_sum ==0 
%            TO(num2,1) = i ;
%            num2=num2+1;
%        end
%     else
%        if t1(i-1,1) > e && t1(i,1) <= e && flag_sum ==0  
%           if i - TO(num2-1,1) < 60
%           else
%              TO(num2,1) = i ;
%              num2=num2+1;
%           end
%        end
%     end
%  
end  
    



