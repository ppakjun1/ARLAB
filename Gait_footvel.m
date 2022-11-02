function [HS,TO,ST] = Gait_footvel(mf,a)

% This code is gait event detection from fsr sensor
% time : temporal information
% mf = vertical velocity of midfoot
% 
% 
p1 = a(1) ;
p2 = a(2) ;
p3 = a(3);
p4 = a(4) ;

% flag = 0 ; 
% num1 = 1 ; 
% cheakp = [] ; 
% temp = [0,0] ; 
% flag2 = 0 ; 
% f = 50 ; 
% m1=0;
% m5=0;

% [~,TO] = findpeaks(mf,'MinPeakHeight',150);
% [cc,bb] = findpeaks(-mf,'MinPeakHeight',50,'MinPeakProminence',50);
% % 
% [~,TO] = findpeaks(mf,'MinPeakHeight',p1,'MinPeakProminence',p3);
% [cc,bb] = findpeaks(-mf,'MinPeakHeight',p2,'MinPeakProminence',p3,'MinPeakDistance',15);
% num1=1;
% for k=1:length(cc)
%     if cc(k,1) < p4
%     HS(num1,1) = bb(k,1);
%     num1=num1+1;
%     end    
% end
% num2 = 1;
% for k=1:length(TO)
%     for i=1:29
%         if TO(k)+i == length(mf)
%             break
%         end
%         if mf(TO(k)+i) >= 0 && mf(TO(k)+i+1) <0
%             ZC(num2,1) = TO(k)+i+1;
%             num2=num2+1;
%         end
%         if num2 == length(TO)-1
%             break
%         end
%     end
% end
hs = 0;
ms = 1;
num1 = 2;
num2 = 2 ;
HS(1,1) = 0;
TO(1,1) = 0;
temp = 1;
for k=2:length(mf)-1
    if ms == 1 && mf(k,1) > mf(k-1,1) && mf(k,1) > mf(k+1) && mf(k,1) > p1 && k > HS(end,1) + 30
        hs = 1 ; 
        ms = 0;
        TO(num1,1) = k;
        num1 = num1+1 ;
        temp = 0;
    elseif hs == 1 && k >= TO(end,1)+29 && mf(k,1) < mf(k-1,1) && mf(k,1) < mf(k+1,1)
%         
        HS(num2,1) = k;
        num2 = num2+1;
        hs = 0;
        ms = 1;
        temp = 1;
    end
    ST(k,1) = temp; 
end

HS = HS(2:end,1);
TO = TO(2:end,1);

end
    







