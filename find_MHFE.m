function [MHF,MHE,class] = find_MHFE(time,data,acc_z,c)
% [MHF,MHE,class] = find_MHFE(time,data,acc)
%%%%%%%%%   각도데이터를 넣으면 MHF을 추출하는 함수이다. 
%%%%%%%%%   (+) : flexion , (-) : extension

HF = 0 ; 
MHF = 0 ; 
time_HF = 0 ;
cross = 0 ;
num1 = 1;
gc = 0 ;
%%%%%%%%%%%%%%%
HE = 0 ; 
MHE = 0 ; 
time_HE = 0 ;
cross_E = 0 ;
num2 = 1;
gc_e = 0 ;
ang_HE = 0 ;

DPF = c(1,1);
MPF = c(2,1);
DPE = c(3,1);
MPE = c(4,1);

% acc = [time ; acc_z];

% onset = [];
% peak = [];
% release = [] ; 
% ends = [] ; 
% 
data_E = 180-data;
class = [] ;
CLAS = [] ; 
for i = 3:length(data) 
    if data(i-1)< MPF && data(i) > MPF && HF == 1 %% 검출선
        cross = 1 ;
        HF = 0;
    end
    
    if data(i-1)< DPF && data(i)> DPF && cross_E == 0 %% 기준선
        HF = 1 ;
    end
    
    if data(i-2)<=data(i-1) && data(i-1)>data(i) && cross == 1 
%         MHF = [MHF ; time(i)] ; 
        MHF(num1,1) = time(i-1) ;
        MHF(num1,2) = data(i-1) ;
        MHF(num1,3) = i-1 ; 
        time_HF = time(i-1) ;
        HF = 0 ;
        cross = 0 ; 
        num1 = num1+1;
    end
    
    if data_E(i-1)< MPE && data_E(i) > MPE  && HE == 1  %% 검출선   사람마다 많이 다르다. 적당히 조절해줘야함.
        cross_E = 1 ;  
        HE = 0 ; 
    end
    
    if data_E(i-1)< DPE && data_E(i)> DPE && cross == 0 %% 기준선
        HE = 1 ;
    end
    
    if data_E(i-2)<=data_E(i-1) && data_E(i-1)>data_E(i) && cross_E == 1 
%         MHF = [MHF ; time(i)] ; 
%         MHE(num2,1) = time(i-1) ;
%         MHE(num2,2) = data(i-1) ;
        time_HE = time(i-1) ;
        ang_HE = data_E(i-1);
        HE = 0 ;
        cross_E = 0 ; 
%         for k=1:10 
%             if ang_HE < data_E(i+k)
%                 time_HE = time(i+k) ;
%                 ang_HE = data_E(i+k); 
%             end 
%         end
        MHE(num2,1) = time_HE ;
        MHE(num2,2) = -ang_HE ;
        MHE(num2,3) = i-1 ;
       if acc_z(i-1,1) < 0
          class(num2,1) = 1 ; % running
          class(num2,2) = acc_z(i-1,1);
       else 
          class(num2,1) = 0 ; % walking
          class(num2,2) = acc_z(i-1,1);
       end
       num2 = num2+1; 
     end
        
        
    end
     
%     
% hold on
% plot(Vel_acc(:,1),Vel_acc(:,2))
% plot(MHE(:,1),Vel_acc(MHE(:,1),2),'r*')
%     
% end
% plot(time,data) 
% hold on
% plot(MHF(:,1),MHF(:,2),'*r')
% plot(MHE(:,1),MHE(:,2),'*b')

end


