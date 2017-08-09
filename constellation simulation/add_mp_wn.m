function [pr,err_vec,mp_err]=add_mp_wn(usrenu,pre_pr,pre_err_vec,map_mp,mp_gridsize,Ng,white_noise);  

global mp_xg;
global mp_yg;
global LOS_xg;
global LOS_yg;

% if usrenu(1)>246.5&usrenu(1)<253.5
%     N=ceil(usrenu(2)/mp_gridsize);    %0.2 is the grid_size
%     pr=pre_pr+map_mp(Ng+N,:);   %500 is the # of grid per lane
%     err_vec=pre_err_vec+map_mp(Ng+N,:);
% end
    if usrenu(2)>246.5&usrenu(2)<250  %vehicle 1
    N=ceil(usrenu(1)/mp_gridsize);
    N_sv=numel(mp_xg);
    for k=1:N_sv
     mp(1,k)=mp_xg{k}(N,13);
     if LOS_xg{k}(N,13)==0   %LOS does not exist
%         mp(1,k)=mp(1,k)*0.3;
%      else   %LOS block
%          if abs(mp(1,k))<10^-3   %NLOS does not exist
             mp(1,k)=1000;   %assign a large mp_error
%          end
     end
    end
    end
    
    if usrenu(2)<253.5&usrenu(2)>250  %vehicle 2
    N=ceil(usrenu(1)/mp_gridsize);
    N_sv=numel(mp_xg);
    for k=1:N_sv
     mp(1,k)=mp_xg{k}(N,27);
     
     if LOS_xg{k}(N,27)==0   %LOS does not exist
%         mp(1,k)=mp(1,k)*0.3;
%      else   %LOS block
%          if abs(mp(1,k))<10^-3   %NLOS does not exist
             mp(1,k)=1000;   %assign a large mp_error
%          end
     end
     
    end
    end


    if usrenu(1)>246.5&usrenu(1)<250  %vehicle 3
    N=ceil(usrenu(2)/mp_gridsize);
    N_sv=numel(mp_yg);
    for k=1:N_sv
     mp(1,k)=mp_yg{k}(N,27);
     if LOS_yg{k}(N,27)==0   %LOS does not exist
%         mp(1,k)=mp(1,k)*0.3;
%      else   %LOS block
%          if abs(mp(1,k))<10^-3   %NLOS does not exist
             mp(1,k)=1000;   %assign a large mp_error
%          end
     end
    end
    end
    
    if usrenu(1)>250&usrenu(1)<253.5  %vehicle 3
    N=ceil(usrenu(2)/mp_gridsize);
    N_sv=numel(mp_yg);
    for k=1:N_sv
     mp(1,k)=mp_yg{k}(N,13);
     if LOS_yg{k}(N,13)==0   %LOS does not exist
%         mp(1,k)=mp(1,k)*0.3;
%      else   %LOS block
%          if abs(mp(1,k))<10^-3   %NLOS does not exist
             mp(1,k)=1000;   %assign a large mp_error
%          end
     end
    end
    end
    
    %mp=zeros(1,6);
    pr=pre_pr+mp;
    err_vec=pre_err_vec+mp+white_noise;
    mp_err=mp;
    pr=pr+white_noise;  
end