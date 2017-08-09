function [y,y_det,c_err,c_std]=cmm_rbpf(mp_flag) %output the localization error, estimation error of the common error, and its std over particles
%this is a test program for Rao Blackwellized particle filter
%for CMM localization. The methodology is to use particle to represent the 
%distribution of the common pseudorange error while use EKF to represent the 
%conditional vehicle pose distribution. Since the vehicle poses are conditionally
%independent given the common error. The dimension of the EKF is relatively small.
%This filter is actually centralized since the particle filter maintains
%all the information available. The map matching is treated as measurement
%to determine the weight of each particle. 
%clear all
% for tt=1:10
%created by macshen
% tt
clearvars -except tt record_mean max_error mp_flag
close all
global pf;
N=4; % # of total vehicles 
% load('distance_and_gradient.mat');  %contains grid_size, distance and gradient of distance from road centers
% global dis;
% global grad;   %global quantities to share with sub-function for interpolating distances
% dis=distance;    
% grad=grad_dis;

Ng=2000;   % # of discrete grids of multipath estimation for each lane
mp_gridsize=0.25;   % gridsize*Ng=lane length
lane_width=3.5;   %width of a single lane
vehicle_width=1.8;  %use vehicle width to increase the positioning accuracy, it is assumed that the GPS receiver locates at the center of the vehicle
velocity=2;  %vehicle velocity
Ns=150;  % #of simulation time points
Nsv=6;  %#of visible satellites
Np=200; % #of particles

%redundent
mpmat=cell(N,1);   %multipath error time history for N vehicles
for i=1:N
mpmat{i}=mpgen(24,1200,2,floor(10000*rand(1))+54321+i); %generate multipath error, use different random seeds
end
%mpmat{1}(1:Ns,1)=10000*sin(1/100*(1:Ns))';
orgllh = [40*pi/180 80*pi/180 0];  %origin of the local coordinate system, first latitude, second longitudinate 
orgxyz = llh2xyz(orgllh);   %convert the origin to the xyz ECEF cooridinate system
loadgps
startt=500; t = startt; deltat=0.1;

% usrenu{1}(1:Ns,1)=(6:velocity*deltat:6+velocity*deltat*(Ns-1))';
% usrenu{1}(1:Ns,1)=0.5*((1:Ns)*deltat).^2;
% 
% usrenu{1}(1:Ns,2)=50-lane_width/2;
% usrenu{1}(1:Ns,3)=0;
for k=1:1
usrenu{k}(1:Ns,1)=(16:velocity*deltat:16+velocity*deltat*(Ns-1))';
usrenu{k}(1:Ns,2)=250-lane_width/2;
usrenu{k}(1:Ns,3)=0;
end
for k=2:2
usrenu{k}(1:Ns,1)=(473:-velocity*deltat:473-velocity*deltat*(Ns-1))';
usrenu{k}(1:Ns,2)=250+lane_width/2;
usrenu{k}(1:Ns,3)=0;
end
for k=3:3
usrenu{k}(1:Ns,2)=(16:velocity*deltat:16+velocity*deltat*(Ns-1))';
usrenu{k}(1:Ns,1)=250+lane_width/2;
usrenu{k}(1:Ns,3)=0;
end
for k=4:4
usrenu{k}(1:Ns,2)=(473:-velocity*deltat:473-velocity*deltat*(Ns-1))';
usrenu{k}(1:Ns,1)=250-lane_width/2;
usrenu{k}(1:Ns,3)=0;             %generate the vehicles' path in the local coordinate system
end
%EndLoop = max(size(usrenu));
%bar1 = waitbar(0,'Generating User Solutions...  ');
%id=[]; %added by macshen
for k=1:N
pr_err{k}=[]; %added by macshen
mp_err{k}=[];
end
state=[];

        sigma_a2=1;  %unmodeled position noise caused by acceleration,assuming the variance of acceleration is 1.
        sigma_b2=1;
        sigma_d2=1;   %specify the uncertainty level of the clock bias
    noise_scaling_factor=[1 1 0 0 1];
    for k=1:N
    white_noise{k}=noise_scaling_factor(1)^2*randn(Ns,Nsv);
    end
%     map_mp=mp_generation(Nsv,[0,0,0,1,0,1],Ng,mp_gridsize);%generate mp according to geometry
%     map_mp=abs(map_mp);
%load('mp.mat');
map_mp=[];
%         sigma_m2=1;   %mp variance
%         sigma_md2=1;   %mp drift variance


% load('mp_x.mat');
% load('mp_y.mat');
load('mp_30.mat');
global mp_xg;
global mp_yg;
global LOS_xg;
global LOS_yg;   %global variable shared with add_mp_wn

for k=1:Nsv
mp_x{k}=mp_x{k}+0.1*abs(randn(size(mp_x{k})));
mp_y{k}=mp_y{k}+0.1*abs(randn(size(mp_y{k})));   %perturb to simulate diffraction
wavelength=3*10^8/1575.42/10^6;   % L1 signal wavelength
phase_x=2*pi*mp_x{k}/wavelength;
phase_y=2*pi*mp_y{k}/wavelength;

%noted mp error is eliminated
mp_x{k}=0.5*mp_x{k}.*cos(phase_x)*0.0001;
mp_y{k}=0.5*mp_y{k}.*cos(phase_y)*0.0001;
LOS_x{k}=ones(1920,40);
LOS_y{k}=ones(1920,40);
end

mp_xg=mp_x;
mp_yg=mp_y;
LOS_xg=LOS_x;
LOS_yg=LOS_y;

if mp_flag==0
for k=1:Nsv
    mp_xg{k}=zeros(1920,40);
    mp_yg{k}=zeros(1920,40);
    LOS_xg{k}=ones(1920,40);
    LOS_yg{k}=ones(1920,40);
end
end



for i = 1:Ns  %draw the first step
    t = t + deltat;

    for k=1:N   %GPS measurement loop for all the vehicles
    usrxyz{k}=enu2xyz(usrenu{k}(i,:),orgxyz);
    [svxyzmat{k},svid{k},elevation_angle{k}] = gensv(usrxyz{k},t,5,[],Nsv);   % here 5 is the mask angle to determine which satellites are visible to the vehicles, macshen
   % id=[id;svid(1)]; The last argument is to specify the wanted number of satellite and give the first group of  available satellites %added by macshen
    [prvec{k},adrvec{k},pr_err_vec{k},common_error(i,:)] = genrng_common_noise(k,usrxyz{k},svxyzmat{k},svid{k},t,noise_scaling_factor,[],mpmat{k});  %pr_err_vec added by macshen, receiver identification k seems to be redundant here. the magnitude of noise flag is thermal,Tropospheric,SA,Multipath,Ionospheric
    [prvec{k},pr_err_vec{k},mp_err_vec{k}]=add_mp_wn(usrenu{k}(i,:),prvec{k},pr_err_vec{k},map_mp,mp_gridsize,Ng,white_noise{k}(i,:));
%    [estusr{k},H{k}] = olspos(prvec{k},svxyzmat{k});  %sovle for the vehicle pose given pr measurement, macshen
%     
%    estenu{k}(i,:) = ( xyz2enu(estusr{k}(1:3),orgxyz) )';   %transform the ECEF coordinate to local one with origin orgxyz
%    err{k}(i,1:3) = estenu{k}(i,1:3) - usrenu{k}(i,:);
%    terr{k}(i) = estusr{k}(4);  % true clk bias is zero
    end   %end for loop of vehicle #
     
    for k=1:N
     pr_err{k}=[pr_err{k};pr_err_vec{k}]; %added by macshen
     mp_err{k}=[mp_err{k};mp_err_vec{k}];
    end
%     P0=inv(H{1}.'*H{1});
%     pf=zeros(Np,3); %first two colomns are coordinates, 3rd colomn is weight,
%     %here particles represents the offset vector that correct the vehicles' pose, therefore the particles are common to each vehicle
%     %particles are initilized every time step
%     pf(:,3)=1/Np;   %initialize weight
%     pf(:,1:2)=mvnrnd([0,0],9*P0(1:2,1:2),Np);   %try Gaussian sampling, this should be modified later
sigma_thermal2=1.0*noise_scaling_factor(1)^2;


    figure(1);
    clf;
    hold on
    

%     for k=1:N  %pf loop for each vehicle
        sigma_multipath2=1.67*noise_scaling_factor(4)^2;   %variance of mutlipath,calculated from the multipath noise in the simulation
%         sigma_eta2=sigma_multipath2+sigma_thermal2;
%         sigma_map2=0.01;  %variance of map inprecision, this might be larger for real map
        

if i==1   %for the first time step, initialize particles for ego-state estimation and multipath mitigation
initialize_pf_CMM(Np,Nsv,N,common_error);
for k=1:N
update_pf_CMM(prvec{k},svxyzmat{k},sigma_thermal2,orgxyz,Np,k);   %pf update given measurement
resample_CMM;  %think about it, resample for every vehicle update or for the whole update?
weight_CMM(k);  %calculate weight according to the map constraints
resample_CMM;
end
else  %second step and so on
predict_pf_CMM(sigma_a2,sigma_b2,sigma_d2,deltat,N,Np,Nsv);
for k=1:N
update_pf_CMM(prvec{k},svxyzmat{k},sigma_thermal2,orgxyz,Np,k);   %pf update given measurement
resample_CMM;
weight_CMM(k);  %calculate weight according to the map constraints
resample_CMM;
end
end

    [mu,cov]=pf_to_gaussian_RBPF;
%         pf=resample(pf);  %resample, %weight needn't be normalized. After resample, weight has been normalized

 
    %calculate the determinant of the variance of the true error, this is expected to be related with the estimation covariance
    
    plotcov2d(usrenu{1}(i,1),usrenu{1}(i,2),[1,0;0,1],'r',0,0,0,1);  %plot the true position of vehicle as a circle
    for k=1:Np
    plotcov2d(pf(k).mu{1}(1),pf(k).mu{1}(3),pf(k).cov{1}([1,3],[1,3]),'b',0,0,0,3);  %plot the estimated position of vehicle as a circle
    end
    plotcov2d(mu(1),mu(2),cov,'g',0,0,0,3);
    err_CMM(i)=norm(mu'-usrenu{1}(i,1:2));
    deter(i)=det(cov);
    
    %when not Kalman
    %plotSamples([pf(:,1)+estenu{1}(i,1),pf(:,2)+estenu{1}(i,2)],'r');  %plot the estimated position sample of 1st vehicle
    %plotcov2d(mu(1)+estenu{1}(i,1),mu(2)+estenu{1}(i,2),cov,'g',0,0,0,3);  %plot the true position of vehicle as a circle
    %when use Kalman filter
    %plotSamples([pf(:,1)+state_mean{1}(1,1),pf(:,2)+state_mean{1}(2,1)],'r');  %plot the estimated position sample of 1st vehicle
    %plotcov2d(mu(1)+state_mean{1}(1,1),mu(2)+state_mean{1}(2,1),cov,'g',0,0,0,3);  %plot the true position of vehicle as a circle
    
    axis equal;
    pause(0.0001);
    
for k=1:Np
    er_com(k)=pf(k).common(1)-common_error(i,1);
end
es_com_err(i)=mean(er_com);   %the estimation error of common error
es_com_std(i)=sqrt(var(er_com));

end  %end (for i = 1:Ns)
y=err_CMM;
y_det=deter;
c_err=es_com_err;
c_std=es_com_std;
end
%close(bar1);
% x=100*rand(1000,1);
% y=100*rand(1000,1);
% for k=1:1000
% d(k)=Distance_to_road([x(k),y(k)],grid_size);    %pay attention to changing the grid-size if 
% end
% record_mean(tt)=mean(err_CMM);
% max_error(tt)=max(err_CMM);
% end