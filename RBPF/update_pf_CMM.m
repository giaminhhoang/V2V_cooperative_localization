function update_pf_CMM(pr,svxyzmat,sigma_thermal2,orgxyz,Np,v_id)  % The last argument is the vehicle id
global pf;
dx=1;  %here the difference step is chosen large such that floating error will not dominate
sv_n=length(pr);
p0=1/sqrt(2*pi*sigma_thermal2)*exp(-0.5*chi2inv(0.99,1));  %for multipath 
for k=1:Np
    pose_local=[pf(k).mu{v_id}(1),pf(k).mu{v_id}(3),0];  
    pose_xyz=enu2xyz(pose_local,orgxyz);  %give xyz as column vector
    for j=1:sv_n
    range_pre(j,k)=norm(pose_xyz.'-svxyzmat(j,:))+pf(k).mu{v_id}(5)+pf(k).common(j);  %predicted pr should include the common error    
    end
    innovation(:,k)=pr.'-range_pre(:,k);
end    

    C=zeros(sv_n,6);    %generate the observation derivative matrix
    %calculate derivative numerically, this observation matrix is
    %calculated only once using the last particle's pose estimation because
    %it is very insensitive to vehicle location
    for j=1:sv_n
        pose_xyz_add=enu2xyz(pose_local+[dx,0,0],orgxyz);
        C(j,1)=norm(pose_xyz_add.'-svxyzmat(j,:))-norm(pose_xyz.'-svxyzmat(j,:));
        C(j,1)=C(j,1)/dx;
        pose_xyz_add=enu2xyz(pose_local+[0,dx,0],orgxyz);
        C(j,3)=norm(pose_xyz_add.'-svxyzmat(j,:))-norm(pose_xyz.'-svxyzmat(j,:));
        C(j,3)=C(j,3)/dx; 
        C(j,5)=1; 
    end
    extract=[1,3,5];
    for j=1:Np
    [y(j,:),D2(j,:)]=individual_compatibility_test(innovation(:,j),C,sigma_thermal2,j,v_id,2);  %check for the compatibility of the measurement, throw away the incompatible measurement    
    %calculate weight
%     y(j,:)=ones(1,6);
    
    for k=1:sv_n
        if y(j,k)==1
            Q=sigma_thermal2+C(k,extract)*pf(j).cov{v_id}(extract,extract)*C(k,extract).';
            pf(j).weight=pf(j).weight*1/sqrt(2*pi*Q)*exp(-0.5*innovation(k,j)^2/Q);
        else  %if y(j,k)==0
            pf(j).weight=pf(j).weight*p0;
        end
    end
    end
%     innovation(:,1)
%     y(1,:)

    %y=ones(1,sv_n);
    for j=1:Np
    extract_compatible=[];
    for k=1:length(y(j,:))
        if y(j,k)==1
            extract_compatible=[extract_compatible,k];   %extract the compatible measurement indices
        end
    end
    N_com=length(extract_compatible);  %'com' stands for compatible
    if N_com~=0   %only update when there is compatible measurement, otherwise, do not update
    C_com=C(extract_compatible,:);
    innovation_com=innovation(extract_compatible,j);
    Q=eye(N_com,N_com)*sigma_thermal2;
%     if v_id==1
%         for m=1:length(extract_compatible)
%             if extract_compatible(m)==3|extract_compatible(m)==5|extract_compatible(m)==6
%                 Q(m,m)=Q(m,m)*9;
%             end
%         end
%     end
%         if v_id==2
%         for m=1:length(extract_compatible)
%             if extract_compatible(m)==3|extract_compatible(m)==5|extract_compatible(m)==6
%                 Q(m,m)=Q(m,m)*9;
%             end
%         end
%         end
%                 if v_id==3
%         for m=1:length(extract_compatible)
%             if extract_compatible(m)==2|extract_compatible(m)==3|extract_compatible(m)==5
%                 Q(m,m)=Q(m,m)*9;
%             end
%         end
%                 end
%                         if v_id==4
%         for m=1:length(extract_compatible)
%             if extract_compatible(m)==3|extract_compatible(m)==4|extract_compatible(m)==5
%                 Q(m,m)=Q(m,m)*9;
%             end
%         end
%         end
    
    
    K=pf(j).cov{v_id}*C_com.'*(Q+C_com*pf(j).cov{v_id}*C_com.')^-1;
    pf(j).mu{v_id}=pf(j).mu{v_id}+K*innovation_com;
    pf(j).cov{v_id}=(eye(6)-K*C_com)*pf(j).cov{v_id};   %for large multipath error, EKF estimation can be over-confident, try using schmidtz EKF instead
    end
    end
end



function [y,D2]=individual_compatibility_test(innovation,C,sigma_thermal2,particle_id,v_id,flag)%individual compatibility test
%flag=1 means determine compatibility in a deterministic way, flag=2 means
%determine in a probabilistic way
global pf;
N=length(innovation);
extract=[1,3,5];  %here the clock offset is also accounted for
P_state=pf(particle_id).cov{v_id}(extract,extract);  %extract the submatrix of pose covariance
for k=1:N
    H=C(k,extract);
    P=H*P_state*H.'+sigma_thermal2;
    D2(k)=innovation(k,1)^2/P;
    
    if flag==1
if (D2(k)<=chi2inv(0.99,1))
    y(k)=1;  %indicate compatible
else
    y(k)=0;
end


    end
    
    if flag==2
        
    if (D2(k)<=chi2inv(0.95,1))  %the restriction is much more stringent here since measurement of moderate error is accepted in a probabilistic way
    y(k)=1;  %indicate compatible
    else 
        if randn>20*(chi2cdf(D2(k),1)-0.95)
        y(k)=1;
        else
        y(k)=0;    %This process allow particles to have more diversity
        end
    end
    
%     if (D2(k)>=chi2inv(0.999,1))
%         y(k)=0;
%     end

    end

end  %end for k=1:N
end  %end for function compatibility_test

%joint compatibility have not been modified to accomodate the
%Rao-blackwellized particle filter
function y=joint_compatibility_test(innovation,C,sigma_thermal2)
global EKF;
global Best_pair;
global z_JCBB;
global C_JCBB;
z_JCBB=innovation;
C_JCBB=C;
N=length(innovation);
Best_pair=zeros(1,N);
JCBB([],1,sigma_thermal2);
y=Best_pair;
end    


function JCBB(H,i,sigma_thermal2)
global EKF;
global Best_pair;
global z_JCBB;
global C_JCBB;
N=length(z_JCBB);
if (i>N)
    if sum(H>0)>sum(Best_pair>0)
    Best_pair=H;
    end  
else % else for if (i>N)
        if unitary_compatible(i,sigma_thermal2)
        if joint_compatible(H,i,sigma_thermal2)
        JCBB([H 1],i+1,sigma_thermal2);
        end
        end
    if sum(H>0)+N-i>sum(Best_pair>0)
        JCBB([H 0],i+1,sigma_thermal2);
    end
end  %end for if (i>N)
end  %end JCBB


function y=unitary_compatible(i,sigma_thermal2)
global EKF;
global z_JCBB;
global C_JCBB;
extract=[1,3,5];  %here the clock offset is also accounted for
P_state=EKF.cov(extract,extract);  %extract the submatrix of pose covariance
    H=C_JCBB(i,extract);
    P=H*P_state*H.'+sigma_thermal2;
    D2=z_JCBB(i,1)^2/P;
if D2<=chi2inv(0.99,1)
    y=1;
else
    y=0;
end
end


function y=joint_compatible(H,i,sigma_thermal2)
global EKF;
global z_JCBB;
global C_JCBB;
extract=[];
for k=1:length(H)
    if H(k)==1
    extract=[extract,k];
    end
end
extract=[extract,i];  %this is the extracted indices for measurement
extract_pose=[1,3,5];
P_state=EKF.cov(extract_pose,extract_pose);
H=C_JCBB(extract,extract_pose);
CH=H*P_state*H.'+eye(length(extract))*sigma_thermal2;
innovation_JCBB=z_JCBB(extract);
DH2=innovation_JCBB.'*CH^(-1)*innovation_JCBB;       
        if DH2<=chi2inv(0.99,length(extract))  %here N is the total length of H but NN is the # of null hypothesis
            y=1;
        else
            y=0;
        end  %end for if
end  %end for the function