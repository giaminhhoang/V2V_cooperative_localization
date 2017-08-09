function predict_pf_CMM(sigma_a2,sigma_b2,sigma_d2,deltat,N,Np,Nsv)
%sigma_x and sigma_x_dot are the variance of unmodelled acceleration variance alone the lane
%sigma_b,sigma_b_dot are variance of clock bias
global pf;
s_y=0.01;   %scaling factor for the lateral acceleration
Q_a=[sigma_a2*0.25*deltat^4, sigma_a2*0.5*deltat^3;
    sigma_a2*0.5*deltat^3,  sigma_a2*deltat^2];
Q_c=[sigma_b2*deltat^2+0.25*sigma_d2*deltat^4,0.5*sigma_d2*deltat^3;
    0.5*sigma_d2*deltat^3, sigma_d2*deltat^2];
% Q_m=[sigma_m2*deltat^2+0.25*sigma_md2*deltat^4,0.5*sigma_md2*deltat^3;
%     0.5*sigma_md2*deltat^3, sigma_md2*deltat^2];   %Noted that the sigma_a2 is of higher order in deltat, this is to account for the truncation error in the propagation equation
Sigma(1:2,1:2)=Q_a;  %lanewise
Sigma(3:4,3:4)=s_y*Q_a;  %lateral
Sigma(5:6,5:6)=Q_c;
Sigma_2(1:2,1:2)=s_y*Q_a;  %lanewise
Sigma_2(3:4,3:4)=Q_a;  %lateral
Sigma_2(5:6,5:6)=Q_c;
A=[1 deltat;
    0 1];
aug_A=zeros(6,6);
aug_A(1:2,1:2)=A;
aug_A(3:4,3:4)=A;
aug_A(5:6,5:6)=A;
% for k=1:(EKF.N-6)/2
%     Sigma(6+2*k-1:6+2*k,6+2*k-1:6+2*k)=Q_m;
% end
%prediction
for j=1:Np
    pf(j).common=pf(j).common+0.1*randn(1,Nsv);
    for i=1:N
    pf(j).mu{i}=aug_A*pf(j).mu{i};
    pf(j).cov{i}=aug_A*pf(j).cov{i}*aug_A.';
    if i==1|i==2
    pf(j).cov{i}=pf(j).cov{i}+Sigma; 
    else
    pf(j).cov{i}=pf(j).cov{i}+Sigma_2;  
    end
    end
end
   
end