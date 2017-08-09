function [new_mean,new_cov]=Kalman_prediction(state_mean,state_cov,sigma_x,sigma_x_dot,deltat)
A=eye(4,4)+deltat*[0 0 1 0;
                   0 0 0 1;
                   0 0 0 0;
                   0 0 0 0];
new_mean=A*state_mean;
cross_cov=sqrt(sigma_x*sigma_x_dot);
B=[sigma_x,0,cross_cov, 0
    0,    sigma_x, 0, cross_cov
    cross_cov, 0, sigma_x_dot, 0
    0,  cross_cov, 0, sigma_x_dot];
new_cov=A*state_cov*A.'+B;
end