function [new_mean,new_cov]=Kalman_update(state_mean,state_cov,gps_pose,P)
C=[1 0 0 0;
   0 1 0 0];
K=state_cov*C.'*inv(C*state_cov*C.'+P);
new_mean=state_mean+K*(gps_pose-state_mean(1:2));
new_cov=(eye(4,4)-K*C)*state_cov;
end