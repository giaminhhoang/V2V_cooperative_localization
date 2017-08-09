function [mu,cov]=pf_to_Gaussian(pf);
N=size(pf,1);
mu=[mean(pf(:,1)),mean(pf(:,2))];
X_tilda=[pf(:,1)-mu(1)];
Y_tilda=[pf(:,2)-mu(2)];
sigma_x=1/(N-1)*sum(X_tilda.^2);
sigma_y=1/(N-1)*sum(X_tilda.^2);
sigma_xy=1/(N-1)*sum(X_tilda.*Y_tilda);
cov=[sigma_x,sigma_xy;
    sigma_xy,sigma_y];
end