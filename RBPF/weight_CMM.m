function weight_CMM(vehicle_id)
global pf;
N_mc=100;  % #of sample for Monte Carlo integration
Np=length(pf);
for k=1:Np
    sample(:,1:2)=mvnrnd([pf(k).mu{vehicle_id}(1),pf(k).mu{vehicle_id}(3)],pf(k).cov{vehicle_id}([1,3],[1,3]),N_mc);
x=(sample(:,1)>247).*(sample(:,1)<253);
y=(sample(:,2)>247).*(sample(:,2)<253);
pf(k).weight=pf(k).weight*sum(x|y);
end
end
