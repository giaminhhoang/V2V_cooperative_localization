function initialize_pf_CMM(Np,Nsv,N,common_error);
global pf;
for k=1:Np
    pf(k).common=common_error+0.1*randn(1,Nsv);  %common error stored as row vector
    pf(k).weight=1/Np;
end
for k=1:Np
    pf(k).mu{1}=[16;4.5;248.25;0;0;0];
    pf(k).mu{2}=[473;-4.5;251.75;0;0;0];
    pf(k).mu{3}=[251.75;0;16;4.5;0;0];
    pf(k).mu{4}=[248.25;0;473;-4.5;0;0];
end
for k=1:Np
    for j=1:N
    pf(k).cov{j}=eye(6);
    end
end
end
