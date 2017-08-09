function [mu,cov]=pf_to_gaussian_RBPF
global pf;
mu=0;
N=length(pf);
for k=1:N
    mu=mu+pf(k).weight*pf(k).mu{1}([1,3]);
end
cov=zeros(2,2);
for k=1:N
    cov=cov+pf(k).weight*(pf(k).cov{1}([1,3],[1,3])+(pf(k).mu{1}([1,3])-mu)*(pf(k).mu{1}([1,3])-mu)');
end
end