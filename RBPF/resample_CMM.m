function resample_CMM
global pf;
pf_copy=pf;
% do some stuff here
size_samples=length(pf);
S=0;
for k=1:size_samples
    S=S+pf(k).weight;
end
for k=1:size_samples
    pf(k).weight=pf(k).weight/S;
end

c(1)=pf(1).weight;
for i=2:size_samples
    c(i)=c(i-1)+pf(i).weight;
end
i=1;
u(1)=1/size_samples*rand(1);
for j=1:size_samples
    while u(j) > c(i)
        i=i+1;
    end
    pf(j)=pf_copy(i);
    u(j+1)=u(j)+1/size_samples;
end
for k=1:size_samples
    pf(k).weight=1/size_samples;
end
end