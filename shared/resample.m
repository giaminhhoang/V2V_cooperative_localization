function [nS] = resample(St)  
St(:,3)=St(:,3)/sum(St(:,3));  % first normalize the weight
N=length(St(:,1));
nS=[];
m=0;
c(1)=St(1,3);
for i=2:N
    c(i)=c(i-1)+St(i,3);
end

u(1)=1/N*rand(1);
i=1;
for j=1:N
    while(u(j)>c(i))
       
        i=i+1;
         end
                nS(m+1,1:2)=St(i,1:2);
        nS(m+1,3)=1/N;
        m=m+1;
        u(j+1)=u(j)+1/N;
end
end
