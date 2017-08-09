function map_mp=mp_generation(Nsv,indicator,Ng,grid_size);
%grid_size=2;   %lanewise size of the map within which the mp error keeps constant 
max_fre=1;  %maximum spatial frequency for mp error
Nf=50;  % #of frequencies
fre_step=max_fre/Nf;
amp=sqrt(fre_step*9);
phase=2*pi*rand(Nf,Nsv);
%Ng=500/grid_size;
x=1:2*Ng;
map_mp=zeros(2*Ng,Nsv);
for k=1:2*Ng
   for j=1:Nsv
       if indicator(j)~=0
       for i=1:Nf
    map_mp(k,j)=map_mp(k,j)+amp*sin(i*fre_step*x(k)+phase(i,j));   
       end
       map_mp(k,j)=map_mp(k,j)+randn(1);
       end
   end
end
end
