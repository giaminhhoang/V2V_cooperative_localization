function d=Distance_to_road(x,grid_size) %calculate the distance to road centers by interpolation from the pre-calculated values
 global dis;
 global grad;
nx=floor(x/grid_size);
size_d=size(dis);
if nx(1)<0|nx(2)<0|nx(1)>size_d(1)-1|nx(2)>size_d(2)-1   %the considered point locates outside the grid
    d=100;   %assign a large distance
else
X=nx(1)*grid_size;
Y=nx(2)*grid_size;
delta_x=x(1)-X;
delta_y=x(2)-Y;
d=dis(nx(1)+1,nx(2)+1)+sum(grad{nx(1)+1,nx(2)+1}.*[delta_x,delta_y]);
end
end
