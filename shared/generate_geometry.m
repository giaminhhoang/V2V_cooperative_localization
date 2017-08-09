clear all
nodes=[50,0;
    50,100;
    0,50;
    100,50];   %x and y coordinates of nodes representing the roads, the length of nodes can be a very large number
Refine_nodes(1:500,1)=50;
Refine_nodes(1:500,2)=0.2:0.2:100;
Refine_nodes(501:1000,1)=0.2:0.2:100;
Refine_nodes(501:1000,2)=50;    %Refine the nodes on the roads for computing the distance of each points on the grid 
%map to the roads centers
grid_size=0.5;
Nx=100/grid_size+1;
Ny=100/grid_size+1;
x=(0:Nx-1)*grid_size;
y=(0:Ny-1)*grid_size;   %get the x,y coordinates of the grid mesh
N_nodes=size(Refine_nodes,1);  %get the total number of road nodes
for i=1:Nx
    for j=1:Ny
d_smallest=Distance([x(i),y(j)],Refine_nodes(1,:));
for k=2:N_nodes
    d_temp=Distance([x(i),y(j)],Refine_nodes(k,:));
    if d_temp<d_smallest
        d_smallest=d_temp;
    end  %end (if d_temp<d_smallest)
end  %end (for k=2:N_nodes)
distance(i,j)=d_smallest;   %calculated the distance of i,j grid points to road centers
    end %end (for j=1:Ny)
end %end (for i=1:Nx)

grad_dis=cell(Nx,Ny);
for i=2:Nx-1
    for j=2:Ny-1
        grad_dis{i,j}=[distance(i+1,j)-distance(i-1,j),distance(i,j+1)-distance(i,j-1)]/(2*grid_size);
    end
end
for i=1:Nx  %there is a small bug which does not matter
    grad_dis{i,1}=grad_dis{i,2};
    grad_dis{i,Ny}=grad_dis{i,Ny-1};
end
for j=1:Ny
    grad_dis{1,j}=grad_dis{2,j};
    grad_dis{Nx,j}=grad_dis{Nx-1,j};
end
% figure(1)
% hold
% for i=1:Nx
%     for j=1:Ny
%         if distance(i,j)>10
%             plot(x(i),y(j),'o')
%         else
%             plot(x(i),y(j),'*')
%         end
%     end
% end





