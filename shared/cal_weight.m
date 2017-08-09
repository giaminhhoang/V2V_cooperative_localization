function weight=cal_weight(pf,estenu,P,sigma_map2,lane_width,vehicle_width,grid_size)  %called by main program to calculate weight for particle filter
point=pf(1,1:2)+estenu(1,1:2);  %position of the hypothetical particle position
sigma_w2=trace(P)+sigma_map2;
d=Distance_to_road(point,grid_size);
if d<=lane_width-vehicle_width/2
    weight=pf(1,3);  %weight does not change
else %outside the road
    weight=pf(1,3)*exp(-(d-lane_width+vehicle_width/2)^2/2/sigma_w2);  %Here it should be considered how to calculate the weight in a better way
end
end
