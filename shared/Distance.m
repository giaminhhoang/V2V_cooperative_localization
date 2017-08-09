function d=Distance(x1,x2);   %calculate the distance between two points, macshen
dimen=length(x1);
d2=0;
for k=1:dimen
    d2=d2+(x1(k)-x2(k))^2;
end
d=sqrt(d2);
end