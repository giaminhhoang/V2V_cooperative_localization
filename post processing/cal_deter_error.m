function deter_error=cal_deter_error(err_temp)
N=size(err_temp,1);
x_tilda=err_temp(:,1)-mean(err_temp(:,1));
y_tilda=err_temp(:,2)-mean(err_temp(:,2));
var_x=mean(x_tilda.^2);
var_y=mean(y_tilda.^2);
var_xy=mean(x_tilda.*y_tilda);
deter_error=var_x*var_y-var_xy^2;
end