%this program calls the three CMM methods and draws corresponding plot 
%depending on the flags:

%control parameters
fsim=0;   % fsim=1: run simulation to obtain localization error; else if fsim=0: load data
fplot=1;  % fplot=1: plot data
fsave=0;  % fsave=1: save data to '.mat' file

if fsim==1
    [er_rbpf,det_rbpf]=cmm_rbpf(0);
    [er_sm,det_sm]=cmm_smoothed_static(0);
    [er_sta,det_sta]=cmm_static(0);
    [er_rbpf_f7,det_rbpf_f7]=cmm_rbpf(1);
    [er_sm_f7,det_sm_f7]=cmm_smoothed_static(1);
    [er_rbpf_f8,det_rbpf_f8,com_er_rbpf_f8,com_std_rbpf_f8]=cmm_rbpf(0);
    [er_sm_f8,det_sm_f8,com_er_sm_f8,com_std_sm_f8]=cmm_smoothed_static(0);
end
if fsim==0
    load CMM.mat;
end
    
    if fplot==1
    figure
    hold on;
    plot(1:length(er_sta),er_sta,'b','LineWidth',1.5)
    plot(1:length(er_sm),er_sm,'r','LineWidth',1.5)
    plot(1:length(er_rbpf),er_rbpf,'g','LineWidth',1.5)
    legend('Static method','Smoothed static method','RBPF')
    ylabel('Localization error (m)')
    title('Fig. 5')
    
    figure
    hold on;
    semilogy(1:length(det_sta),det_sta,'b','LineWidth',1.5)
    semilogy(1:length(det_sm),det_sm,'r','LineWidth',1.5)
    semilogy(1:length(det_rbpf),det_rbpf,'g','LineWidth',1.5)
    legend('Static method','Smoothed static method','RBPF')
    ylabel('Determinant of covariance (m^2)')
    title('Fig. 6')

    figure
    hold on;
    plot(1:length(er_sm_f7),er_sm_f7,'r','LineWidth',1.5)
    plot(1:length(er_rbpf_f7),er_rbpf_f7,'g','LineWidth',1.5)
    legend('Smoothed static method','RBPF')
    ylabel('Localization error (m)')
    title('Fig. 7')

    figure
    errorbar(com_er_sm_f8,3*com_std_sm_f8,'LineWidth',1.5)
    legend('Smoothed static method')
    ylabel('Common bias estimation error (m)')
    title('Fig. 8 (left)')
    
    figure
    errorbar(com_er_rbpf_f8,3*com_std_rbpf_f8,'LineWidth',1.5)
    legend('RBPF')
    ylabel('Common bias estimation error (m)')
    title('Fig. 8 (right)')

    openfig('complexity_error.fig');
    openfig('histogram.fig');
    openfig('position_5sv.fig');
    openfig('position_ego_rohani_RBPF.fig');
    
    end
    
    if fsave==1
        save('CMM.mat','er_rbpf','det_rbpf','er_sm','det_sm','er_sta','det_sta', ...
        'er_rbpf_f7','det_rbpf_f7','er_sm_f7','det_sm_f7','er_rbpf_f8','det_rbpf_f8', ...
    'com_er_rbpf_f8','com_std_rbpf_f8','er_sm_f8','det_sm_f8','com_er_sm_f8','com_std_sm_f8');
    end

    


    
