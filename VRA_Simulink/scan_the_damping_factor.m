clear, close all

LPF_const = 1.591549430918954e-04;
%% big one
phase_resistor_big = 0.03516;
phase_Ls_big = 0.0000865;
Ke = 38.475; %V_peak/Kprm
flux_linkage = 0.0101011;
damping_factor_big = 21;
inertia_big = 0.0012010759;
%%  8118 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
phase_resistor = 0.186706;
phase_Ls = 0.0001265;
EMF_Constant = 0.00708491;
NPP = 21;
damping_factor_8118 = 21.2;
inertia = 0.000095332;
%% 8110
phase_resistor_8110 = 0.071;
phase_Ls_8110 = 0.0000434;
EMF_Constant_8110 = 0.000692679;
damping_factor_8110 = 21.2;
inertia = 0.000095332;
%% 8115
phase_resistor_8115 = 0.1625;
phase_Ls_8115 = 0.000105;
EMF_Constant_8115 = 0.00147095;
damping_factor_8115 = 13;
inertia_8115 = 0.000095332;
%% Little Black one
phase_resistor_black = 0.1215;
phase_Ls_black = 0.000068;
flux_linkage_black = 0.00289658;
damping_factor_black = 21;
%% 10015
phase_resistor_10015 = 0.231629;
phase_Ls_10015 = 0.000205;
Ke = 34.504; %V_peak/Kprm
flux_linkage_10015 = 0.0090586;
damping_factor_10015 = 21.1;
inertia_10015 = 0.000037588913926;
%% white one
phase_resistor_4310 = 0.2262;
phase_Ls_4310 = 0.00003462;
flux_linkage_4310 = 0.002582;
damping_factor_4310 = 21.5;
%% 
%K_b = 643.947;
sampling_period_cur = 0.000025;
sampling_period = 0.000025;
%%delta_vector
delta_vector = [1.5, 2.5, 4, 6, 10, 20, 35, 70];
super_refine_delta_vector=[6,8,10,12,14,16,18,20];
refine_delta_vector = [6,8,10,15,20,25,30,35];
%% compute Vec
vector_use = super_refine_delta_vector;
phase_resitor_use = phase_resistor_8115;
phase_Ls_use = phase_Ls_8115;
EMF_Constant_Use = EMF_Constant_8115;
inertia_use = inertia_8115;
%%
choose_done = 1;
manual = 0;
K_b = phase_resitor_use/phase_Ls_use;
sys_cur_open_loop = tf([1],[phase_Ls_use phase_resitor_use]);
if(manual)
    K_c = 0.5;
    K_d = 13.980;
end
%% parameter choose
damp_factor_use = damping_factor_4310;
K_a = 0.4;
K = 3*NPP*EMF_Constant_Use/(2*inertia_use);
H=tf([1],[1]);
figure(1);

if(choose_done == 0)
for i = 1 : length(vector_use)
  K_c = 1/(K*LPF_const*vector_use(i));
  K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
  sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
  sys_dis = c2d(sys, sampling_period, 'zoh');
  margin(sys_dis);
  legend_str{i}=['Delta=',num2str(vector_use(i))];
  hold on;
end
legend(legend_str);

figure(2);
for i = 1 : length(vector_use)
    K_c = 1/(K*LPF_const*vector_use(i));
    K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
    sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
    T= feedback(sys,H);
    sys_dis = c2d(T, sampling_period, 'zoh');
    margin(sys_dis);
    hold on;
end
legend(legend_str);

figure(3)
for i = 1 : length(vector_use)
    K_c = 1/(K*LPF_const*vector_use(i));
    K_d = 1 / (vector_use(i)* vector_use(i)* LPF_const);
    sys=tf([K*K_c K*K_c*K_d],[LPF_const 1 0 0]);
    T= feedback(sys,H);
    sys_dis = c2d(T, sampling_period, 'zoh');
    stepplot(sys_dis);
    hold on;
end
legend(legend_str);
end

if(choose_done == 1)
    Ka_ub = pi*phase_Ls_use/(5*sampling_period_cur);
    Ka_lb = 10*phase_Ls_use/(damp_factor_use*LPF_const);
    figure(4);
    if(manual == 0)
      K_c = 1/(K*LPF_const*damp_factor_use);
      K_d = 1 / (damp_factor_use* damp_factor_use* LPF_const);
    end
    sys_vel=tf([K*K_c K*K_c*K_d],[(phase_Ls_use/K_a)*LPF_const (LPF_const+(phase_Ls_use/K_a)) 1 0 0]);
    sys_cur = tf([1],[(phase_Ls_use/K_a) 1]);
    bw_cur = bandwidth(sys_cur);
    sys_cur_cl_dis = c2d(sys_cur, sampling_period, 'zoh');
    sys_vel_cl = feedback(sys_vel, H);
    bw_vel = bandwidth(sys_vel_cl);
    sys_vel_cl_dis = c2d(sys_vel_cl, sampling_period, 'zoh');
    subplot(1,5,1);
    margin(sys_vel);
    title('Vel OpenLoop');
    subplot(1,5,2);
    margin(sys_cur_cl_dis);
    title('Cur discrete CloseLoop')
    subplot(1,5,3);
    margin(sys_vel_cl_dis);
    title('Vel CloseLoop')
    subplot(1,5,4);
    stepplot(sys_vel_cl_dis);
    title('Vel Step');
    subplot(1,5,5);
    stepplot(sys_cur_cl_dis);
    title('Cur Step');
    figure(4);
end






