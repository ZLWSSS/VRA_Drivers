close all
clc
clear all
%%
load('test1128_00.mat')

%%
%st_idx = 9900;
st_idx = 1;%-85000;
end_idx = st_idx + 10000; %length(wbc_lcm_data.lcm_timestamp);%-85000;
time_cmd = controller2robot_legs.timestamp(st_idx:end_idx,1);
time_data = robot2controller_legs.timestamp(st_idx:end_idx,1);
figure(1)

set(gca,'Color','none','FontSize',15)
hold on
plot(time_cmd(st_idx:end_idx), controller2robot_legs.q_des(st_idx:end_idx,2),'LineWidth', 3)
plot(time_data(st_idx:end_idx), robot2controller_legs.tauIq(st_idx:end_idx,2),'LineWidth', 3)
grid on
axis tight
xlabel('Time(s)')
ylabel('Pos(Nm)')
ylim([-30 30])
legend('$q_{cmd}$','$q_{data}$','FontSize',15)




