close, clear all

load("experiment_1202.mat")

figure(1)
plot(clipped_iq_one.clipped_iq.Time, clipped_iq_one.clipped_iq.Data, 'LineWidth', 2,'Color',[0 0 0]);
hold on
plot(clipped_iq_both.clipped_iq.Time, clipped_iq_both.clipped_iq.Data, 'LineWidth', 2,'Color',[1 0 0]);
hold on
title('8115 Iq sat');
xlabel('time','FontSize',15);
ylabel('iq','FontSize',15);
legend("cliped_one", "clipped_both")