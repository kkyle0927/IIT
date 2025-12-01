clc; clear all; close all;

%%
data = load('DOb_verification.txt');

t_t = 0:0.001:length(data(:,1))/1000 - 0.001;
pos = data(:,3) - data(1,3);
input = data(:,5);

%%
J = 0.0190227963;
B = 0.0325528197;
wn = 50*2*pi;

T = 0.001;
s = tf('s');
z = tf('z', T);

sys = 1/(J*s^2+B*s);
Q_filter = (wn/(s+wn))^2;

sys_d = minreal(c2d(sys, T, 'zoh'));
uin2uout_d = minreal(c2d(Q_filter, T, 'zoh'));

%%
syms s_ z_

[num, den] = tfdata(Q_filter, 'v');

q_temp = ((wn)/(((1-z^-1)/T)+wn))^2;
% q_temp_d = subs(q_temp, s_, (1-z_^-1)/T)

% backwar = 3391170250170917/(34359738368*((1000/z - 1000)^2 - 200*pi*(1000/z - 1000) + 3391170250170917/34359738368))
% t_z = zero(sys_d)
% 
% j = 0;
% for i = 1:length(t_z)
%     if(t_z(i) <= -0.999)
%         sys_d = sys_d/(z-t_z(i));
%         sys_d = sys_d*(1-t_z(i));
%         j = j+1;
%     end
% end

% sys_d = minreal(sys_d)


%%
y2uhat_d = minreal(sys_d^-1 * uin2uout_d)

t_z = zero(uin2uout_d)

uin2uout_d_removed = uin2uout_d;

j = 0;
for i = 1:length(t_z)
    if(t_z(i) <= 0)
        uin2uout_d_removed = uin2uout_d_removed/(z-t_z(i));
        uin2uout_d_removed = uin2uout_d_removed*(1-t_z(i));
        j = j+1;
    end
end

uin2uout_d_removed = minreal(uin2uout_d_removed);

%%
uin2uout_d
y2uhat_d = minreal(y2uhat_d);
disp(strcat(num2str(j), " unstable zeros are detected"));


%%

upd = lsim(y2uhat_d, [pos(1:end)], t_t);
u = lsim(uin2uout_d, [input(1:end)], t_t);


%%
figure
plot(upd);
hold on;
plot(u);
plot(upd-u);
legend('u+d', 'u', 'dhat')

%%
figure
bode(y2uhat_d)
hold on
bode(sys_d)
