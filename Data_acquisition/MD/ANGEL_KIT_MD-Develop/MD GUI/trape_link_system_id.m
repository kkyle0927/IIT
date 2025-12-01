%% 
clear all; close all; clf;

% data = load('230413_OLD_LEG_TRAPE_ID_5_45_60.txt');
data = load('230413_OLD_LEG_TRAPE_ID_DUAL_5_120_60.txt');

cnt     = data(:,1);
pos_ref = data(:,2);
pos_act = data(:,3);
vel_tar = data(:,4);
input   = data(:,5);
 
time = cnt*0.001;
len  = length(time);

input_filt = zeros(len, 1);
vel_act = zeros(len, 1);
for i = 2:len
    vel_raw = (pos_act(i) - pos_act(i-1))/0.001;

    vel_act(i) = 0.99*vel_act(i-1) + 0.01*vel_raw;
    input_filt(i) = 0.99*input_filt(i-1) + 0.01*input(i);
end

figure();
subplot(3,1,1);
hold on;
plot(time, pos_ref, 'r');
plot(time, pos_act, 'k');

subplot(3,1,2); hold on;
plot(input);
plot(input_filt);

subplot(3,1,3);
plot(vel_tar);

%% Remove DC-Delay Section 
ratio = 0.75;
amp = max(pos_ref);
j = 1;
for i = 1:len
    if (pos_act(i) < amp*ratio) && (-amp*ratio < pos_act(i))

        pos_act_cut(j)    = pos_act(i);
        input_filt_cut(j) = input_filt(i);
        vel_tar_cut(j)    = vel_tar(i);
        vel_act_cut(j)    = vel_act(i);
        power(j)          = input_filt(i) * vel_act(i);

        j = j+1;
    end
end
figure();
subplot(4,1,1);
plot(pos_act_cut, 'k');
title('Actual Position');

subplot(4,1,2); 
plot(vel_act_cut, 'k');
title('Actual Velocity');

subplot(4,1,3);
plot(input_filt_cut, 'k');
title('Control Input');

subplot(4,1,4);
plot(power, 'k');
title('Power');

%% Positive & Negative Decomposition
p = 1; n = 1;
for i = 1:length(power)

    if (power(i) > 0)

        pos_cnt(p) = i;
        pos_pos(p) = pos_act_cut(i);
        pos_vel(p) = vel_act_cut(i);
        pos_input(p) = input_filt_cut(i);
        p = p + 1;
    else
        neg_cnt(n) = i;
        neg_pos(n) = pos_act_cut(i);
        neg_vel(n) = vel_act_cut(i);
        neg_input(n) = input_filt_cut(i);
        n = n + 1;
    end
end

figure();
subplot(3,1,1); hold on;
plot(pos_cnt, pos_pos, 'r.'); plot(neg_cnt, neg_pos, 'b.'); 
hold off; grid on;

subplot(3,1,2); hold on;
plot(pos_cnt, pos_vel, 'r.'); plot(neg_cnt, neg_vel, 'b.'); 
hold off; grid on;

subplot(3,1,3); hold on;
plot(pos_cnt, pos_input, 'r.'); plot(neg_cnt, neg_input, 'b.'); 
hold off; grid on;

%% Decomposition by velocity
t_vel = -1; 
N = 0;
j = 1;
for i = 1:length(power)

    if (vel_tar_cut(i) ~= t_vel)
        N = N+1;
        velocity_buffer(N) = vel_tar_cut(i);
       
        t_vel = vel_tar_cut(i);
        j = 1;
    end

    pos_act_proc{N}(j) = pos_act_cut(i);
    vel_act_proc{N}(j) = vel_act_cut(i);
    input_filt_cut_proc{N}(j) = input_filt_cut(i);
    power_proc{N}(j) = power(i);
    j = j +1;
end

%% 
N = 1;
figure();
plot(sin(pos_act_proc{N}), input_filt_cut_proc{N});