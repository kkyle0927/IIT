clc; clear all; close all;

%%

fmin = 0.5;
fmax = 50;

f_resolution = 20;
f_repeat = 20;
ctrl_freq = 1000;

f_index = 1:f_resolution+1;

freqs = 10.^((f_index-1)*((log10(fmax) - log10(fmin))/(f_resolution)) + log10(fmin))
sys_id_signal_length = double(uint16((f_repeat*ctrl_freq./freqs)))
sys_id_signal_length_sum = double(sum(sys_id_signal_length))

fileID = fopen('freqs.txt','w');
fprintf(fileID, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n', freqs);
fprintf(fileID, '%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n', sys_id_signal_length);
fclose(fileID);


%% load data
t_data = load('sipal.txt');

cnt = t_data(:,1);
vel = t_data(:,2);
cur = t_data(:,3);


%% Spline Data

new_cnt = 1:192940;
splined_cnt = spline(cnt,cnt,new_cnt);
splined_vel = spline(cnt,vel,new_cnt);
splined_cur = spline(cnt,cur,new_cnt);


%% Divide frequency by frequency

cursor = 0;

for i = 1:length(sys_id_signal_length)

    cell_cur{i, :} = num2cell( splined_cur(cursor+1 : cursor+sys_id_signal_length(i) ));
    cell_vel{i, :} = num2cell( splined_vel(cursor+1 : cursor+sys_id_signal_length(i) ));

    cursor = cursor + sys_id_signal_length(i);

end


%% Bandpass filtering

band_range = 0.2;
n = 2;


for i = 1:length(sys_id_signal_length)

    d = designfilt('bandpassiir','FilterOrder', n, ...
               'HalfPowerFrequency1',(freqs(i)-freqs(i)*band_range),'HalfPowerFrequency2',(freqs(i)+freqs(i)*band_range), ...
                'DesignMethod', "butter", 'SampleRate',1000);

    cell_filter_cur{i, :} = filter(d, double(cell2mat(cell_cur{i, :})) );
    cell_filter_vel{i, :} = filter(d, double(cell2mat(cell_vel{i, :})) );

end



%% Remove the Transient Section

invalid_data_size = floor(double(sys_id_signal_length)/2);

for i = 1:length(sys_id_signal_length)
    cell_cutted_cur{i, :} = cell_filter_cur{i, 1}(invalid_data_size(i)+1:end);
    cell_cutted_vel{i, :} = cell_filter_vel{i, 1}(invalid_data_size(i)+1:end);

end


%% Current FFT 

fs = 1000;

figure;
hold on;

for i = 1:length(sys_id_signal_length)

    mat_cur = cell_cutted_cur{i, 1}(1:end);
    mat_vel = cell_cutted_vel{i, 1}(1:end);
    
    fft_cur = fft(mat_cur);
    fft_vel = fft(mat_vel);
    
    N = length(mat_cur);
    nyq_freq = ceil(N/2);
    
    fft_nyq_cur = (fft_cur(1:nyq_freq));
    fft_nyq_vel = (fft_vel(1:nyq_freq));
    
    plot(abs(fft_nyq_vel));
    hold on;

    index = 11;
    tf_vel = fft_nyq_vel(index)/fft_nyq_cur(index)

    % magnitude
    vel_mag(i) = 20*log10(abs(tf_vel));
    
    % phase
    vel_phase(i) = 180/pi*angle(tf_vel);

end



%% Bode Plotting

figure
semilogx(2*pi*freqs, vel_mag, 'b', 'LineWidth', 1.5);
hold on;
grid on;
subtitle('Magnitude','interpreter','latex');
ylabel('Magnitude (dB)','interpreter','latex');
set(gca, 'FontSize', 18);


%% Curve Fitting

type = fittype('abs( 1/((J*x*j + B)) )');
Initial = [1, 1];
F=fitoptions('method','NonlinearLeastSquares','Display','iter','MaxIter',2000,'MaxFunEvals',600,'StartPoint',Initial);
[X,R,O]=fit(2*pi*freqs', double((10.^(vel_mag/20))'), type, F);

s = tf('s');
sys_pos = 1/(abs(X.J)*s + abs(X.B))

save('Current2Position_sys.mat', "sys_pos");

figure
bodemag(sys_pos)
hold on
semilogx(2*pi*freqs, vel_mag, 'b', 'LineWidth', 1.5);
hold on;
grid on;
title('Magnitude','interpreter','latex');
ylabel('Magnitude (dB)','interpreter','latex');
set(gca, 'FontSize', 13);












