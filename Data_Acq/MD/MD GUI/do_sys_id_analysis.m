function results = do_sys_id_analysis(t_filename)


filename = sprintf('%s.txt', t_filename);

%% load data
% t_data = load('mingu_2nd_try.txt');
t_data = load(filename);

cnt  = t_data(:,1);
freq = t_data(:,2);
cur  = t_data(:,3);
vel  = t_data(:,4);

len = length(t_data);

%% Spline Data

new_cnt = 1:len;
%splined_cnt  = spline(cnt,cnt,new_cnt);
splined_freq = freq;
%splined_freq = spline(cnt,freq,new_cnt);
splined_cur  = spline(cnt,cur,new_cnt);
splined_vel  = spline(cnt,vel,new_cnt);

%% Divide frequency by frequency

t_freq = 0; %dummy value
j = 0;
N = 1;

for i = 1:len
    
    if (splined_freq(i) ~= t_freq)
        t_freq = splined_freq(i);
        j = j+1;
        frequency_samples(j) = t_freq; 
        N = 1;
    end

    if(j > 0)
        cell_cur{j}(N) = splined_cur(i);
        cell_vel{j}(N) = splined_vel(i);
        N = N+1;
    end

end

N_samples = length(frequency_samples);


%% Bandpass filtering

band_range = 0.15;
n = 2;

for i = 1:N_samples

    d = designfilt('bandpassiir','FilterOrder', n, ...
               'HalfPowerFrequency1',(frequency_samples(i)-frequency_samples(i)*band_range),'HalfPowerFrequency2',(frequency_samples(i)+frequency_samples(i)*band_range), ...
                'DesignMethod', "butter", 'SampleRate',1000);

    cell_filter_cur{i} = filter(d, cell_cur{i});
    cell_filter_vel{i} = filter(d, cell_vel{i});

end



%% Remove the Transient Section


for i = 1:N_samples

    cell_cutted_cur{i} = cell_filter_cur{i}(fix(end/2):end);
    cell_cutted_vel{i} = cell_filter_vel{i}(fix(end/2):end);

end


%% Current FFT 

fs = 1000;

for i = 1:N_samples

    L = length(cell_cutted_cur{i});
  
    y_input = fft(cell_cutted_cur{i});
    P2_input = y_input/L;
    P1_input = P2_input(1:fix(L/2)+1);
    P1_input(2:end-1) = 2*P1_input(2:end-1);

    y_output = fft(cell_cutted_vel{i});
    P2_output = y_output/L;
    P1_output = P2_output(1:fix(L/2)+1);
    P1_output(2:end-1) = 2*P1_output(2:end-1);
    
    index = fix(frequency_samples(i)*L/1000 + 1);

    tf_val = P1_output(index)/P1_input(index);

    tf_mag(i) = 20*log10(abs(tf_val));
    tf_phase(i) = 180/pi*angle(tf_val);
end



%% Curve Fitting


type = fittype('abs( 1/((J*x*j + B)) )');
Initial = [0.01, 0.01];

% type = fittype('abs( 1/((J*(x*j)^2 + B*(x*j) + K)) )');
% Initial = [1, 1, 1];

F=fitoptions('method','NonlinearLeastSquares','MaxIter',2000,'MaxFunEvals',600,'StartPoint',Initial);
[X,R,O]=fit(2*pi*frequency_samples', double((10.^(tf_mag/20))'), type, F);

J = abs(X.J)
B = abs(X.B)
%K = abs(X.K) 

for i = 1:N_samples
    
    w = 2*pi*frequency_samples(i);

    nominal_model_mag(i) = 20*log10(1/sqrt((J*w)^2+B^2));
    nominal_model_phase(i) = 180/pi*angle(1/( J*w*sqrt(-1) + B ));

%     nominal_model_mag(i) = 20*log10(1/sqrt((J*w^2)^2+(B*w)^2+K^2));
%     nominal_model_phase(i) = 180/pi*angle(1/( J*w*sqrt(-1) + B ));
end

% G(z) = b/(z+a)

a = -exp(-B*0.001/J);
b = (1-exp(-B*0.001/J))/B;

s = tf('s');

inertia_sys = 1/(abs(X.J)*s + abs(X.B)/2);

wc1 = 300*2*pi;
irc_low_filter = (wc1/(s+wc1))^2;
irc_d = (c2d((inertia_sys^-1)*irc_low_filter*0.95, 0.001, 'zoh'));
[irc_b, irc_a] = tfdata(irc_d, 'v');


results = [abs(X.J), abs(X.B), a, b, irc_a(2), irc_a(3), irc_b(2), irc_b(3)]

datamatrix = [frequency_samples', tf_mag', tf_phase', nominal_model_mag', nominal_model_phase']; 

name = sprintf('%s(frequency_domain).txt', t_filename);
writematrix(datamatrix, name);

end



