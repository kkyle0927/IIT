function results = get_current_controll_frequency_response(input_file_ID, BW)

data = load(input_file_ID);

ref = data(1:end-2,2);
act = data(3:end  ,1);

L = length(ref);
Fs = 25000;
f = Fs*(0:(L/2))/L;

Y_ref           = fft(ref);
P2_ref          = Y_ref/L;
%P1_ref          = P2_ref(1:L/2+1);
P1_ref = P2_ref(1:floor(L/2)+1);
P1_ref(2:end-1) = 2*P1_ref(2:end-1);

Y_act           = fft(act);
P2_act          = Y_act/L;
%P1_act          = P2_act(1:L/2+1);
P1_act = P2_act(1:floor(L/2)+1);
P1_act(2:end-1) = 2*P1_act(2:end-1);

Mag = 20*log10(abs(P1_act./P1_ref));
Phase = (180/pi)*angle(P1_act./P1_ref);

fc = BW;
fs = 25000;
[b, a] = butter(2, fc/(fs/2));
Mag_filt = filtfilt(b,a, Mag);
Phase_fit = filtfilt(b,a, Phase);

writematrix([f', Mag_filt, Phase_fit], 'ELEC_CurrCtrl_BWTest_FrequencyDomain.txt');

results = 0;

end