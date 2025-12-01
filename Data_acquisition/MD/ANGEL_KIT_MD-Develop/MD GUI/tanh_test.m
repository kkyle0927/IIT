t = 0:0.001:10-0.001;
amp = 2;
freq = 0.5;
settle_time = 1/freq/4;
rising_time = settle_time;
offset = 0;
lead_time = -5;

tx1 = zeros(1, length(t));

for i = 1:length(t)

    if((i/1000)+lead_time <= rising_time)
        tx1(i) = amp*tanh(pi*(4/rising_time)*(t(i) - rising_time/2 + lead_time))+amp;
    elseif(((i/1000)+lead_time > rising_time) && ((i/1000)+lead_time <= (rising_time + settle_time)))
        tx1(i) = 2*amp;
    else
        tx1(i) = -amp*tanh(pi*(4/rising_time)*(t(i) - rising_time - rising_time/2 - settle_time + lead_time))+amp;
    end

end

% tx2 = zeros(1, length(t));
% 
% for i = 1:length(t)
% 
%     if((i/1000) <= (1/(rising_time)/2))
%         tx2(i) = amp*tanh(pi*4*rising_time*(t(i)-0.01) - pi)+amp + offset;
%     else
%         tx2(i) = -amp*tanh(pi*(4*rising_time)*(t(i)-0.01) - 3*pi)+amp + offset;
%     end
% 
% end

figure
plot(t, tx1);
% hold on
% plot(t, tx2);
grid on