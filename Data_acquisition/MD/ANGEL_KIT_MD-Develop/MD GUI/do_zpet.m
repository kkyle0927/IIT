function results = do_zpet()
%UNTITLED 이 함수의 요약 설명 위치
%   자세한 설명 위치
clc; clear all;

Ts = 0.001;
s = tf('s');
z = tf('z', Ts);

G = 1/(0.019022796303034*s^2 + 0.032552819699049*s);
C_d = 1 + 0.7*((1-z^-1)/Ts);
G_d = c2d(G, Ts, 'zoh');
Gclosed = minreal((G_d*C_d)/(1+G_d*C_d), 1e-4);

[num, den] = tfdata(Gclosed,'v');

if (length(num)>length(den))
    results = -1;
end

[Z,P,K] = tf2zpk(num,den);

j = 0;      k = 0;
Zu = [];    Zs = [];

for i=1:length(Z)
    if(abs(Z(i)) >= 0.999)
        j = j+1;
        Zu(j) = Z(i);
    else
        k = k+1;
        Zs(k) = Z(i);
    end
end

disp(strcat(num2str(j), ' unstable zero(s) detected.'));

F1 = zpk(P, Zs, K^-1, Ts);
F2temp = zpk(Zu, [], 1, Ts);
Gain = dcgain(F2temp);

if(Gain == inf)
    Gain = bode(F2temp, 2*pi);
    edit_commBoard(app, 'DC-gain is infinity. The magnitude is matched at 1Hz.');
end

[N,D] = tfdata(F2temp,'v')

for p = 1:(j+1)
    Nu(j-p+2) = N(p)
end
Du = zeros(1,(j+1));
Du(1) = 1;


F2 = tf(Nu, Du, Ts);
F = F1*F2/Gain^2;
Gzpet = minreal(F)
w = 0:1:500*2*pi;

[sys_mag, sys_phase] = bode(Gclosed, w);
[sys_inv_mag, sys_inv_phase] = bode(Gzpet, w);

sys_mag = 20*log10(squeeze(sys_mag));
sys_inv_mag = 20*log10(squeeze(sys_inv_mag));

figure
bode(Gclosed,w);
hold on;
bode(Gzpet,w);
bode(minreal(Gzpet*Gclosed),w);

legend('Gclosed', 'Zpet', 'Overall');



end

