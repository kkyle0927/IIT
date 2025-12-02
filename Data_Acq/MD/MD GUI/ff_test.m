           
s = tf('s');
Ts = 0.001;

J = 0.0190227963;
B = 0.0325528197;

sys = 1/(J*s^2+B*s);
sys_d = minreal(c2d(sys, Ts, 'zoh'));

[num, den] = tfdata(sys_d,'v');

if (length(num)>length(den))
    f = msgbox("Error: Improper model","Error","error");
end

%%
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

F1 = zpk(P, Zs, K^-1, Ts);
F2temp = zpk(Zu, [], 1, Ts);
Gain = dcgain(F2temp);

if(Gain == inf)
    Gain = bode(F2temp, 2*pi);
    edit_commBoard(app, 'DC-gain is infinity. The magnitude is matched at 1Hz.');
end

[N,D] = tfdata(F2temp,'v');

for p = 1:(j+1),
    Nu(j-p+2) = N(p);
end
Du = zeros(1,(j+1));
Du(1) = 1;

F2 = tf(Nu, Du, Ts);
F = F1*F2/Gain^2;
Gzpet = minreal(F);


w = 0:1:500*2*pi;

figure
bode(sys_d,w)
hold on
bode(Gzpet,w)
bode(sys_d*Gzpet,w)