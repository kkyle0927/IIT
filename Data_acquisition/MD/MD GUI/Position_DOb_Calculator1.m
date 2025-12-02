function results = Position_DOb_Calculator1(J, B, wn)

T = 0.001;
s = tf('s');
z = tf('z', T);

sys = 1/(J*s^2+B*s);
Q_filter = (wn/(s+wn))^2;

sys_d      = minreal(c2d(sys, T, 'zoh'));
uin2uout_d = minreal(c2d(Q_filter, T, 'zoh'));

F = sys_d^-1*uin2uout_d;
poles = pole(F);
zeros = zero(F);
Comp = (1-zero(1))/(1-pole(1))*(z - poles(1))/(z - zeros(1));
y2uhat_d = minreal(F*Comp);


%%
simplify(y2uhat_d)
[GQ_num, GQ_den] = tfdata(y2uhat_d, 'v')

simplify(uin2uout_d)
[Q_num, Q_den] = tfdata(uin2uout_d, 'v')

results = [length(GQ_num), length(GQ_den), length(Q_num), length(Q_den), GQ_num, GQ_den, Q_num, Q_den];

end

