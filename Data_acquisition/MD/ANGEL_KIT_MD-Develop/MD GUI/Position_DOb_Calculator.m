function results = Position_DOb_Calculator(J, B, wn)

T = 0.001;
s = tf('s');
z = tf('z', T);

sys = 1/(J*s^2+B*s);
Q_filter = (wn/(s+wn))^2;

sys_d = minreal(c2d(sys, T, 'zoh'))
uin2uout_d = minreal(c2d(Q_filter, T, 'matched'))

F = sys_d^-1*uin2uout_d;
% p = real(pole(F));
% z = zero(F);
% 
% 
% %----REMOVE NEGATIVE UNSTABLE ZEROS of G----%
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
% 
% sys_d = minreal(sys_d);
% 
% 
% %----REMOVE NEGATIVE ZEROS of (G^-1*Q)----%
% y2uhat_d = minreal(sys_d^-1*uin2uout_d);
% [Z_,gain_] = zero(y2uhat_d)
% y2uhat_d = zpk(Z_, real(pole(y2uhat_d)), gain_, T);
% y2uhat_d = minreal(y2uhat_d, 0.01);
% t_z = zero(y2uhat_d);
% 
% for i = 1:length(t_z)
%     if(((t_z(i)) <= 0))
%         y2uhat_d = (y2uhat_d/(z-t_z(i)))*(1-t_z(i));
%     end
% end
% 
% y2uhat_d = minreal(y2uhat_d);
y2uhat_d = minreal(F, 0.5);
%%
[GQ_num, GQ_den] = tfdata(y2uhat_d, 'v')

[Q_num, Q_den] = tfdata(uin2uout_d, 'v')

results = [length(GQ_num), length(GQ_den), length(Q_num), length(Q_den), GQ_num, GQ_den, Q_num, Q_den];

end

