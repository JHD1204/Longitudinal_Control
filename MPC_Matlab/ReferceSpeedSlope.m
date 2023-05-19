function [vx_ref, Np_out] = ReferceSpeedSlope(t, Np)
%UNTITLED5 此处提供此函数的摘要
%   此处提供详细说明

t1 = 8;
t2 = 6;
k1 = 6;
k2 = 4;
T1 = 60;
k3 = 7.5;
k4 = 5;

x1 = 0:0.05:t1;
N1 = length(x1);
x2 = t1+0.05:0.05:pi*k1+t1;
N2 = length(x2);
x3 = pi*k1+t1:0.05:T1-pi*k2-t2;
N3 = length(x3);
x4 = T1-pi*k2-t2:0.05:T1-t2;
N4 = length(x4);
x5 = T1-t2+0.05:0.05:70;
N5 = length(x5);

N6 = N1+N2+N3+N4+N5;

vx = zeros(1,70/0.05+1);
x = 0:0.05:70;
for i=1:N6

    if x(i) <= t1
        vx(1:N1) = 5*ones(1,length(x1));
    elseif x(i) <= pi * k1 + t1
        vx(N1+1:N1+N2) = sin((x2-pi/2*k1-t1) * 1/k1) * k3 + 5+k3;
    elseif x(i) <= T1-pi*k2-t2
        vx(N1+N2+1:N1+N2+N3) = (5 + 2*k3)*ones(1,length(x3));
    elseif x(i) <= T1-t2
        vx(N1+N2+N3+1:N1+N2+N3+N4) = cos(1/k2*(x4 + pi*k2 -T1 +t2)) * k4 + 5+2*k3-k4;
    else
        vx(N1+N2+N3+N4+1:N1+N2+N3+N4+N5) = (5+2*k3-2*k4)*ones(1,length(x5));
    end

end


T = 0.05;

if (t+Np*T) <= 70
    Np_out = Np;
    index1 = t/T;
    index2 = (t+Np*T)/T;
else
    Np_out = (60-t)/T;
    index1 = t/T;
    index2 = 60/T;
end

index1 = round(index1);
index2 = round(index2);
vx_ref = vx(index1+1:index2);

end