function [vx_ref, Np_out] = FindReferceSpeed(t, Np)
%UNTITLED5 此处提供此函数的摘要
%   此处提供详细说明

x = 0:0.05:70;
vx = 6*sin(0.3*x + 7.5*pi) + 10;
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