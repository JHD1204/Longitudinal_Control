function [sys,x0,str,ts] = sfun_Tectrl(t,x,u,flag)
%主函数
%主函数包含四个输出：
%                 sys数组包含某个子函数返回的值
%                 x0为所有状态的初始化向量
%                 str是保留参数，总是一个空矩阵
%                 Ts返回系统采样时间
%函数的四个输入分别为采样时间t、状态x、输入u和仿真流程控制标志变量flag
%输入参数后面还可以接续一系列的附带参数simStateCompliance
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 3
    sys=mdlOutputs(t,x,u);
  case {2,4,9}
    sys=[];
  otherwise
    error(['Unhandled flag=', num2str(flag)]);
 
end
%主函数结束
%下面是各个子函数，即各个回调过程
function [sys,x0,str,ts]=mdlInitializeSizes
%初始化回调子函数
%提供状态、输入、输出、采样时间数目和初始状态的值
%初始化阶段，标志变量flag首先被置为0，S-function首次被调用时
%该子函数首先被调用，且为S-function模块提供下面信息
%该子函数必须存在
sizes = simsizes;
%生成sizes数据结构，信息被包含在其中
sizes.NumContStates  = 0;
%连续状态数，缺省为0
sizes.NumDiscStates  = 0;
%离散状态数，缺省为0
sizes.NumOutputs     = 5;
%输出个数，缺省为0
sizes.NumInputs      = 8;
%输入个数，缺省为0
sizes.DirFeedthrough = 1;
%是否存在直馈通道，1存在，0不存在
sizes.NumSampleTimes = 1;
%采样时间个数，至少是一个
sys = simsizes(sizes);
%返回size数据结构所包含的信息
x0  = [];
%设置初始状态
str = [];
%保留变量置空
ts  = [0.02];
%设置采样时间

function sys=mdlOutputs(t,x,u)
%计算输出回调函数
%给定t,x,u计算输出，可以在此描述系统的输出方程
%该子函数必须存在

Je=0.16;    % 发动机转动惯量
Jwf=0.9;    % 前后轮转动惯量
Jwr=0.9;
m=1412;
r=0.33;
g=9.8;
f=0.0038;
nd1=1;
Cd=0.3;
rou=1.206;
A=2.2;
cx = Cd*rou*A/2;

v=u(1);
dv=u(2);
vd=u(3);
dvd=u(4);
ei=u(5);
k11=u(6);
ratio=u(7);
i=u(8);
Rg=1/(4.1*ratio);
% switch gear
%   case 1
%     Rg=1/(4.1*3.538);
%   case 2
%     Rg=1/(4.1*2.06);
%   case 3
%     Rg=1/(4.1*1.404);
%   case 4
%     Rg=1/(4.1*1.0);
%   case 5
%     Rg=1/(4.1*0.713);
%   case 6
%     Rg=1/(4.1*0.582);
%   otherwise
%     Rg=1/(4.1*(-3.168));
% end

J=(Je+Rg*Rg*(Jwf+Jwr+m*r*r))/Rg/r;

e=v-vd;
ut1=J*(dvd-nd1*e)+Rg*r*(m*g*f+cx*v^2+m*g*i);
%s11=v-vd+nd1*ei;
s11=v-vd;
ds11=dv-dvd+nd1*e;
ut2=-k11*sign(s11);
%ut2=-k11*sat(s11);
ut=ut1+ut2;
if ut<-67.65
    ut=-67.65;
end
if ut>252.35
    ut=252.35;
end

sys(1)=ut;
sys(2)=s11;
sys(3)=ds11;
sys(4)=ut1;
sys(5)=ut2;

function m=sat(s)
k=0.05;
d=abs(s/k);
if d<=1
    m=s/k;
else
    m=sign(s/k);
end


