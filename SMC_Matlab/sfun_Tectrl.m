function [sys,x0,str,ts] = sfun_Tectrl(t,x,u,flag)
%������
%�����������ĸ������
%                 sys�������ĳ���Ӻ������ص�ֵ
%                 x0Ϊ����״̬�ĳ�ʼ������
%                 str�Ǳ�������������һ���վ���
%                 Ts����ϵͳ����ʱ��
%�������ĸ�����ֱ�Ϊ����ʱ��t��״̬x������u�ͷ������̿��Ʊ�־����flag
%����������滹���Խ���һϵ�еĸ�������simStateCompliance
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
%����������
%�����Ǹ����Ӻ������������ص�����
function [sys,x0,str,ts]=mdlInitializeSizes
%��ʼ���ص��Ӻ���
%�ṩ״̬�����롢���������ʱ����Ŀ�ͳ�ʼ״̬��ֵ
%��ʼ���׶Σ���־����flag���ȱ���Ϊ0��S-function�״α�����ʱ
%���Ӻ������ȱ����ã���ΪS-functionģ���ṩ������Ϣ
%���Ӻ����������
sizes = simsizes;
%����sizes���ݽṹ����Ϣ������������
sizes.NumContStates  = 0;
%����״̬����ȱʡΪ0
sizes.NumDiscStates  = 0;
%��ɢ״̬����ȱʡΪ0
sizes.NumOutputs     = 5;
%���������ȱʡΪ0
sizes.NumInputs      = 8;
%���������ȱʡΪ0
sizes.DirFeedthrough = 1;
%�Ƿ����ֱ��ͨ����1���ڣ�0������
sizes.NumSampleTimes = 1;
%����ʱ�������������һ��
sys = simsizes(sizes);
%����size���ݽṹ����������Ϣ
x0  = [];
%���ó�ʼ״̬
str = [];
%���������ÿ�
ts  = [0.02];
%���ò���ʱ��

function sys=mdlOutputs(t,x,u)
%��������ص�����
%����t,x,u��������������ڴ�����ϵͳ���������
%���Ӻ����������

Je=0.16;    % ������ת������
Jwf=0.9;    % ǰ����ת������
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


