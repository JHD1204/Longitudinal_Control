function [sys,x0,str,ts] = sfun_throttle(t,x,u,flag)
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
sizes.NumOutputs     = 3;
%���������ȱʡΪ0
sizes.NumInputs      = 4;
%���������ȱʡΪ0
sizes.DirFeedthrough = 1;
%�Ƿ����ֱ��ͨ����1���ڣ�0������
sizes.NumSampleTimes = 0;
%����ʱ�������������һ��
sys = simsizes(sizes);
%����size���ݽṹ����������Ϣ
x0  = [];
%���ó�ʼ״̬
str = [];
%���������ÿ�
ts  = [];
%���ò���ʱ��

function sys=mdlOutputs(t,x,u)
%��������ص�����
%����t,x,u��������������ڴ�����ϵͳ���������
%���Ӻ����������

taoe=0.5;

Te=u(1);
Ted=u(2);
dTed=u(3);
k12=u(4);
s12=Te-Ted;
%ds12=-k12*sign(s12)-0.1*s12;
ds12=-k12*sign(s12);

ut=taoe*(ds12+dTed)+Te;

sys(1)=ut;
sys(2)=s12;
sys(3)=ds12;

function m=sat(s)
k=100;
d=abs(s/k);
if d<=1
    m=s/k;
else
    m=sign(s/k);
end
