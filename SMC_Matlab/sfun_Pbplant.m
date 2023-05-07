function [sys,x0,str,ts] = sfun_Pbplant(t,x,u,flag)
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
  case 1
    sys=mdlDerivatives(t,x,u);  
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
sizes.NumContStates  = 2;
%����״̬����ȱʡΪ0
sizes.NumDiscStates  = 0;
%��ɢ״̬����ȱʡΪ0
sizes.NumOutputs     = 3;
%���������ȱʡΪ0
sizes.NumInputs      = 1;
%���������ȱʡΪ0
sizes.DirFeedthrough = 0;
%�Ƿ����ֱ��ͨ����1���ڣ�0������
sizes.NumSampleTimes = 0;
%����ʱ�������������һ��
sys = simsizes(sizes);
%����size���ݽṹ����������Ϣ
x0  = [0,0];
%���ó�ʼ״̬
str = [];
%���������ÿ�
ts  = [];
%���ò���ʱ��

function sys=mdlDerivatives(t,x,u)
%���㵼���ص��Ӻ���
%����t,x,u��������״̬�ĵ����������ڴ˸���ϵͳ������״̬����
%���Ӻ������Բ�����
kb=1;
taob=1;
sys(1)=x(1);
sys(2)=(kb*u-x(1))/taob;

function sys=mdlOutputs(t,x,u)
%��������ص�����
%����t,x,u��������������ڴ�����ϵͳ���������
%���Ӻ����������
sys(1)=x(1);
sys(2)=x(2);
sys(3)=dt;

