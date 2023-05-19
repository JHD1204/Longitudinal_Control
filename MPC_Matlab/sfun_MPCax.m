% 基于运动学模型的MPC，求解控制量ax

function [sys,x0,str,ts,simStateCompliance] = sfun_MPCax(t,x,u,flag) %主函数

switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes; %初始化函数
  case 1
    sys=mdlDerivatives(t,x,u);  %更新连续状态
  case 2
    sys=mdlUpdate(t,x,u);  %更新离散状态
  case 3
    sys=mdlOutputs(t,x,u); %计算模块输出
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sample_t = 0.05;
sizes = simsizes;
sizes.NumContStates  = 0; %连续状态量个数
sizes.NumDiscStates  = 0; %离散状态量个数
sizes.NumOutputs     = 2; %输出量个数
sizes.NumInputs      = 2; %输入量个数
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];                 %状态量初始化
str = [];                 %matlab保留变量，默认为空，用不到
ts  = [sample_t 0];       %采样时间及其偏移
simStateCompliance = 'UnknownSimState';

%设置一些全局变量
global MPCParameters; 
MPCParameters.Np = 30;   %预测时域
MPCParameters.Nc = 30;   %控制时域
MPCParameters.Nx = 2;    %状态变量
MPCParameters.Nu = 1;    %控制输入
MPCParameters.Ny = 1;    %输出变量
MPCParameters.Ts = 0.05; %设置采样时间
MPCParameters.Q = 10;   %权重
MPCParameters.R = 1;     %权重
MPCParameters.S = 1;     %权重
MPCParameters.umin = -5.0;  % 最大减速
MPCParameters.umax = 3.5;   % 最大加速
MPCParameters.dumin = -1.0; % minimum limits of jerk
MPCParameters.dumax = 1.0;  % maximum limits of jerk
global WarmStart;
WarmStart = zeros(MPCParameters.Np,1);

end

function sys=mdlDerivatives(t,x,u) %连续状态更新
sys = [];
end

function sys=mdlUpdate(t,x,u)      %离散状态更新
sys = x;
end

function sys=mdlOutputs(t,x,u)     %输出函数

global MPCParameters;
global WarmStart;

tic; %开始计时

%Step(1).更新车辆纵向状态
Vx = u(1);
a_x = u(2);
kesi = [Vx; a_x]; %更新车辆状态向量

%Step(2).获取期望速度
% [SpeedProfile, MPCParameters.Np] = ReferceSpeedSlope(t, MPCParameters.Np);  %平稳曲线
[SpeedProfile, MPCParameters.Np] = FindReferceSpeed(t, MPCParameters.Np);   %正弦曲线

%Step(3).调用MPC优化求解函数得到最优控制量
Ts = MPCParameters.Ts;
StateSpaceModel.A = [1  Ts; 0   1];
StateSpaceModel.B = [0; 1];
StateSpaceModel.C = [1, 0];

[PHI, THETA] = func_Update_PHI_THETA(StateSpaceModel, MPCParameters);
[H, ~, g] = func_Update_H_f(kesi, SpeedProfile, PHI, THETA, MPCParameters);

% qp-solver:quadprog
[A,b,Aeq,beq,lb,ub] = func_Constraints_du_quadprog(MPCParameters,a_x);
options = optimset('Display','off', ...
    'TolFun', 1e-8, ...
    'MaxIter', 2000, ...
    'Algorithm', 'active-set', ...
    'FinDiffType', 'forward', ...
    'RelLineSrchBnd', [], ...
    'RelLineSrchBndDuration', 1, ...
    'TolConSQP', 1e-8);
warning off all  % close the warnings during computation

U0 = WarmStart;
[U, ~, EXITFLAG] = quadprog(H, g, A, b, Aeq, beq, lb, ub, U0, options); %
WarmStart = shiftHorizon(U);
if (1 ~= EXITFLAG) %if optimization NOT succeeded.
    U(1) = 0.0;
    fprintf('MPC solver not converged!\n');
end
delta_ax =  U(1);

prog_run_t = toc;  %computation time
sys = [delta_ax; prog_run_t];

end
    
function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
end

function sys=mdlTerminate(t,x,u)
sys = [];
end

%% sub functions
  
function [PHI, THETA] = func_Update_PHI_THETA(StateSpaceModel, MPCParameters)
%***************************************************************%
% 预测输出表达式 Y(t)=PHI*kesi(t)+THETA*DU(t) 
% Y(t) = [Eta(t+1|t) Eta(t+2|t) Eta(t+3|t) ... Eta(t+Np|t)]'
%***************************************************************%
Np = MPCParameters.Np;
Nc = MPCParameters.Nc;
Nx = MPCParameters.Nx;
Ny = MPCParameters.Ny;
Nu = MPCParameters.Nu;
A = StateSpaceModel.A;
B = StateSpaceModel.B;
C = StateSpaceModel.C;

PHI_cell = cell(Np,1);    % PHI=[CA CA^2 CA^3 ... CA^Np]T
THETA_cell = cell(Np,Nc);
for j = 1:1:Np
    PHI_cell{j,1}=C*A^j;
    for k = 1:1:Nc
        if k<=j
            THETA_cell{j,k}=C*A^(j-k)*B;
        else
            THETA_cell{j,k} = zeros(Ny,Nu);
        end
    end
end
PHI=cell2mat(PHI_cell);    % size(PHI)=[(Ny*Np) * Nx]
THETA=cell2mat(THETA_cell);% size(THETA)=[Ny*Np Nu*Nc]
end

function[H, f, g] = func_Update_H_f(kesi, SpeedProfile, PHI, THETA, MPCParameters)

Np = MPCParameters.Np;
Nc = MPCParameters.Nc;
Q  = MPCParameters.Q;
R  = MPCParameters.R;

Qq = kron(eye(Np),Q);
Rr = kron(eye(Nc),R);

% Vref = cell2mat(SpeedProfile);
Vref = SpeedProfile';
error1 = PHI*kesi;

H = THETA'*Qq*THETA +Rr;
f = (error1'-Vref')*Qq*THETA;
g = f';
end

function [A, b, Aeq, beq, lb, ub] = func_Constraints_du_quadprog(MPCParameters, um)
%************************************************************************%
% generate the constraints of the vehicle
%************************************************************************%
Np = MPCParameters.Np;
Nc = Np;
dumax = MPCParameters.dumax;
umin = MPCParameters.umin;
umax = MPCParameters.umax;
Umin = kron(ones(Nc,1),umin);
Umax = kron(ones(Nc,1),umax);
Ut   = kron(ones(Nc,1),um);

%----(1) A*x<=b----------%
A_t=zeros(Nc,Nc);
for p=1:1:Nc
    for q=1:1:Nc
        if p >= q
            A_t(p,q)=1;
        else
            A_t(p,q)=0;
        end
    end
end
A_cell=cell(2,1);
A_cell{1,1} = A_t; %
A_cell{2,1} = -A_t;
A=cell2mat(A_cell);  %


b_cell=cell(2, 1);
b_cell{1,1} = Umax - Ut; %
b_cell{2,1} = -Umin + Ut;
b=cell2mat(b_cell);  %

%----(2) Aeq*x=beq----------%
Aeq = [];
beq = [];

%----(3) lb=<x<=ub----------%
lb=kron(ones(Nc,1),-dumax);
ub=kron(ones(Nc,1),dumax);
end

function u0 = shiftHorizon(u)      % shift control horizon
u0 = [u(:,2:size(u,2)), u(:,size(u,2))];  %  size(u,2))
end


end




