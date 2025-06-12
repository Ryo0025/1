clc
clear all
close all


l1 = 3;
l2 = 2;
l3 = 0;
l4 = 1;
l5 = 1;
l7 = 2; % l7即钻臂长度

err = 10;

%Link(theta, d, a, alpha)
L(1) = Link([0,0,0,0],'modified');L(1).qlim = [-7*pi/36,7*pi/36];
L(2) = Link([0,0,0,-pi/2],'modified');L(2).qlim = [l1,l1 + l2]; L(2).jointtype = 'P';
L(3) = Link([-pi/2,0,0,pi/2],'modified');L(3).qlim = [pi/2-7*pi/36,pi/2+7*pi/36];L(3).offset = 0;
L(4) = Link([pi/2,0,l3,pi/2],'modified');L(4).qlim = [pi/2,pi/2+pi/30];
L(5) = Link([0,l4,0,pi/2],'modified');
L(6) = Link([0,l5,0,-pi/2],'modified');L(6).qlim = [-15*pi/180,15*pi/180];

%L7为钻臂
L(7) = Link([0,0,0,pi/2],'modified');L(7).qlim = [l7,l7];L(7).offset = l7;L(7).jointtype ='P';
%L(8) = Link([0,0,0,0],'modified');L(8).qlim = [pi/2,pi/2];L(8).offset = pi/2;
%L(9) = Link([0,0,0,pi/2],'modified');L(9).qlim = [l,l];L(9).offset = l;L(9).jointtype = 'P';

Six_link = SerialLink([L(1),L(2),L(3),L(4),L(5),L(6),L(7)]);
Six_link.teach

%目标坐标,角度限制
%Target = transl(input("输入坐标x"),input("输入坐标y"),input("输入坐标z"));
Target = transl(3,5,1)* troty(-pi/2);

%机器人限位
% q_min = [-7*pi/36, l1, pi/2-7*pi/36, pi/2, -pi, -15*pi/180, l7];
% q_max = [7*pi/36, l1+l2, pi/2+7*pi/36, pi/2+pi/30, pi, 15*pi/180, l7];

%%随机生成初始角度
%for i = 1:30
%    q0 = unifrnd(q_min, q_max);
%    sol = Six_link.ikcon(Target,q0,'TolFun',1e-15,'MaxIterations', 10000);
    %sol = Six_link.ikine(Target,'q0',q0,'tol',1e-6,'ilimit',100,'pinv',true);

%    T_check = Six_link.fkine(sol);
%    error = norm(T_check.t - Target(1:3,4));
%    disp(['位置误差：', num2str(error)]);
%    if error < err 
%        err = error;
%        sol1 = sol;
%    end
%    if error < 1e-3
%        break
%    end
%end

%q0 = (q_min + q_max)/2
q0 = [pi,pi,pi,pi,pi,pi,pi];

sol_ikine = Six_link.ikine(Target, 'q0', q0, 'tol', 1e-15, 'ilimit', 2000, 'pinv', true);
sol = Six_link.ikcon(Target, sol_ikine, 'Display', 'iter', 'TolFun', 1e-10, 'MaxIterations', 10000);
T_check = Six_link.fkine(sol);
error = norm(T_check.t - Target(1:3,4));
disp(['位置误差：', num2str(error)]);


%disp(['最终位置误差：', num2str(err)]);



Six_link.teach(sol)
sol

if error > 1
    disp(['该位置或不可达'])
end