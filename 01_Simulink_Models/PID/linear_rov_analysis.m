% Linear system control - velocity
clc;
clear all
close all
load('rovmodel75kg.mat')

% Equilibrium point (about the vertical plane at a particular yaw angle)
uo=0.001;
wo=0.05;
psio=90*pi/180;
Xudot=Ma(1,1);
Zwdot=Ma(3,3);


% Preparing parameters for State-space matrices
ML=M-Ma
DL
CL12=[0 (m(1)-Zwdot)*wo 0;  -(m(1)-Zwdot)*wo 0 (m(1)-Xudot)*uo; 0 -(m(1)-Xudot)*uo 0];
CL=[zeros(3) CL12; -CL12' zeros(3)]
GL=[zeros(3)  zeros(3); zeros(3) [-zb*W 0 0; 0 -zb*W 0; 0 0 0]] 
J11=[cos(psio)  -sin(psio) 0; sin(psio) cos(psio) 0; 0 0 1];
J=[J11 zeros(3);zeros(3) eye(3)]

% Form SS - A, B, C, D matrices
AA=[-inv(ML)*(CL+DL)]   % 6x6
BB=[inv(ML)*T]         % 6x4
CC=eye(6)               % 6x6
DD=zeros(6,6)           % 6x4
ww=logspace(-3,3,100);

%sys matrix- ROV
sys_rov=ss(AA,BB,CC,DD);

%eigenstructure
eig_AA_rov=eig(AA)

%transmission zero
trans_zeros_rov=tzero(sys_rov)

%condition number
cond_num_rov=cond(AA)

% Controllability/ observability check
Rank_A_B_rov=rank(ctrb(AA,BB))
Rank_A_C_rov=rank(obsv(AA,CC))

