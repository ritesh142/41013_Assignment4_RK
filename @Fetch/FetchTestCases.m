%% FetchTestCases
% This is a test script for testing features of the Fetch Function
clear all
close all
clc
%% Creating the Object - Fetch

robotBase = transl(0,0,0.8);
robotName = 'Fetch_01';
robot = Fetch(robotName,robotBase);
clear 'robotBase';
clear 'robotName';


%% Plotting Fetch at Rest State

robot.model.plot(robot.restState);


qInitial = robot.model.ikcon(transl(0.7,0.21,0.8)*trotx(pi));
% qPartTop = robot.model.ikcon(transl(0.25,0.3,1)*trotx(pi));
robot.model.animate(qInitial);



%% Testing Component (XYZ-RPY) based Trajectory Generation
% Description of second code block

startTr = transl(0.5,0.5,0.9);
goalTr = transl(-0.5,-0.5,0.9);

steps = 200;

[pos,rot]= robot.GenCompTraj(startTr,goalTr,steps);


%% RMRC Class Method Test
clc
startTr = transl(0.4,0.4,0.8)*trotx(pi);
goalTr = transl(0.4,-0.4,0.8)*trotx(pi);
time = 2;
controlFeq = 0.05;
qMatrix = robot.RMRC(startTr,goalTr,time,controlFeq,0);
robot.model.plot(qMatrix,'trail','r-');

%Visualising Results
% for i=1:size(qMatrix,1)
%     
%     robot.model.animate(qMatrix(i,:));
% end 
