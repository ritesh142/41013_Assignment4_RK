%% Script for Testing Class called ObjectModels
% Summary of example objective

clear all
close all 
clc
%% Creating a 'Crate Object in the environment
% Description of first code block
base = transl(0,0,0) * trotz(0/4);

stair = ObjectModels('Stairs',base,'helicalStaircaseModel.ply');
stair.PlotAndColour();

% Check if the base location set
baseTrReturned = stair.GetCurrentPose();

if base == baseTrReturned
   disp('Test 1 PASSED'); 
else
   disp('Test 1 FAILED'); 
end


%% Setting/Changing the Pose of the Object

newBase = transl(0.6,0.6,0)*trotz(pi/4);

stair.SetPose(newBase);

% Check if the base location set
baseTrReturned = stair.GetCurrentPose();

if newBase  == baseTrReturned
   disp('Test 2 PASSED'); 
else
   disp('Test 2 FAILED'); 
end

% Visually Inspect if the location of the object changes in the plot
stair.UpdatePlot();

%% Getting the 'Rest' Poisition of the Object once location has been changed

% Check if the base location set
baseTrReturned = stair.GetRestPose();

if base  == baseTrReturned
   disp('Test 3 PASSED'); 
else
   disp('Test 3 FAILED'); 
end

%% Get Face and Vertice data

[face,vertice] = stair.GetFaceandVertice();

% visualisation of the vertice
hold on

% figure(2)
plot3(vertice(end-100:end-95,1),vertice(end-100:end-95,2),vertice(end-100:end-95,3),'g*');

axis([-0.2 0.2 -0.2 0.2 0 0.6])
grid on
grid minor
