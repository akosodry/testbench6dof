clear all
close all
clc

load matlab.mat

dt = 2;
tsim = 20;

%% apply magnetic disturbance
measLength = tsim/Tsampling;
magwindow = round(dt/Tsampling);
measMagWindowRatio = measLength/magwindow;
startLocX = (randperm(floor(measMagWindowRatio/3.5))-1)*3+1;
startLocY = (randperm(floor(measMagWindowRatio/3.5))-1)*3+1;
startLocZ = (randperm(floor(measMagWindowRatio/3.5))-1)*3+1;

magDistTime = [1:measLength]'*Tsampling;
magDisturbance = zeros(measLength,3);

for i=1:length(startLocX)
    ramdomDisturbance = magnetic_dist_noise(magwindow) * (1+rand*4.5);
    
    magDisturbance(startLocX(i)*magwindow:(startLocX(i)+1)*magwindow-1,1) = ...
        ramdomDisturbance;
end

for i=1:length(startLocY)
    ramdomDisturbance = magnetic_dist_noise(magwindow) * (1+rand*4.5);
    
    magDisturbance(startLocY(i)*magwindow:(startLocY(i)+1)*magwindow-1,2) = ...
        ramdomDisturbance;
end
for i=1:length(startLocZ)
    ramdomDisturbance = magnetic_dist_noise(magwindow) * (1+rand*4.5);
    
    magDisturbance(startLocZ(i)*magwindow:(startLocZ(i)+1)*magwindow-1,3) = ...
        ramdomDisturbance;
end

open testbench_model_2019a.slx