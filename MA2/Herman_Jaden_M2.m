%% MAE 453 
% MATLAB Assignment 2
% Jaden Herman

clear;
clc;
close all;

%% (a) Load aircraft data
scriptFolder = fileparts(mfilename('fullpath'));
data = load(fullfile(scriptFolder, 'MA2.mat'));

ctrlCase = input('Input case 1-4:');

% Case 1
elev = 0;
ail = 0;
rud = 0;

if ctrlCase == 2
    elev = 5;
elseif ctrlCase == 3
    ail = 1;
elseif ctrlCase == 4
    rud = 1;
end

