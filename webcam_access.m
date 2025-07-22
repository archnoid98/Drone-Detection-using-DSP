% Clear workspace
clear; close all; clc;

% Turn on webcam and show live feed
cam = webcam;       % Connect to default webcam
preview(cam);       % Show live preview window

% To stop: 
% closePreview(cam);  % Close preview window
% clear cam;          % Release webcam