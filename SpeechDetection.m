%% Initialization
% HCI Homework 3 Question 2
% Louis Keith and Chris Hickman
% 4-7-22
clear;
clc;
close all;
%% First Example - helperExtractAuditoryFeatures(x,fs) doesn't work
% Load the pretrained speech detection network
load("commandNet.mat")
% Load a short speech signal where a person says "stop"
[x,fs] = audioread("stop_command.flac");
% Listen to the command
sound(x,fs)
% Extract auditory spectrum from the sound
% auditorySpect = helperExtractAuditoryFeatures(x,fs);
% % Classify the command based on the spectrogram
% command = classify(trainedNet,auditorySpect);

%% Second Example
x = audioread("play_command.flac");
sound(x,fs)
% auditorySpect = helperExtractAuditoryFeatures(x,fs);
% command = classify(trainedNet,auditorySpect);

%% Third Example
x = pinknoise(16e3);
auditorySpect = helperExtractAuditoryFeatures(x,fs);
command = classify(trainedNet,auditorySpect);

%% Experimentation
[y, Fs] = audioread('Up.mp3');
player = audioplayer(y, Fs);
play(player)
%%
[y, Fs] = audioread('Down.mp3');
player = audioplayer(y, Fs);
play(player)
%%
[y, Fs] = audioread('Left.mp3');
player = audioplayer(y, Fs);
play(player)
%%
[y, Fs] = audioread('Right.mp3');
player = audioplayer(y, Fs);
play(player)
