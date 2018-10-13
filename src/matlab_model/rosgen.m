% The purpose of this script is to Generate custom ROS messages

% ROSCORE SHOULD BE RUNNING BEFORE THIS SCRIPT IS RAN

clear;clear all;clc;

% Connect to ROS
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Generate custom ROS messages
fprintf('\nPlease verify the correct path up to but not including /au_everything, then press the ENTER key to continue\n')
ARVP_HOME = uigetdir('~','au_everything');
ARVP_path = strcat(ARVP_HOME,'/au_everything/catkin_ws/src/');
rosgenmsg(ARVP_path)