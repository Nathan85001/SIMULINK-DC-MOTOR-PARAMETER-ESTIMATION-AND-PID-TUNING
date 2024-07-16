%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  SCRIPT : Import DC motor log file to MATLAB workspace              %%
%%  BY     : Waleed El-Badry                                           %%
%%  DATE   : 13/08/2021                                                %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read log csv file
file_path = '../log/log.csv';
motor_log = readtable(file_path);

%% Store each column in a variable
t = motor_log.time;
speed = motor_log.SPEED;
pwm = motor_log.PWM;

%% export to mat file
save('motor_log','t','pwm','speed');