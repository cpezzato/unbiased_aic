% Simple scipt to generate the set of poses for the sttaistical analysis
close all
clear all

collision = 0; % Set 1 o collision or 0 for no coision data
n_samples = 100; % 100 reaching tasks

if collision == 1 
    for i=1:n_samples
        % Sample initial desired state at the start of the trial
        mu_d_seq(:,i) = 1.57*2*rand(2, 1)-1.57;  % sample in [-pi/2, pi/2]
        % Sample collision duration and collision time
        coll_dur_seq(i) = 2*rand(1, 1)+1; % sample in [1, 3]
        t_collision_seq(i) = 3*rand(1, 1); % sample in [0, 3]
    end
else 
    for i=1:n_samples
        % Sample initial desired state at the start of the trial
        mu_d_seq(:,i) = 1.57*2*rand(2, 1)-1.57;  % sample in [-pi/2, pi/2]
        % Sample collision duration and collision time
        coll_dur_seq(i) = 0; % no collsion duration
        t_collision_seq(i) = 0; % set to zero for consistency with previous data. Must bezero for corect settling time result
    end
end