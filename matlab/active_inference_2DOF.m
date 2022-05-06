%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Active inference for robot control
%
% Simple example of active inference control for a 2DOF robot arm. The free
% -energy is minimised in a dynamic case through beliefs and actions update
% The script makes use of generalised motions since the states are
% dynamically changing. This is using the u-AIC
% 
% Author: Corrado Pezzato, TU Delft
% Last modified: 10.02.2022
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
 
%% Set-up variables for simulation
% Select unbiased u-AIC (1) or normal AIC (0)
isUnbiased = 0;

t = 10;             % [s] Simulation time
h = 0.001;          % [s] Integration step
actionsTime = 0;    % [s] Action time, set to 0 for the trial beginning
t_collision = 2;    % [s] Collision time
coll_dur =3;
% Planar 2DOF robot parameters
a1 = 1;             % [m] Length link 1 
a2 = 1;             % [m] Length link 2 

%% Set-up variables for perception

% Initialize generative process (so the sensors' output)
q = zeros(2,t/h);   % [rad]
dq = zeros(2,t/h);  % [rad/s]
ddq = zeros(2,t/h); % [rad/s^2]

% Initial state of the robot
q(:,1) = [-pi/2, 0]';     

% Prior belief about the states of the robot arm, desired position
mu_d = [-0.2 0.3]';  

%% Tuning parameters

% Confidence in the prior belief about the states
P_mu0 = eye(2);
% Confidence in the prior belief about the derivative of the states
P_mu1 = eye(2);
% Confidence in the proprioceptive sensory data (position)
P_y0 = eye(2);
% Confidence in the proprioceptive sensory data (velocity)
P_y1 = eye(2);
 
% Learning rate for beliefs update
k_mu = 200;
% Learning rate for actions
k_a = 500;

% Integral gain
Ki = 0.5*eye(2);

% I_gain from the controller to be initialized to zero, for u-AIC
I_term = [0; 0];
% Saturation
max_I = 10*[100, 10]';
%% Initialize vectors

% Initialize actions vector
u = zeros(2,t/h);
% Initial control action
u(:,1) = [0 0]';

% Initialize the vector of beliefs abot the states and its derivatives
mu = zeros(2,t/h);     
mu_p = zeros(2,t/h);
mu_pp = zeros(2,t/h);   
 
% Initial guess about the states of the robot, initial belief
mu(:,1) = q(:,1);   % mu    position + random constant to appreciate the convergence
mu_p(:,1) = [0 0]';              % mu'   velocity
mu_pp(:,1) = [0 0]';             % mu''  acceleration

% Initialize vector for collecting the free-energy values
F = zeros(1,t/h-1);
 
% Initialize vectors for sensory input (these will be noisy)
y_q = zeros(2,t/h);   
y_dq = zeros(2,t/h);

%% Active Inference Loop

if isUnbiased == 1
    a = 2; % Index starts from 2 since we need values one step back
    % Initial state of the robot
    q(:,2) = q(:,1);   
    mu(:,2) = mu(:,1); 
    x = mu;
    x_p = mu_p;
else 
    a = 1;
end

for i=a:t/h
    
    %% Simulate noisy sensory input from encoders and tachometers
    z = random('norm', 0, 0.001, size(q,1), 1);  
    z_prime = random('norm', 0, 0.001, size(q,1), 1);
    y_q(:,i) = q(:,i) + z;
    y_dq(:,i) = dq(:,i) + z_prime; 
   
    if isUnbiased == 0
        %% Compute free-energy in generalised coordinates
        F(i) = 0.5*(y_q(:,i)-mu(:,i))'*P_y0*(y_q(:,i)-mu(:,i))+...              % Proprioceptive position for joint 1 and 2
             + 0.5*(y_dq(:,i)-mu_p(:,i))'*P_y1*(y_dq(:,i)-mu_p(:,i))+...        % Proprioceptive velocity for joint 1 and 2 
             + 0.5*(mu_p(:,i)+mu(:,i)-mu_d)'*P_mu0*(mu_p(:,i)+mu(:,i)-mu_d)+... % Model prediction errors mu_p      
             + 0.5*(mu_pp(:,i)+mu_p(:,i))'*P_mu1*(mu_pp(:,i)+mu_p(:,i));        % Model prediction errors mu_pp
     
        %% Belifs update and control with AIC
        mu_dot = mu_p(:,i) - k_mu*(-P_y0*(y_q(:,i)-mu(:,i)) + P_mu0*(mu_p(:,i)+mu(:,i)-mu_d));
        mu_dot_p = mu_pp(:,i) - k_mu*(-P_y1*(y_dq(:,i)-mu_p(:,i)) ...                              
                              +P_mu0*(mu_p(:,i)+mu(:,i)-mu_d) ...                                
                              +P_mu1*(mu_pp(:,i)+mu_p(:,i)));
        mu_dot_pp = - k_mu*(P_mu1)*(mu_pp(:,i)+mu_p(:,i));

        % State estimation
        mu(:,i+1) = mu(:,i) + h*mu_dot;             % Belief about the position
        mu_p(:,i+1) = mu_p(:,i) + h*mu_dot_p;       % Belief about motion of mu
        mu_pp(:,i+1) = mu_pp(:,i) + h*mu_dot_pp;    % Belief about motion of mu'
        
        %% Control actions
        u(:,i+1) = u(:,i)-h*k_a*(P_y1*(y_dq(:,i)-mu_p(:,i)) + P_y0*(y_q(:,i)-mu(:,i)));
        
        % Monitoring SPE
        SPEq(i) = (y_q(:,i)-mu(:,i))'*P_y0*(y_q(:,i)-mu(:,i));

    else
        % Unbiased AIC
        %% Belifs update and control with AIC
        mu_dot = mu_p(:,i) - k_mu*(-P_y0*(y_q(:,i)-mu(:,i))...
                                    + P_mu0*(mu(:,i)-mu(:,i - 1))); % careful with when you start the loop (start with 2 instead of 1)
    
        mu_dot_p = 0 - k_mu*(-P_y1*(y_dq(:,i)-mu_p(:,i)) ...                              
                                +P_mu0*(mu(:,i)-mu(:,i - 1)) ...                                
                                +P_mu1*(mu_p(:,i) - mu_p(:,i - 1)));
        
        mu(:,i+1) = mu(:,i) + h*mu_dot;             % Belief about the position
        mu_p(:,i+1) = mu_p(:,i) + h*mu_dot_p;       % Belief about motion of mu
        
        I_term = I_term + mu_d-mu(:,i);
        if I_term(1) > max_I(1)
            I_term(1) = max_I(1);           
        end
        if I_term(2) > max_I(2)
            I_term(2) = max_I(2);           
        end
        
        I(:,i)  = I_term;
                
        u(:,i+1) = 250*eye(2)*(mu_d-mu(:,i)) + [100, 0; 20, 0]*(0-mu_p(:,i)) + 0.1*Ki*(I_term);
        
        x_dot = x_p(:,i) - k_mu*(-P_y0*(q(:,i)-x(:,i)) ...
                           +P_mu0*(x(:,i)-x(:,i - 1))); % careful with when you start the loop (start with 2 instead of 1)
    
        x_dot_p = 0 - k_mu*(-P_y1*(dq(:,i)-x_p(:,i)) ...                              
                                +P_mu0*(x(:,i)-x(:,i - 1)) ...                                
                                +P_mu1*(x_p(:,i) - x_p(:,i - 1)));
        
        x(:,i+1) = x(:,i) + h*x_dot;             % Belief about the position
        x_p(:,i+1) = x_p(:,i) + h*x_dot_p;       % Belief about motion of x
        SPEq(i) = (y_q(:,i)-mu(:,i))'*P_y0*(y_q(:,i)-mu(:,i));
    end
    
    %% Update sensory input according to the actions taken
    if (i>t_collision/h)&&(i<t_collision/h+coll_dur/h)
        dq(:,i+1) = 0;
        q(:,i+1) = q(:,i);
    else
        ddq(:,i) = RealrobotDynamics(q(1,i),q(2,i),dq(1,i),dq(2,i),u(1,i),u(2,i));
        dq(:,i+1) = dq(:,i)+h*ddq(:,i);
        q(:,i+1) = q(:,i)+h*dq(:,i);
    end
    
end 

% Convergence mu
(y_q(1, 4000) - (mu_p(1,4000) - mu_d(1)))/2

%% Graphics

% Plot parameters
time_vec = 0:h:t;   
fontSize = 20;
linewidth = 2.5;

% Indixes for the markers, downsample 
ind = 400;

set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaulttextinterpreter','latex');

figure('DefaultAxesFontSize',fontSize,'Position', [10 10 640 1.5*480])
    subplot(3,1,1)
    plot(time_vec,mu(1,:),'linewidth',linewidth)
    ylim([q(1,1)-0.06 max(q(1,:))+0.12])
    hold on
    plot(time_vec, q(1,:),':','linewidth',linewidth)
    plot(time_vec, mu_d(1)*ones(size(time_vec)),'--k','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',linewidth)
    if actionsTime > 0
        plot([actionsTime actionsTime], ylim,'-.k','linewidth',linewidth); 
        legend('$\mu_1$','$q_1$','$t_a$','$\mu_{d_1}$','Location','best')
    else
        legend('$\mu_1$','$q_1$','$\mu_{d_1}$','Location','best')
    end
    grid on
    grid minor
    ylim([-1.58, 0.2])
    title('\textbf{State estimation for $q_1$}')
    %xlabel ('Time $[s]$');
    ylabel ('$\mu_1\ [rad]$');

%     subplot(3,1,2)
%     plot(time_vec, mu(2,:),'linewidth',linewidth)
%     ylim([q(2,1)-0.06 max(q(2,:))+0.06])
%     hold on
%     plot(time_vec, q(2,:),':','linewidth',linewidth)   
%     plot(time_vec, mu_d(2)*ones(size(time_vec)),'--k','MarkerIndices',1:ind:length(mu),'MarkerSize',9,'linewidth',linewidth)    
% 
%     if actionsTime > 0
%         plot([actionsTime actionsTime], ylim,'-.k','linewidth',linewidth); 
%         legend('$\mu_1$','$q_1$','$t_a$','$\mu_{d_1}$','Location','best')
%     else
%         legend('$\mu_1$','$q_1$','$\mu_{d_1}$','Location','best')
%     end
%     
%     grid on
%     grid minor
%     title('\textbf{State estimation for $q_2$}')
%     xlabel ('Time $[s]$');
%     ylabel ('$\mu_2\ [rad]$');
%     
    subplot(3,1,2)
    plot(time_vec,u(1,:),'linewidth',linewidth)
    ylim([min(min(u)) max(max(u))+10])
    hold on
    plot(time_vec,u(2,:),':','linewidth',linewidth)
    if actionsTime > 0
        plot([actionsTime actionsTime], ylim,'-.k','linewidth',linewidth); 
        legend('$u_1$','$u_2$','$t_a$','$\mu_{d_1}$','Location','best')
    else
        legend('$u_1$','$u_2$','Location','best')
    end
    grid on
    grid minor
    ylim([0, 350])
    title('\textbf{Control actions}')
    %xlabel ('Time $[s]$');
    ylabel ('$ u\ [Nm]$');
    
    subplot(3,1,3)
    plot(time_vec(1:end-1),SPEq,'linewidth',linewidth)
    hold on
    %ylim([min(I) max(I)+0.06])
    legend('${\varepsilon}_{y}^\top \Sigma_{y}^{-1}{\varepsilon}_{y}$','Location','best')
    grid on
    grid minor
    ylim([0, 0.3])
    title('\textbf{Sensory prediction error, joint 1}')
    xlabel ('Time $[s]$');
    ylabel ('Amplitude $[-]$');

