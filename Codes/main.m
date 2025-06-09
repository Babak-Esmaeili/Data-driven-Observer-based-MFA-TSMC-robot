clc;
clear;
close all;

tic;
%% Time
ts = 0;
T = 0.001;
tf = 10;

t = ts:T:tf;

nSteps = length(t);

%% Controller parameters
% MFASMC
eta = 1;
rho = 1;
mu = 1;
lambda = 1;

qqq = 5000;

ep = 50;

GAMMA = diag([2,2]);

% Observer-based MFATSMC
K = diag([0.9,0.9]);
F = eye(2,2) - K;

mu_1 = 1;
mu_2 = 1;

epsilon = 10^-5;

% TSMC
pp = 3;
qq = 5;

lambda_1_1 = 50;
lambda_1_2 = 50;
lambda_1 = diag([lambda_1_1,lambda_1_2]);

lambda_2_1 = 60;
lambda_2_2 = 20;
lambda_2 = diag([lambda_2_1,lambda_2_2]);

lambda_s_1 = 0.01;
lambda_s_2 = 0.01;
lambda_s = diag([lambda_s_1,lambda_s_2]);

m = 0.1*diag([1,1]);

for ii = 1:2
    
    %% Initialization
    nInputs = 2;
    nOutputs = 2;
    
    I = eye(nInputs,nOutputs);
    
    q = [0 0]';
    qdot = [0 0]';
    
    u_MFA = zeros(nInputs,nSteps);
    u_SM = zeros(nInputs,nSteps);
    
    u_eq = zeros(nInputs,nSteps);
    u_sw = zeros(nInputs,nSteps);
    u = zeros(nInputs,nSteps);
    
    y = zeros(nOutputs,nSteps+1);
    y_hat = zeros(nOutputs,nSteps+1);
    
    y_d = zeros(nOutputs,nSteps+1);
%     y_d(:,1) = [1.5*cos((2*pi/4)*t(1))
%                 1.2*cos((2*pi/7)*t(1))];
%     y_d(:,2) = [1.5*cos((2*pi/4)*t(2))
%                 1.2*cos((2*pi/7)*t(2))];       
    
    delta_u = zeros(nInputs,nSteps);
    delta_y = zeros(nInputs,nSteps+1);
    
    e = zeros(nOutputs,nSteps);
    e_o = zeros(nOutputs,nSteps+1);
    
    s = zeros(nOutputs,nSteps);
    
    H = cell(1,nSteps);
    H{1} = [delta_y(:,1)' delta_u(:,1)']';
    
    PHI_hat_1 = cell(1,nSteps+1);
    PHI_hat_1{1} = 1*[ 1  0
                       0  1 ];
    
    PHI_hat_2 = cell(1,nSteps+1);
    PHI_hat_2{1} = 1*[ 1  0
                       0  1 ];
    
    d = zeros(nOutputs,nSteps);
    
    if(ii == 1)
        
        fprintf('\nSimulating FFDL-MFASMC on two-link robot manipulator...\n\nPlease wait...\n');
    
    else
    
        fprintf('\nSimulating observer-based FFDL-MFATSMC on two-link robot manipulator...\n\nPlease wait...\n');
        
    end
    
    %% Simulation
    for k = 2:nSteps
        
        y_d(:,k+1) = [1.5*cos((2*pi/4)*t(k))
                      1.2*cos((2*pi/7)*t(k))];
        
%         y_d(:,k+1) = [(2*pi/4)*cos((2*pi/4)*t(k))
%                        (2*pi/7)*cos((2*pi/7)*t(k)) ];
%         
%         y_d(:,k+1) = [sat_func_yd(y_d(1,k+1),-0.4,0.4)
%                       sat_func_yd(y_d(2,k+1),-0.6,0.6)];

%         y_d(:,k+1) = [(2*pi/10)*cos((2*pi/10)*t(k))
%                        (2*pi/7)*cos((2*pi/7)*t(k)) ];

%         y_d(:,k+1) = [sat_func_yd(y_d(1,k+1),-0.4,0.4)
%                       sat_func_yd(y_d(2,k+1),-0.6,0.6)];

        if(ii == 1)
            
            PHI_hat_1{k} = PHI_hat_1{k-1} + (eta*(delta_y(:,k)-(([PHI_hat_1{k-1} PHI_hat_2{k-1}])*([delta_y(:,k-1)' delta_u(:,k-1)'])')))*delta_y(:,k-1)'/(mu+norm(delta_y(:,k-1))^2+norm(delta_u(:,k-1))^2);
            PHI_hat_2{k} = PHI_hat_1{k-1} + (eta*(delta_y(:,k)-(([PHI_hat_1{k-1} PHI_hat_2{k-1}])*([delta_y(:,k-1)' delta_u(:,k-1)'])')))*delta_u(:,k-1)'/(mu+norm(delta_y(:,k-1))^2+norm(delta_u(:,k-1))^2);
            
        else
            
            GAMMA_1 = 2/(norm(H{k-1})^2+mu_1);
            GAMMA_2 = 2/(norm(H{k-1})^2+mu_2);
            GAMMA = [GAMMA_1 GAMMA_2]';
                        
            temp = [PHI_hat_1{k-1} PHI_hat_2{k-1}] + diag(GAMMA)*diag(e_o(:,k)-F*e_o(:,k-1))*(H{k-1}*ones(1,2))';
            
            PHI_hat_1{k} = temp(:,1:2);
            PHI_hat_2{k} = temp(:,3:4);
            
        end
        
        for i = 1:nInputs
            for j = 1:nOutputs
                if(sign(PHI_hat_1{k}(i,j)) ~= sign(PHI_hat_1{1}(i,j)))    % better!
%                 if(abs(PHI_hat_1{k}(i,j)) <= epsilon)
                    PHI_hat_1{k}(i,j) = PHI_hat_1{1}(i,j);
                end
            end
        end
        
        for i = 1:nInputs
            for j = 1:nOutputs
                if(sign(PHI_hat_2{k}(i,j)) ~= sign(PHI_hat_2{1}(i,j)))    % better!
%                     if(abs(PHI_hat_2{k}(i,j)) <= epsilon)
                    PHI_hat_2{k}(i,j) = PHI_hat_2{1}(i,j);
                end
            end
        end
        
        e(:,k) = y(:,k) - y_d(:,k);
        
        if(ii == 1)
            
            s(:,k) = e(:,k);
            
            u_MFA(:,k) = u_MFA(:,k-1) + (rho*PHI_hat_2{k}'*(y_d(:,k+1)-y(:,k)-PHI_hat_1{k}*delta_y(:,k)))/(norm(PHI_hat_2{k})^2+lambda);
            
            u_SM(:,k) = (PHI_hat_2{k}^-1)*(y_d(:,k+1)-PHI_hat_1{k}*delta_y(:,k)-y(:,k)+(1-qqq*T)*s(:,k)-ep*T*sign(s(:,k)));
            
            u(:,k) = u_MFA(:,k) + GAMMA*u_SM(:,k);
            
        else
            
%             s(:,k) = s(:,k-1) + lambda_1*e(:,k) + lambda_2*sig_func(e(:,k-1),pp/qq);
            s(:,k) = lambda_1*e(:,k) + lambda_2*sig_func(e(:,k-1),pp/qq);
            
%             u_eq(:,k) = u_eq(:,k-1) + ((PHI_hat_2{k}+m)^-1)*(y_d(:,k+1)-y(:,k)-PHI_hat_1{k}*delta_y(:,k)-(2*I-F)*e_o(:,k)+e_o(:,k-1)-(lambda_1^-1)*lambda_2*sig_func(e(:,k),pp/qq));
            u_eq(:,k) = u_eq(:,k-1) + ((PHI_hat_2{k}+m)^-1)*(y_d(:,k+1)-y(:,k)-PHI_hat_1{k}*delta_y(:,k)-(2*I-F)*e_o(:,k)+e_o(:,k-1)+(lambda_1^-1)*s(:,k)-(lambda_1^-1)*lambda_2*sig_func(e(:,k),pp/qq));
            
%             u_sw(:,k) = u_sw(:,k-1) + ((PHI_hat_2{k}+m)^-1)*(-lambda_s)*sign(s(:,k));
            u_sw(:,k) = u_sw(:,k-1) + ((PHI_hat_2{k}+m)^-1)*(-lambda_s)*sat_func(s(:,k),10);
            
            u(:,k) = u_eq(:,k) + u_sw(:,k);
            
        end
        
        delta_u(:,k) = u(:,k) - u(:,k-1);
        
        %% Plant
%         d(:,k) = [(5*sin(t(k)))*(t(k)>=5 && t(k)<=8)
%                   (5*sin(t(k)))*(t(k)>=5 && t(k)<=8)];
              
        d(:,k) = [(8)*(t(k)>=5 && t(k)<=8)
                  (8)*(t(k)>=5 && t(k)<=8)];
        
        qddot = twoLink_robot_dynamics(1*t(k),q,qdot,u(:,k),d(:,k));
        qdot = qdot + T*qddot;
        q = q + T*qdot;
        
        y(:,k+1) = qdot;
        
        %%
        delta_y(:,k+1) = y(:,k+1) - y(:,k);
        
        if(ii == 2)
            
            H{k} = [delta_y(:,k)' delta_u(:,k)']';
            
            y_hat(:,k+1) = y_hat(:,k) + [PHI_hat_1{k} PHI_hat_2{k}]*H{k} + K*e_o(:,k);
            
            e_o(:,k+1) = y(:,k+1) - y_hat(:,k+1);
            
        end
        
    end
    
    if(ii == 1)
        
        PHI_hat_1_MFASMC = PHI_hat_1;
        PHI_hat_2_MFASMC = PHI_hat_2;
        
        e_MFASMC = e;
        
        s_MFASMC = s;
        
        u_MFASMC = u;
        
        delta_u_MFASMC = delta_u;
        
        y_MFASMC = y;
        
    else
        
        PHI_hat_1_OMFATSMC = PHI_hat_1;
        PHI_hat_2_OMFATSMC = PHI_hat_2;
        
        e_OMFATSMC = e;
        
        s_OMFATSMC = s;
        
        u_OMFATSMC = u;
        
        delta_u_OMFATSMC = delta_u;
        
        y_OMFATSMC = y;
        
    end
    
end

fprintf('\nSuccessfully done!\n\n');

PHI_hat_1_MFASMC_mat = cell2mat(PHI_hat_1_MFASMC);
PHI_hat_1_11_MFASMC = PHI_hat_1_MFASMC_mat(1,1:2:end);
PHI_hat_1_12_MFASMC = PHI_hat_1_MFASMC_mat(1,2:2:end);
PHI_hat_1_21_MFASMC = PHI_hat_1_MFASMC_mat(2,1:2:end);
PHI_hat_1_22_MFASMC = PHI_hat_1_MFASMC_mat(2,2:2:end);

PHI_hat_2_MFASMC_mat = cell2mat(PHI_hat_2_MFASMC);
PHI_hat_2_11_MFASMC = PHI_hat_2_MFASMC_mat(1,1:2:end);
PHI_hat_2_12_MFASMC = PHI_hat_2_MFASMC_mat(1,2:2:end);
PHI_hat_2_21_MFASMC = PHI_hat_2_MFASMC_mat(2,1:2:end);
PHI_hat_2_22_MFASMC = PHI_hat_2_MFASMC_mat(2,2:2:end);

PHI_hat_1_OMFATSMC_mat = cell2mat(PHI_hat_1_OMFATSMC);
PHI_hat_1_11_OMFATSMC = PHI_hat_1_OMFATSMC_mat(1,1:2:end);
PHI_hat_1_12_OMFATSMC = PHI_hat_1_OMFATSMC_mat(1,2:2:end);
PHI_hat_1_21_OMFATSMC = PHI_hat_1_OMFATSMC_mat(2,1:2:end);
PHI_hat_1_22_OMFATSMC = PHI_hat_1_OMFATSMC_mat(2,2:2:end);

PHI_hat_2_OMFATSMC_mat = cell2mat(PHI_hat_2_OMFATSMC);
PHI_hat_2_11_OMFATSMC = PHI_hat_2_OMFATSMC_mat(1,1:2:end);
PHI_hat_2_12_OMFATSMC = PHI_hat_2_OMFATSMC_mat(1,2:2:end);
PHI_hat_2_21_OMFATSMC = PHI_hat_2_OMFATSMC_mat(2,1:2:end);
PHI_hat_2_22_OMFATSMC = PHI_hat_2_OMFATSMC_mat(2,2:2:end);

y_MFASMC = y_MFASMC(:,1:end-1);
y_OMFATSMC = y_OMFATSMC(:,1:end-1);

y_d = y_d(:,1:end-1);

%% Performance indices
IAE_MFASMC = sum((abs(e_MFASMC)'))';
IAE_OMFATSMC = sum((abs(e_OMFATSMC)'))';

fprintf('*********************************************************************\n');
fprintf('*\t\t\t\t\t\tIAE of output #1\t\tIAE of output #2\t*\n');
fprintf('*\t    MFASMC:\t\t\t\t%f\t\t\t\t%f\t\t*\n',IAE_MFASMC(1),IAE_MFASMC(2));
fprintf('*\t  OMFATSMC:\t\t\t\t%f\t\t\t\t%f\t\t*\n',IAE_OMFATSMC(1),IAE_OMFATSMC(2));
fprintf('*********************************************************************\n\n');

TV_MFASMC = sum((abs(delta_u_MFASMC)'))';
TV_OMFATSMC = sum((abs(delta_u_OMFATSMC)'))';

fprintf('*********************************************************************\n');
fprintf('*\t\t\t\t\t\t  TV of input #1\t\t  TV of input #2\t*\n');
fprintf('*\t    MFASMC:\t\t\t\t%f\t\t\t\t%f\t\t*\n',TV_MFASMC(1),TV_MFASMC(2));
fprintf('*\t  OMFATSMC:\t\t\t\t%f\t\t\t\t%f\t\t*\n',TV_OMFATSMC(1),TV_OMFATSMC(2));
fprintf('*********************************************************************\n\n');

toc;
%% Plot results
% Dynamics of the PPD
figure(1);
subplot(1,2,1);
plot(t,PHI_hat_1_11_MFASMC,'r','Linewidth',1.5);
hold on;
plot(t,PHI_hat_1_12_MFASMC,'g','Linewidth',1.5);
plot(t,PHI_hat_1_21_MFASMC,'b','Linewidth',1.5);
plot(t,PHI_hat_1_22_MFASMC,'c','Linewidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('PPD estimation');
title('$\hat{\Phi}_1(k)$','Interpreter','LATEX');
lg_1_handle = legend('$\hat{\phi}_{1_{11}}$','$\hat{\phi}_{1_{12}}$',...
                     '$\hat{\phi}_{1_{21}}$','$\hat{\phi}_{1_{22}}$');
lg_1_handle.Interpreter = 'LATEX';
lg_1_handle.FontName = 'Times';
lg_1_handle.FontSize = 14;

ttl_1_handle = title('$\hat{\Phi}(k)$ for FFDL-MFASMC');
ttl_1_handle.Interpreter = 'LATEX';
ttl_1_handle.FontName = 'Times';
ttl_1_handle.FontSize = 14;

subplot(1,2,2);
plot(t,PHI_hat_2_11_MFASMC,'r','Linewidth',1.5);
hold on;
plot(t,PHI_hat_2_12_MFASMC,'g','Linewidth',1.5);
plot(t,PHI_hat_2_21_MFASMC,'b','Linewidth',1.5);
plot(t,PHI_hat_2_22_MFASMC,'c','Linewidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('PPD estimation');
title('$\hat{\Phi}_2(k)$','Interpreter','LATEX');
lg_2_handle = legend('$\hat{\phi}_{2_{11}}$','$\hat{\phi}_{2_{12}}$',...
                     '$\hat{\phi}_{2_{21}}$','$\hat{\phi}_{2_{22}}$');
lg_2_handle.Interpreter = 'LATEX';
lg_2_handle.FontName = 'Times';
lg_2_handle.FontSize = 14;

figure(2);
subplot(1,2,1);
plot(t,PHI_hat_1_11_OMFATSMC,'r','Linewidth',1.5);
hold on;
plot(t,PHI_hat_1_12_OMFATSMC,'g','Linewidth',1.5);
plot(t,PHI_hat_1_21_OMFATSMC,'b','Linewidth',1.5);
plot(t,PHI_hat_1_22_OMFATSMC,'c','Linewidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('PPD estimation');
title('$\hat{\Phi}_1(k)$','Interpreter','LATEX');
lg_3_handle = legend('$\hat{\phi}_{1_{11}}$','$\hat{\phi}_{1_{12}}$',...
                     '$\hat{\phi}_{1_{21}}$','$\hat{\phi}_{1_{22}}$');
lg_3_handle.Interpreter = 'LATEX';
lg_3_handle.FontName = 'Times';
lg_3_handle.FontSize = 14;

ttl_2_handle = title('$\hat{\Phi}(k)$ for Observer-based FFDL-MFATSMC');
ttl_2_handle.Interpreter = 'LATEX';
ttl_2_handle.FontName = 'Times';
ttl_2_handle.FontSize = 14;

subplot(1,2,2);
plot(t,PHI_hat_2_11_OMFATSMC,'r','Linewidth',1.5);
hold on;
plot(t,PHI_hat_2_12_OMFATSMC,'g','Linewidth',1.5);
plot(t,PHI_hat_2_21_OMFATSMC,'b','Linewidth',1.5);
plot(t,PHI_hat_2_22_OMFATSMC,'c','Linewidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('PPD estimation');
title('$\hat{\Phi}_2(k)$','Interpreter','LATEX');
lg_4_handle = legend('$\hat{\phi}_{2_{11}}$','$\hat{\phi}_{2_{12}}$',...
                     '$\hat{\phi}_{2_{21}}$','$\hat{\phi}_{2_{22}}$');
lg_4_handle.Interpreter = 'LATEX';
lg_4_handle.FontName = 'Times';
lg_4_handle.FontSize = 14;

%% Sliding surfaces
figure(3);
subplot(2,1,1);
plot(t,s_MFASMC(1,:),'b','LineWidth',1.5);
hold on;
plot(t,s_OMFATSMC(1,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('s_1');
legend('MFASMC (wang et al. [])','O-MFATSMC');

title('Sliding surfaces');

subplot(2,1,2);
plot(t,s_MFASMC(2,:),'b','LineWidth',1.5);
hold on;
plot(t,s_OMFATSMC(2,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('s_2');
legend('MFASMC (wang et al. [])','O-MFATSMC');

%% Tracking errors
figure(4);
subplot(2,1,1);
plot(t,e_MFASMC(1,:),'b','LineWidth',1.5);
hold on;
plot(t,e_OMFATSMC(1,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('e_1');
legend('MFASMC (wang et al. [])','O-MFATSMC');

title('Tracking errors');

subplot(2,1,2);
plot(t,e_MFASMC(2,:),'b','LineWidth',1.5);
hold on;
plot(t,e_OMFATSMC(2,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('e_2');
legend('MFASMC (wang et al. [])','O-MFATSMC');

%% Control inputs
figure(5);
subplot(2,1,1);
plot(t,u_MFASMC(1,:),'b','LineWidth',1.5);
hold on;
plot(t,u_OMFATSMC(1,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('u_1');
legend('MFASMC (wang et al. [])','O-MFATSMC');

title('Control inputs');

subplot(2,1,2);
plot(t,u_MFASMC(2,:),'b','LineWidth',1.5);
hold on;
plot(t,u_OMFATSMC(2,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('u_2');
legend('MFASMC (wang et al. [])','O-MFATSMC');

%% Plant outputs
ROI = 4800:5600;

figure(6);
subplot(2,1,1);
plot(t,y_d(1,:),'k--','LineWidth',2);
hold on;
plot(t,y_MFASMC(1,:),'b-.','LineWidth',1.5);
plot(t,y_OMFATSMC(1,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('y_1');
legend('Desired','MFASMC (wang et al. [])','O-MFATSMC');
axes('position',[0.37,0.590,0.15,0.15]);
plot(t(ROI),y_d(1,ROI),'k--','LineWidth',2);
hold on;
plot(t(ROI),y_MFASMC(1,ROI),'b-.','LineWidth',1.5);
plot(t(ROI),y_OMFATSMC(1,ROI),'r','LineWidth',1.5);
xlim([4.8,5.6]);
ylim([-2.6,1.2]);

title('Plant outputs');

subplot(2,1,2);
plot(t,y_d(2,:),'k--','LineWidth',2);
hold on;
plot(t,y_MFASMC(2,:),'b-.','LineWidth',1.5);
plot(t,y_OMFATSMC(2,:),'r','LineWidth',1.5);
grid on;
xlabel('Time (sec)');
ylabel('y_2');
legend('Desired','MFASMC (wang et al. [])','O-MFATSMC');
axes('position',[0.35,0.275,0.15,0.15]);
plot(t(ROI),y_d(2,ROI),'k--','LineWidth',2);
hold on;
plot(t(ROI),y_MFASMC(2,ROI),'b-.','LineWidth',1.5);
plot(t(ROI),y_OMFATSMC(2,ROI),'r','LineWidth',1.5);
xlim([4.8,5.6]);
ylim([-0.6,0.5]);
