%% Code to estimate ankle angle from shank and foot mounted IMUs
% Tanner Morreale & Edgar Bolivar - Sep. 2023

clc, clearvars, close all % Prefer a clear the workspace to have repeatable results

Table_s = readtable("Walking_4-000_00B4A3D1.txt");
Roll_s = Table_s{:,20};
Pitch_s = Table_s{:,21};
Yaw_s = Table_s{:,22};
Acc_X_s = Table_s{:,7};
Acc_Y_s = Table_s{:,8};
Acc_Z_s = Table_s{:,9};
Gyro_X_s = Table_s{:,13};
Gyro_Y_s = Table_s{:,14};
Gyro_Z_s = Table_s{:,15};
R_s = reshape(table2array(Table_s(:, 23:31)).', [3, 3, 3190]); % Notice the transpose

Table_f = readtable("Walking_4-000_00B4A3CC.txt");
Roll_f = Table_f{:,20};
Pitch_f = Table_f{:,21};
Yaw_f = Table_f{:,22};
Acc_X_f = Table_f{:,7};
Acc_Y_f = Table_f{:,8};
Acc_Z_f = Table_f{:,9};
Gyro_X_f = Table_f{:,13};
Gyro_Y_f = Table_f{:,14};
Gyro_Z_f = Table_f{:,15};
R_f = reshape(table2array(Table_f(:, 23:31)).', [3, 3, 3190]);

%% Getting the joint angle from the rotation matrix

n = length(Roll_f);
phiCal = zeros(n,1);
ankAng = zeros(n,1);
R_f_s  = zeros(3,3,n);

for i = 1:n
    % Calculate roll angle from rotation matrix as a sanity check
    theta = asin(-R_f(3, 1, i));
    phiCal(i) = atan2d(R_f(3,2,i)/cos(theta), R_f(3,3,i)/cos(theta));

    % Create rotation matrix from foot frame to shank frame - Notice the
    % transpose
    R_f_s(:,:,i) = R_f(:,:,i).'*R_s(:,:,i);
    % Calculate Angle from fixed axis - (Subject to change depending on
    % validation experiments)
    ankAng(i) = asind(-R_f_s(3,1,i));
end

% Plot Roll angle as a sanity check of the operation
figure, hold on, grid on, title('Sanity Check')
plot(phiCal);
plot(Roll_f, '--o');
legend('Calculated', 'XSENS')
xlabel('Sample')
ylabel('Roll angle [deg.]')

% Plot ankle angle from absolute rotation around the y-axis
figure, hold on, grid on, title('Estimation of Ankle Angle')
plot(ankAng-ankAng(1), '--o');
xlabel('Sample')
ylabel('Ankle angle [deg.]')
