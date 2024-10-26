clear
clc
close all force
diary('off')
fclose('all');

% Initialize variables
accel_times = [];
accel_scores = [];
skidpad_times = [];
skidpad_scores = [];
autocross_times = [];
autocross_scores = [];
endurance_times = [];
endurance_scores = [];
total_scores_mass_cl = [];
total_scores_cl_cd = [];
total_scores_mass_cd = [];

% Define constant values for the third variable in each plot
constant_cd = -1.635;   % For mass vs. Cl plot
constant_mass = 316;  % For Cl vs. Cd plot (e.g., 270 kg)
constant_cl = -3.066;      % For mass vs. Cd plot (e.g., Cl = 4)

mass_range = 220:25:320;
cl_range = -6:0.5:-2;
cd_range = -3:0.25:-1;

% 1. Loop through mass and Cl values, keeping Cd constant
for m = mass_range
    for cl = cl_range
        cd = constant_cd;
        create_vehicle_params('vehicle_descriptions/Rhode Rage.xlsx', m, cl, cd);

        % Run simulations for each test
        accel_time = run_accel('Rhode Rage', false);
        skidpad_time = run_skidpad('Rhode Rage', false) / 2;
        autocross_time = run_lap('Rhode Rage', 'Autocross 2024_Open_Forward', false)
        endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

        % Calculate scores
        accel_score = calculate_accel_score(accel_time);
        skidpad_score = calculate_skidpad_score(skidpad_time);
        autocross_score = calculate_autocross_score(autocross_time, 51.569);
        endurance_score = calculate_endurance_score(endurance_time, 1360);

        % Calculate total score and store it
        total_score = accel_score + skidpad_score + autocross_score + endurance_score;
        total_scores_mass_cl = [total_scores_mass_cl total_score];
    end
end

% 2. Loop through Cl and Cd values, keeping mass constant
for cl = cl_range
    for cd = cd_range
        create_vehicle_params('vehicle_descriptions/Rhode Rage.xlsx', constant_mass, cl, cd);

        % Run simulations for each test
        accel_time = run_accel('Rhode Rage', false);
        skidpad_time = run_skidpad('Rhode Rage', false);
        autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', false);
        endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

        % Calculate scores
        accel_score = calculate_accel_score(accel_time);
        skidpad_score = calculate_skidpad_score(skidpad_time);
        autocross_score = calculate_autocross_score(autocross_time, 51.569);
        endurance_score = calculate_endurance_score(endurance_time, 1360);

        % Calculate total score and store it
        total_score = accel_score + skidpad_score + autocross_score + endurance_score;
        total_scores_cl_cd = [total_scores_cl_cd total_score];
    end
end

% 3. Loop through mass and Cd values, keeping Cl constant
for m = mass_range
    for cd = cd_range
        create_vehicle_params('vehicle_descriptions/Rhode Rage.xlsx', m, constant_cl, cd);

        % Run simulations for each test
        accel_time = run_accel('Rhode Rage', false);
        skidpad_time = run_skidpad('Rhode Rage', false);
        autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', false);
        endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

        % Calculate scores
        accel_score = calculate_accel_score(accel_time);
        skidpad_score = calculate_skidpad_score(skidpad_time);
        autocross_score = calculate_autocross_score(autocross_time, 51.569);
        endurance_score = calculate_endurance_score(endurance_time, 1360);

        % Calculate total score and store it
        total_score = accel_score + skidpad_score + autocross_score + endurance_score;
        total_scores_mass_cd = [total_scores_mass_cd total_score];
    end
end

% Subtract the minimum total score from each set for better comparison
min_total_score_mass_cl = min(total_scores_mass_cl);
total_scores_mass_cl = total_scores_mass_cl - min_total_score_mass_cl;

min_total_score_cl_cd = min(total_scores_cl_cd);
total_scores_cl_cd = total_scores_cl_cd - min_total_score_cl_cd;

min_total_score_mass_cd = min(total_scores_mass_cd);
total_scores_mass_cd = total_scores_mass_cd - min_total_score_mass_cd;

%%

% 1. Mass vs. Cl plot
[MassGrid, ClGrid] = meshgrid(mass_range, cl_range);
TotalScoreGridMassCl = reshape(total_scores_mass_cl, length(cl_range), length(mass_range));

figure;
contourf(MassGrid, ClGrid, TotalScoreGridMassCl, 20);
colorbar;
xlabel('Mass (kg)');
ylabel('Lift Coefficient (Cl)');
title(['Score Delta for Mass vs. Cl (Cd = ' num2str(constant_cd) ')']);

% 2. Cl vs. Cd plot
[ClGrid, CdGrid] = meshgrid(cl_range, cd_range);
TotalScoreGridClCd = reshape(total_scores_cl_cd, length(cd_range), length(cl_range));

figure;
contourf(ClGrid, CdGrid, TotalScoreGridClCd, 20);
colorbar;
xlabel('Lift Coefficient (Cl)');
ylabel('Drag Coefficient (Cd)');
title(['Score Delta for Cl vs. Cd (Mass = ' num2str(constant_mass) ' kg)']);

% 3. Mass vs. Cd plot
[MassGridCd, CdGridMass] = meshgrid(mass_range, cd_range);
TotalScoreGridMassCd = reshape(total_scores_mass_cd, length(cd_range), length(mass_range));

figure;
contourf(MassGridCd, CdGridMass, TotalScoreGridMassCd, 20);
colorbar;
xlabel('Mass (kg)');
ylabel('Drag Coefficient (Cd)');
title(['Score Delta for Mass vs. Cd (Cl = ' num2str(constant_cl) ')']);

% Assuming mass_range and total_scores_mass_cl contain your mass values and score delta data
% Use the 'total_scores_mass_cl' as the y-values and 'mass_range' as the x-values.

% Check the sizes of mass_range and cl_range
disp(size(mass_range));  % Expected: 1 x 5
disp(size(cl_range));    % Expected: 1 x 9 (example)

% Reshape total_scores_mass_cl if needed
% total_scores_mass_cl should have length(cl_range) rows and length(mass_range) columns
total_scores_mass_cl = reshape(total_scores_mass_cl, length(cl_range), length(mass_range));

% Choose a specific Cl slice for fitting (e.g., the first Cl value)
fixed_cl_index = 1;  % Choose which Cl index you want to fit for

% Extract the score deltas for this Cl value
x_mass = mass_range;  % Mass remains the same
y_score_delta = total_scores_mass_cl(fixed_cl_index, :);  % Scores for the fixed Cl

%%

% Check the sizes of mass_range and cl_range
disp(size(mass_range));  % Expected: [1 5] for mass_range
disp(size(cl_range));    % Expected: [1 9] for cl_range (example, adjust if cl_range has a different size)

% Reshape total_scores_mass_cl into a 2D matrix where each row corresponds to a Cl value
% It should have length(cl_range) rows and length(mass_range) columns
total_scores_mass_cl = reshape(total_scores_mass_cl, length(cl_range), length(mass_range));

% Choose a specific Cl slice for fitting (e.g., the first Cl value)
fixed_cl_index = 1;  % Choose the Cl index you want to fit for (1 = first Cl value, 2 = second Cl value, etc.)

% Extract the score deltas for this Cl value (fixed Cl, varying mass)
x_mass = mass_range;  % Mass remains the same
y_score_delta = total_scores_mass_cl(fixed_cl_index, :);  % Scores for the fixed Cl

% Now, x_mass and y_score_delta should have the same length
disp(size(x_mass));        % Should be [1, length of mass_range]
disp(size(y_score_delta)); % Should be [1, length of mass_range]

% Fit the data if sizes match
if numel(x_mass) == numel(y_score_delta)
    % 1. First-order (linear) fit
    p1 = polyfit(x_mass, y_score_delta, 1);  % Fit a first-degree polynomial (linear)
    y_fit_1 = polyval(p1, x_mass);           % Evaluate the fitted polynomial

    % 2. Second-order (quadratic) fit
    p2 = polyfit(x_mass, y_score_delta, 2);  % Fit a second-degree polynomial (quadratic)
    y_fit_2 = polyval(p2, x_mass);           % Evaluate the fitted polynomial

    % Plot original data
    figure;
    plot(x_mass, y_score_delta, 'ko', 'MarkerSize', 8);  % Original data as black circles
    hold on;

    % Plot first-order fit
    plot(x_mass, y_fit_1, 'r-', 'LineWidth', 2);  % Linear fit as a red line

    % Plot second-order fit
    plot(x_mass, y_fit_2, 'b--', 'LineWidth', 2);  % Quadratic fit as a blue dashed line

    % Add labels and legend
    xlabel('Mass (kg)');
    ylabel('Score Delta');
    title(['First-order vs. Second-order Fit of Mass vs. Score Delta (Cl = ', num2str(cl_range(fixed_cl_index)), ')']);
    legend('Original Data', 'Linear Fit', 'Quadratic Fit', 'Location', 'best');
    grid on;
else
    error('x_mass and y_score_delta still have mismatched sizes.');
end

% Assuming you already have p1 and p2 from polyfit
% p1 = [slope, intercept]; for first-order polynomial
% p2 = [a, b, c]; for second-order polynomial

% First-order polynomial (linear fit)
slope = p1(1);
intercept = p1(2);

% Construct the LaTeX string for the first-order polynomial
first_order_eq = sprintf('y = %.3fx + %.3f', slope, intercept);

% Second-order polynomial (quadratic fit)
a = p2(1);
b = p2(2);
c = p2(3);

% Construct the LaTeX string for the second-order polynomial
second_order_eq = sprintf('y = %.3fx^2 + %.3fx + %.3f', a, b, c);

% Display the equations in the command window
disp('First-order polynomial (linear fit):');
disp(['$$' first_order_eq '$$']);
disp('Second-order polynomial (quadratic fit):');
disp(['$$' second_order_eq '$$']);


function output = calculate_accel_score(time)
    output = 95.5* ((1.5*4.206 / time)-1)/(0.5) + 4.5;
end

function output = calculate_skidpad_score(time)
    output = 71.5* ((1.25*5.18 / time)^2-1)/(1.25^2-1) + 3.5;
end

function output = calculate_autocross_score(time, t_max)
    output = 118.5 * ((1.45*t_max / time)-1)/(0.45) + 6.5;
end

function output = calculate_endurance_score(time, t_max)
    output = 250 * ((1.45*t_max / time)-1)/(0.45);
end

function output = calculate_efficiency_score(time, t_min, CO2min, CO2your)
    output = (t_min/20)/(time/20) * (CO2min/10)/(CO2your/10)
end
