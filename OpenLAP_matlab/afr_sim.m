clear
clc
close all force
diary('off')
fclose('all') ;

AFR = [10 10.10204124 10.20408154 10.30612278 10.40816307 ...
 10.51020432 10.61224461 10.71428585 10.81632614 10.91836739 11.02040863 ...
 11.12244892 11.22449017 11.32653046 11.4285717  11.53061199 11.63265324 ...
 11.73469353 11.83673477 11.93877506 12.04081631 12.14285755 12.24489784 ...
 12.34693909 12.44897938 12.55102062 12.65306091 12.75510216 12.85714245 ...
 12.95918369 13.06122494 13.16326523 13.26530647 13.36734676 13.46938801 ...
 13.5714283  13.67346954 13.77550983 13.87755108 13.97959137 14.08163261 ...
 14.18367386 14.28571415 14.38775539 14.48979568 14.59183693 14.69387722 ...
 14.79591846 14.89795876 15]

accel_times = [];
accel_scores = [];
skidpad_times = [];
skidpad_scores = [];
autocross_times = [];
autocross_scores = [];
endurance_times = [];
endurance_scores = [];
efficiency_scores = [];
total_scores = [];

for i =1:length(AFR)
    create_vehicle(['AFR_Files/Rhode_Rage_' num2str(i) '.xlsx']);
    % Run simulations for each test
    accel_time = run_accel('Rhode Rage', false);
    skidpad_time = run_skidpad('Rhode Rage', false) / 2;
    autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', true);
    endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

    % Calculate scores
    accel_score = calculate_accel_score(accel_time);
    skidpad_score = calculate_skidpad_score(skidpad_time);
    autocross_score = calculate_autocross_score(autocross_time, 51.569);
    endurance_score = calculate_endurance_score(endurance_time, 1395);

    efficiency_score = calculate_efficiency_score(endurance_time, 1395, AFR(i), 5.453, 0.784, 0.297)

    % Append times and scores to arrays
    accel_times = [accel_times accel_time];
    accel_scores = [accel_scores accel_score];
    skidpad_times = [skidpad_times skidpad_time];
    skidpad_scores = [skidpad_scores skidpad_score];
    autocross_times = [autocross_times autocross_time];
    autocross_scores = [autocross_scores autocross_score];
    endurance_times = [endurance_times endurance_time];
    endurance_scores = [endurance_scores endurance_score];
    efficiency_scores = [efficiency_scores efficiency_score];

    % Calculate total score
    total_score = accel_score + skidpad_score + autocross_score + endurance_score + efficiency_score;
    total_scores = [total_scores total_score];
end

%%

% Subtract the minimum total score from all total scores
min_total_score = min(total_scores);
total_scores = total_scores - min_total_score;

% Plot: AFR vs Acceleration Score
figure;
plot(AFR, accel_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('AFR vs Acceleration Score');
xlabel('AFR');
ylabel('Acceleration Score');
grid on;

% Plot: AFR vs Skidpad Score
figure;
plot(AFR, skidpad_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('AFR vs Skidpad Score');
xlabel('AFR');
ylabel('Skidpad Score');
grid on;

% Plot: AFR vs Autocross Score
figure;
plot(AFR, autocross_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('AFR vs Autocross Score');
xlabel('AFR');
ylabel('Autocross Score');
grid on;

% Plot: AFR vs Endurance Score
figure;
plot(AFR, endurance_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('AFR vs Endurance Score');
xlabel('AFR');
ylabel('Endurance Score');
grid on;

% Plot: AFR vs Total Scores
figure;
plot(AFR, total_scores, '-', 'LineWidth', 2, 'MarkerSize', 6);
title('Air Fuel Ratio vs Score Delta');
xlabel('Air Fuel Ratio');
ylabel('Score Delta');
grid on;

%%
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

function output = calculate_efficiency_score(time, t_min, AFR_val, CO2min, eff_max, eff_min)
    fuel_used = ((4.51 * 13.7)/AFR_val);

    CO2_FACTOR = 2.31;

    CO2your = fuel_used * CO2_FACTOR;

    efficency_factor = ((t_min/10)/(time/10)) * ((CO2min/10)/(CO2your/10));

    output = 100 * (efficency_factor - eff_min)/(eff_max - eff_min);
end
