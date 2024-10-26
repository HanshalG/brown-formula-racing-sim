clear
clc
close all force
diary('off')
fclose('all') ;

intake_lengths = [100 104.08163452 108.16326904 112.24489594 ...
 116.32653046 120.40816498 124.4897995  128.57142639 132.65306091 ...
 136.73469543 140.81632996 144.89796448 148.979599   153.06121826 ...
 157.14285278 161.2244873  165.30612183 169.38775635 173.46939087 ...
 177.55102539 181.63265991 185.71427917 189.7959137  193.87754822 ...
 197.95918274 202.04081726 206.12245178 210.2040863  214.28572083 ...
 218.36734009 222.44897461 226.53060913 230.61224365 234.69387817 ...
 238.7755127  242.85714722 246.93878174 251.020401   255.10203552 ...
 259.1836853  263.26531982 267.34692383 271.42855835 275.51019287 ...
 279.59182739 283.67346191 287.75509644 291.83673096 295.91836548 ...
 300]

accel_times = [];
accel_scores = [];
skidpad_times = [];
skidpad_scores = [];
autocross_times = [];
autocross_scores = [];
endurance_times = [];
endurance_scores = [];
total_scores = [];

for i =1:length(intake_lengths)
    create_vehicle(['Intake_Files/Rhode_Rage_' num2str(i) '.xlsx']);
    % Run simulations for each test
    accel_time = run_accel('Rhode Rage', false);
    skidpad_time = run_skidpad('Rhode Rage', false) / 2;
    autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', false);
    endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

    % Calculate scores
    accel_score = calculate_accel_score(accel_time);
    skidpad_score = calculate_skidpad_score(skidpad_time);
    autocross_score = calculate_autocross_score(autocross_time, 51.569);
    endurance_score = calculate_endurance_score(endurance_time, 1395);

    % Append times and scores to arrays
    accel_times = [accel_times accel_time];
    accel_scores = [accel_scores accel_score];
    skidpad_times = [skidpad_times skidpad_time];
    skidpad_scores = [skidpad_scores skidpad_score];
    autocross_times = [autocross_times autocross_time];
    autocross_scores = [autocross_scores autocross_score];
    endurance_times = [endurance_times endurance_time];
    endurance_scores = [endurance_scores endurance_score];

    % Calculate total score
    total_score = accel_score + skidpad_score + autocross_score + endurance_score;
    total_scores = [total_scores total_score];
end

%%

% Subtract the minimum total score from all total scores
min_total_score = min(total_scores);
total_scores = total_scores - min_total_score;

figure;

% Plot 1: Exhaust Lengths vs Acceleration Score
subplot(3,2,1); % 3 rows, 2 columns, position 1
plot(intake_lengths, accel_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('Exhaust Length vs Acceleration Score');
xlabel('Exhaust Length (mm)');
ylabel('Acceleration Score');
grid on;

% Plot 2: Exhaust Lengths vs Skidpad Score
subplot(3,2,2); % position 2
plot(intake_lengths, skidpad_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('Exhaust Length vs Skidpad Score');
xlabel('Exhaust Length (mm)');
ylabel('Skidpad Score');
grid on;

% Plot 3: Exhaust Lengths vs Autocross Score
subplot(3,2,3); % position 3
plot(intake_lengths, autocross_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('Exhaust Length vs Autocross Score');
xlabel('Exhaust Length (mm)');
ylabel('Autocross Score');
grid on;

% Plot 4: Exhaust Lengths vs Endurance Score
subplot(3,2,4); % position 4
plot(intake_lengths, endurance_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('Exhaust Length vs Endurance Score');
xlabel('Exhaust Length (mm)');
ylabel('Endurance Score');
grid on;

% Plot 5: Exhaust Lengths vs Total Scores
subplot(3,2,5); % position 5
plot(intake_lengths, total_scores, '-o', 'LineWidth', 2, 'MarkerSize', 6);
title('Intake Length vs Total Scores');
xlabel('Intake Lengths (mm)');
ylabel('Score Delta');
grid on;

% Adjust layout for better visibility
sgtitle('Performance Scores vs Exhaust Lengths');

figure;

% Plot: Exhaust Lengths vs Total Scores (without markers)
plot(intake_lengths, total_scores, '-','LineWidth', 2); % Remove the 'o' to exclude markers
title('Intake Length vs Score Delta');
xlabel('Intake Length (mm)');
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

function output = calculate_efficiency_score(time, t_min, CO2min, CO2your)
    output = (t_min/20)/(time/20) * (CO2min/10)/(CO2your/10)
end
