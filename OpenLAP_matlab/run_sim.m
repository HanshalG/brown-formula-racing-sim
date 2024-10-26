clear
clc
close all force
diary('off')
fclose('all') ;

exhaust_lengths = [175 195 215 235 255 275 295 315 335 355 375 415 435 455 475 495 515 525]
intake_lengths = [100, 104.0816345, 108.163269, 112.2448959, 116.3265305, 120.408165, 124.4897995, 128.5714264, 132.6530609, 136.7346954, 140.81633, 144.8979645, 148.979599, 153.0612183, 157.14285280000001, 161.2244873, 165.3061218, 169.3877563, 173.4693909, 177.5510254, 181.6326599, 185.7142792, 189.7959137, 193.8775482, 197.9591827, 202.0408173, 206.1224518, 210.2040863, 214.2857208, 218.3673401, 222.4489746, 226.5306091, 230.6122437, 234.6938782, 238.7755127, 242.85714719999999, 246.9387817, 251.020401, 255.1020355, 259.1836853, 263.2653198, 267.3469238, 271.4285583, 275.5101929, 279.5918274, 283.6734619, 287.7550964, 291.836731, 295.9183655, 300]
AFR = [10 10.10204124 10.20408154 10.30612278 10.40816307 ...
 10.51020432 10.61224461 10.71428585 10.81632614 10.91836739 11.02040863 ...
 11.12244892 11.22449017 11.32653046 11.4285717  11.53061199 11.63265324 ...
 11.73469353 11.83673477 11.93877506 12.04081631 12.14285755 12.24489784 ...
 12.34693909 12.44897938 12.55102062 12.65306091 12.75510216 12.85714245 ...
 12.95918369 13.06122494 13.16326523 13.26530647 13.36734676 13.46938801 ...
 13.5714283  13.67346954 13.77550983 13.87755108 13.97959137 14.08163261 ...
 14.18367386 14.28571415 14.38775539 14.48979568 14.59183693 14.69387722 ...
 14.79591846 14.89795876 15.        ]
score_list = zeros(1, length(exhaust_lengths))
accel_times = zeros(1, length(exhaust_lengths))
skidpad_times = zeros(1, length(exhaust_lengths))
endurance_times = zeros(1, length(exhaust_lengths))
autocross_times = zeros(1, length(exhaust_lengths))
endurance_scores = zeros(1, length(exhaust_lengths))
accel_scores = zeros(1, length(exhaust_lengths))

%{
for i =1:length(exhaust_lengths)
    exhaust_lengths(i)
    create_vehicle(['Exhaust Files/Rhode_Rage_' sprintf('%.10g', exhaust_lengths(i)) '.xlsx']);
    accel_time = run_accel('Rhode Rage', false);
    skidpad_time = run_lap('Rhode Rage', 'FSAE Skidpad_Closed_Forward', false) / 2;
    autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', false);
    endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;
    
    accel_score = calculate_accel_score(accel_time);
    accel_times(i) = accel_time;
    accel_scores(i) = accel_score;
    skidpad_times(i) = skidpad_time;
    skidpad_score = calculate_skidpad_score(skidpad_time);
    autocross_times(i) = autocross_time;
    autocross_score = calculate_autocross_score(autocross_time, 51.569);
    autocross_scores(i) = autocross_score;
    endurance_times(i) = endurance_time;
    endurance_score = calculate_endurance_score(endurance_time, 1360);
    endurance_scores(i) = endurance_score;
    
    total_score = accel_score + skidpad_score + autocross_score + endurance_score;

    %efficiency_score = calculate_efficiency_score(endurance_time, 1067)
    score_list(i) = total_score;
end
%}

% Initialize variables
accel_times = [];
accel_scores = [];
skidpad_times = [];
skidpad_scores = [];
autocross_times = [];
autocross_scores = [];
endurance_times = [];
endurance_scores = [];
total_scores = [];
mass_vals = []; % Store mass values for contour plot
cl_vals = [];   % Store lift coefficient values for contour plot

% Loop through mass and lift coefficient values
for m=150:5:250
    for cl=2:0.2:6
        cd = -1.635;
        create_vehicle_params('vehicle_descriptions/Rhode Rage.xlsx', m, -cl, cd);

        % Run simulations for each test
        accel_time = run_accel('Rhode Rage', false);
        skidpad_time = run_skidpad('Rhode Rage', false);
        autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', true);
        endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 10;

        % Calculate scores
        accel_score = calculate_accel_score(accel_time);
        skidpad_score = calculate_skidpad_score(skidpad_time);
        autocross_score = calculate_autocross_score(autocross_time, 51.569);
        endurance_score = calculate_endurance_score(endurance_time, 1360);

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

        % Store mass and cl values for plotting
        mass_vals = [mass_vals m];
        cl_vals = [cl_vals cl];
    end
end

%% 


% Reshape total_scores to fit the contour plot grid
mass_range = 150:5:250;   % Corresponds to the range of mass values
cl_range = 2:0.2:6;       % Corresponds to the range of cl values
[MassGrid, ClGrid] = meshgrid(mass_range, cl_range);
TotalScoreGrid = reshape(total_scores, length(cl_range), length(mass_range));

% Plot the contour plot
figure;
contourf(MassGrid, ClGrid, TotalScoreGrid, 20);  % Filled contour plot with 20 levels
colorbar; % Show color scale
xlabel('Mass (kg)');
ylabel('Lift Coefficient (Cl)');
title('Total Score Contour Plot for Mass and Lift Coefficient');

%% 

%score_list

%plot(intake_lengths, score_list)

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
