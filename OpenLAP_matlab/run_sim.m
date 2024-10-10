clear
clc
close all force
diary('off')
fclose('all') ;

intake_lengths = [100, 104.0816345, 108.163269, 112.2448959, 116.3265305, 120.408165, 124.4897995, 128.5714264, 132.6530609, 136.7346954, 140.81633, 144.8979645, 148.979599, 153.0612183, 157.14285280000001, 161.2244873, 165.3061218, 169.3877563, 173.4693909, 177.5510254, 181.6326599, 185.7142792, 189.7959137, 193.8775482, 197.9591827, 202.0408173, 206.1224518, 210.2040863, 214.2857208, 218.3673401, 222.4489746, 226.5306091, 230.6122437, 234.6938782, 238.7755127, 242.85714719999999, 246.9387817, 251.020401, 255.1020355, 259.1836853, 263.2653198, 267.3469238, 271.4285583, 275.5101929, 279.5918274, 283.6734619, 287.7550964, 291.836731, 295.9183655, 300]

score_list = zeros(1, length(intake_lengths))
accel_times = zeros(1, length(intake_lengths))
skidpad_times = zeros(1, length(intake_lengths))
endurance_times = zeros(1, length(intake_lengths))
autocross_times = zeros(1, length(intake_lengths))
endurance_scores = zeros(1, length(intake_lengths))

create_vehicle('vehicle_descriptions/Rhode Rage.xlsx');
    
accel_time = run_accel('Rhode Rage', true);
    
skidpad_time = run_skidpad('Rhode Rage', true);

autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', true);

accel_time
skidpad_time

%{
for i =1:length(intake_lengths)
    create_vehicle(['Output_Files/Rhode_Rage_' sprintf('%.10g', intake_lengths(i)) '.xlsx']);
    
    accel_time = run_drag('Rhode Rage', false);
    
    skidpad_time = run_lap('Rhode Rage', 'FSAE Skidpad_Closed_Forward', false) / 2;
    
    autocross_time = run_lap('Rhode Rage', 'FSAE Autocross Nebraska 2013_Open_Forward', false);
    
    endurance_time = run_lap('Rhode Rage', 'FSAE Endurance Michigan 2012_Closed_Forward', false) * 20;
    
    accel_score = calculate_accel_score(accel_time);
    accel_times(i) = calculate_accel_score(accel_time);
    skidpad_times(i) = skidpad_time;
    skidpad_score = calculate_skidpad_score(skidpad_time);
    autocross_times(i) = autocross_time;
    autocross_score = calculate_autocross_score(autocross_time, 51.569);
    endurance_times(i) = endurance_time;
    endurance_score = calculate_endurance_score(endurance_time, 1067);
    endurance_scores(i) = endurance_score
    
    total_score = accel_score + skidpad_score + autocross_score + endurance_score;

    %efficiency_score = calculate_efficiency_score(endurance_time, 1067)
    score_list(i) = total_score;
end
%}
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
