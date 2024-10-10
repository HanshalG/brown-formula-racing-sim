% Read the image and extract the red channel
img = imread('cleanedtrack.png'); % Update the path as needed
if size(img, 3) == 3
    red_channel = img(:, :, 1);
else
    red_channel = img; % If the image is already grayscale
end

% Define upper and lower bounds for red thresholding
lower_bound = 100; % Adjust these values as needed
upper_bound = 200; % Adjust these values as needed

% Apply thresholding to extract the track
binary_img = (red_channel >= lower_bound) & (red_channel <= upper_bound);

% Use morphological operations to clean up noise
binary_img = imclose(binary_img, strel('disk', 3)); % Close gaps in the track
binary_img = imfill(binary_img, 'holes'); % Fill any holes in the track

% Extract edges using edge detection
edges = edge(binary_img, 'Canny');

% Find the coordinates of edge points
[y, x] = find(edges);
points = [x, y]; % Store the x and y coordinates

% Start with the topmost point
[~, start_idx] = min(points(:,2));
ordered_points = points(start_idx, :);
remaining_points = points;
remaining_points(start_idx, :) = [];

% Define the maximum distance to consider for nearby points
max_distance = 0.005; % Adjust this value based on your image scale

% Order points by nearest neighbor, considering only nearby points
while ~isempty(remaining_points)
    last_point = ordered_points(end, :);
    distances = pdist2(last_point, remaining_points);
    nearby_indices = find(distances <= max_distance);
    
    if isempty(nearby_indices)
        % If no nearby points, find the closest point among all remaining points
        [~, nearest_idx] = min(distances);
    else
        % Find the nearest point among nearby points
        [~, nearest_local_idx] = min(distances(nearby_indices));
        nearest_idx = nearby_indices(nearest_local_idx);
    end
    
    ordered_points = [ordered_points; remaining_points(nearest_idx, :)];
    remaining_points(nearest_idx, :) = [];
end

x_sorted = ordered_points(:, 1);
y_sorted = ordered_points(:, 2);

% Use piecewise linear interpolation
num_samples = 200;
%% 
t = 1:length(x_sorted);
tt = linspace(1, length(x_sorted), num_samples);
xq = interp1(t, x_sorted, tt, 'linear');
yq = interp1(t, y_sorted, tt, 'linear');

% Calculate distances between consecutive points to determine length
meters_per_pixel = 0.01; % Adjust this based on your image scale
distances = sqrt(diff(xq).^2 + diff(yq).^2) * meters_per_pixel;

% Calculate curvature using central differences
dx = gradient(xq);
dy = gradient(yq);
ddx = gradient(dx);
ddy = gradient(dy);
curvatures = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);

% Define a threshold for curvature to differentiate straights and curves
curvature_threshold = median(curvatures) * 2; % Adaptive threshold

% Initialize arrays to store turn information
turn_type = {};
turn_length = [];
turn_radius = [];

% Classify segments and calculate lengths and radii
i = 1;
while i < length(curvatures)
    if curvatures(i) > curvature_threshold
        % Detect a curve
        start_idx = i;
        while i < length(curvatures) && curvatures(i) > curvature_threshold
            i = i + 1;
        end
        end_idx = i;

        % Calculate length and radius of the curve segment
        curve_length = sum(distances(start_idx:end_idx-1));
        curve_radius = 1 / mean(curvatures(start_idx:end_idx));

        % Store as curve
        turn_type{end+1} = 'Curve';
        turn_length(end+1) = curve_length;
        turn_radius(end+1) = curve_radius;
    else
        % Detect a straight
        start_idx = i;
        while i < length(curvatures) && curvatures(i) <= curvature_threshold
            i = i + 1;
        end
        end_idx = i;

        % Calculate length of the straight segment
        straight_length = sum(distances(start_idx:end_idx-1));

        % Store as straight with a large radius (straight approximation)
        turn_type{end+1} = 'Straight';
        turn_length(end+1) = straight_length;
        turn_radius(end+1) = Inf; % Use Inf for straight segments
    end
end

% Create a table and save to CSV
turn_data = table(turn_type', turn_length', turn_radius', ...
    'VariableNames', {'Turn Type', 'Length', 'Radius'});
writetable(turn_data, 'track_analysis.csv');

% Plot detected points on the track
figure;
imshow(binary_img);
hold on;
plot(x_sorted, y_sorted, 'g.', 'MarkerSize', 2); % Detected points in green
title('Detected Points on Track');
hold off;

% Plot interpolated points on the track
figure;
imshow(binary_img);
hold on;
plot(xq, yq, 'r-', 'LineWidth', 2); % Interpolated points in red
title('Piecewise Linear Interpolation of Track');
hold off;

% Plot curves and straights identified on the track
figure;
imshow(binary_img);
hold on;
cumulative_length = [0, cumsum(turn_length)];
for idx = 1:length(turn_type)
    start_idx = find(cumsum(distances) >= cumulative_length(idx), 1);
    end_idx = find(cumsum(distances) >= cumulative_length(idx+1), 1);
    
    if strcmp(turn_type{idx}, 'Curve')
        plot(xq(start_idx:end_idx), yq(start_idx:end_idx), 'r', 'LineWidth', 2); % Curves in red
    else
        plot(xq(start_idx:end_idx), yq(start_idx:end_idx), 'b', 'LineWidth', 2); % Straights in blue
    end
end
title('Identified Curves and Straights on Track');
hold off;

disp('Track analysis complete. Data saved to track_analysis.csv');