% Example data (replace with your actual data)
% Assuming xRecord contains ECEF positions and quaternions
% xRecord(1:4,:) are the quaternions
% xRecord(5:7,:) are the ECEF positions
numTimePts = size(xRecord, 2);  % Number of time points

% Plot Vehicle Track
llaRecord = ecef2lla(xRecord(5:7, :)');
uif = uifigure();
g = geoglobe(uif);
geoplot3(g, llaRecord(:,1), llaRecord(:,2), llaRecord(:,3));

% Create the figure for the animation
fig = figure;

% Create axes within the figure
ax = axes('Parent', fig);

% Loop through the time points and update the pose plot
% for i = 1:numTimePts
%     % Get the position and quaternion orientation for the current time step
%     position = xRecord(5:7, i)';
%     quat = xRecord(1:4, i)';
%     q = quaternion(quat);
% 
%     % Normalize the quaternion to ensure it represents a valid rotation
%     % quaternion = quaternion / norm(quaternion);
% 
%     % Update the pose plot with the position and quaternion orientation
%     poseplot(q, position);
% 
%     % Pause to control the animation speed (adjust as needed)
%     pause(0.01);
% end
