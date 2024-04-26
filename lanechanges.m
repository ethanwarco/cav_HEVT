function lanechanges

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [0 0 0; 100 0 0];
barrierCenters = [100 0 0; 100 -4 0];
laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'Width', 0.61, ...
    'Height', 0.81);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2 -2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
egoDirection = [1 0 0];

% Add the non-ego actors
otherVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2 2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
otherDirection = [1 0 0];



% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
ll = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0.9], ...
    'Yaw', 135, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'ActorProfiles', profiles);
ul = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.95 0.9], ...
    'Yaw', 45, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'ActorProfiles', profiles);
back = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'MaxRange', 100, ...
    'DetectorOutput', 'Objects only', ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'ActorProfiles', profiles);
forward = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 100, ...
    'DetectorOutput', 'Objects only', ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors = {ll ul back forward};
vehicles = {egoVehicle otherVehicle};
directions = {egoDirection otherDirection};
velocities = [5 3];

plot(scenario)

dt = 1;
time = 19 / dt;
laneChangeDistance = 10;
timeToSwitchLanes = 3;
for n = 1:time
    % collect measurement data and find distance from vehicle in y
    % direction, vehicles velocity, and lane data
    measurements = getMeasurements(n * dt, sensors, egoVehicle);

    % updates vehicle positions based off of their velocities
    updatePositions(vehicles, velocities, directions, dt);
    
    % get the cars in the left lane (the one we want to turn into)
    otherLaneMeasurements = getLaneMeasurements(measurements, -1);

    refreshdata
    drawnow
end




% helper functions

% returns all sensor measurements aggregated at a certain time
function measurements = getMeasurements(time, sensors, egoVehicle)
    % detections array
    dets = {};
    measurements = {};
    for i = 1:length(sensors)
        sensor = sensors{i};
        detections = sensor(targetPoses(egoVehicle), time);
        dets{i} = detections;
    end

    index = 1;
    for i = 1:length(dets)
            sensorDets = dets{i};
        for j = 1:length(sensorDets)
            measure = sensorDets{j, 1}.Measurement;
            measurements{index} = measure;
            index = index + 1;
        end
    end
end


% updates vehicles positions
function updated = updatePositions(vehicles, velocities, directions, dt)
    for i = 1:length(vehicles)
        direction = directions{i};
        velocity = velocities(i);
        vehicle = vehicles{i};
        vehicle.Position = [(direction(1) * velocity * dt + vehicle.Position(1))
                           (direction(2) * velocity * dt + vehicle.Position(2))
                           (direction(3) * velocity * dt + vehicle.Position(3))];
    end
    updated = true;
end

% returns the measurements for vehicles in a specific lane, where lane is
% the distance in the y direction, so the first right lane is 1, the second
% is 2, the first left lane is -1, the second is -2, etc
function laneMeasurements = getLaneMeasurements(measurements, lane)
    laneMeasurements = {};

    index = 1;
    ylow = 2 + (abs(lane) - 1) * 4;
    yhigh = 2 + abs(lane) * 4;
    for i = 1:length(measurements)
        ydiff = measurements{i}(2);
        if ylow < ydiff & ydiff < yhigh & sign(lane) ~= sign(ydiff) % basically checks if the ydiff (which is opposite to the lane in direction) is the same sign as lane222222222222222222222222222
            laneMeasurements{index} = measurements{i};
        end
    end
end


end