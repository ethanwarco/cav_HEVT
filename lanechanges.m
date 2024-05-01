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
otherDirection = [1 0 0];

vehicles = {egoVehicle};
vehicles{2} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [2 2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');



% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors = {};
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.5 -0.9], ...
    'Yaw', 90, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([300 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [1.5 0.9], ...
    'Yaw', -90, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([300 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
    'SensorLocation', [0.95 0], ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([300 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'SensorLocation', [1.9 0], ...
    'Yaw', 180, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([300 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
directions = {egoDirection otherDirection};
velocities = [5 3];

plot(scenario)

dt = 1;
time = 19 / dt;
laneChangeDistance = 10;
changingLane = -1;
inLane = false;
laneVelocity = 0;
for n = 1:time
    % collect measurement data and find distance from vehicle in y
    % direction, vehicles velocity, and lane data
    measurements = getMeasurements(n * dt, sensors, egoVehicle);

    % updates vehicle positions based off of their velocities
    updatePositions(vehicles, velocities, directions, dt);
    
    % when not in lane, check if there is room to switch lanes, and when
    % in lane, try to match the velocity of the other cars in your lane
    % the middle block keeps track of how long the car has been changing
    % lane
    if ~inLane & changingLane == -1
        % get the cars in the left lane (the one we want to turn into)
        otherLaneMeasurements = getLaneMeasurements(measurements, -1);
        % get the cars in the current lane
        laneMeasurements = getLaneMeasurements(measurements, 0);

        % adjust the cars speed according to the average speed
        velocities(1) = velocities(1) * .8 + averageVelocity(laneMeasurements) * .2;

        % determines if car has room to turn into the left lane
        canChange = canChangeLane(otherLaneMeasurements, 0);
        if canChange
            changingLane = 20.1557 / velocities(1);
            % merges into lane at 80 degree angle
            directions{1} = [0.9848 0.1736 0];

            % calculates the average speed in the lane you are turning into
            laneVelocity = averageVelocity(otherLaneMeasurements);
        end
    elseif changingLane > 0
        if changingLane > 0
            changingLane = changingLane - 1;
        end
        if changingLane <= 0
            directions{1} = [1 0 0];
            inLane = true;
        end
        velocities(1) = velocities(1) * .5 + laneVelocity * .5;
    else
        % adjust the cars speed according to the average speed
        laneMeasurements = getLaneMeasurements(measurements, 0);
        velocities(1) = velocities(1) * .8 + averageVelocity(laneMeasurements) * .2;
    end

    refreshdata
    drawnow
    plot(scenario)
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
% this assumes the ego car is going straight in the positive x direction
function laneMeasurements = getLaneMeasurements(measurements, lane)
    laneMeasurements = {};

    index = 1;
    ylow = 2 + (abs(lane) - 1) * 4;
    yhigh = 2 + abs(lane) * 4;
    for i = 1:length(measurements)
        ydiff = measurements{i}(2);
        % last conditionbasically checks if the ydiff (which is opposite to the lane in 
        % direction) is the same sign as laneMeasurements{index} = measurements{i};
        if ylow < ydiff & ydiff < yhigh & sign(lane) ~= sign(ydiff)
            laneMeasurements{index} = measurements{i};
            index = index + 1;
        end
    end
end

% returns the distance from the closest possible lane change in front of 
% the egocar and behind the egocar
function closestDistances = getLaneChangeDistances(measurements)
    closestFront = 99999;
    closestBack = 99999;
    for i = 1:length(measurements)
        % x is the distance from the egocar to a point where it could
        % potentially change lanes
        x = measurements{i}(1) + laneChangeDistance;
        % checks if this opening in the road is a possible point to change lanes
        if canChangeLane(measurements, x)
            % checks whether it is closer to a previous closesFront or
            % closestBack
            if x < closestFront & sign(closestFront) == sign(1)
                closestFront = x;
            elseif x > closestBack & sign(closestFront) == sign(-1)
                closestBack = x;
            end
        end
    end
    closestDistances = [closestBack closestFront]
end

% returns whether the car can change lanes when it is dx away from its
% current position, with measurements being the set of measurements of cars
% in the desired lane
function canChange = canChangeLane(measurements, dx)
    canChange = true;
    for i = 1:length(measurements)
        distance = measurements{i}(1) - dx;
        if abs(distance) < laneChangeDistance
            canChange = false;
        end
    end
end

% calculates the average velocity of the cars in the measurements including
% the velocity of the egocar
function avg = averageVelocity(measurements)
    avg = velocities(1);
    for i = 1:length(laneMeasurements)
        avg = avg + laneMeasurements{i}(4);
    end
    avg = avg / (length(laneMeasurements) + 1);
end

end