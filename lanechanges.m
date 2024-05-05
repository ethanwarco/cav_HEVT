function lanechanges

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [0 0 0; 180 0 0];
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
    'Position', [20 -2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
egoDirection = [1 0 0];
otherDirection = [1 0 0];

vehicles = {egoVehicle};
vehicles{2} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [20 2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
vehicles{3} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [40 -2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
vehicles{4} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [10 -2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
vehicles{5} = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [45 2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');




% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors = {};
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [1.5 -0.9], ...
    'MaxRange', 99999, ...
    'Yaw', 90, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([200 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [1.5 0.9], ...
    'MaxRange', 99999, ...
    'Yaw', -90, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([200 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
    'MaxRange', 99999, ...
    'SensorLocation', [0.95 0], ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([200 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'MaxRange', 99999, ...
    'SensorLocation', [1.9 0], ...
    'Yaw', 180, ...
    'DetectionProbability', 1, ...
    'FalsePositivesPerImage', 0, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([200 800],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
directions = {egoDirection otherDirection otherDirection otherDirection otherDirection};
velocities = [5 3 5 4 4];



dt = 1;
time = 30 / dt;
laneChangeDistance = 10;
safeDistance = 10;
changingLane = -1;
inLane = false;
laneVelocity = 0;
prevMeasurements = {};
for n = 1:time
    plot(scenario)

    % collect measurement data and find distance from vehicle in y
    % direction, vehicles velocity, and lane data
    measurements = getMeasurements(n, sensors, egoVehicle);

    % updates vehicle positions based off of their velocities
    updatePositions(vehicles, velocities, directions, dt);
    
    % when not in lane, check if there is room to switch lanes, and when
    % in lane, try to match the velocity of the other cars in your lane
    % the middle block keeps track of how long the car has been changing
    % lane
    if ~inLane & changingLane == -1
        % get the cars in the left lane (the one we want to turn into)
        otherLaneMeasurements = getLaneMeasurements(measurements, -1);
        % get the cars in the current lanelin
        laneMeasurements = getLaneMeasurements(measurements, 0);
        % get the direction the car should move to get closer to a possible
        % lane change
        direction = getLaneChangeDirection(otherLaneMeasurements, laneMeasurements);

        % adjust the cars speed according to the average speed
        velocities(1) = velocities(1) * (1 - .4 * dt) + averageVelocity(laneMeasurements) * .4 * dt + .1 * dt * direction * velocities(1);

        % determines if car has room to turn into the left lane
        canChange = canChangeLane(otherLaneMeasurements, 0);
        if canChange
            % calculates the average speed in the lane you are turning into
            laneVelocity = averageVelocity(otherLaneMeasurements)
            
            v0 = velocities(1) * 0.1736; % initial horizontal velocity
            vf = laneVelocity * 0.1736; % final horizontal velocity
            changingLane = getLaneChangeTime(.5, v0, vf) / dt;
            % merges into lane at 80 degree angle
            directions{1} = [0.9848 0.1736 0];
        end
    elseif changingLane > 0
        if changingLane > 0
            changingLane = changingLane - 1;
        end
        if changingLane <= 0
            directions{1} = [1 0 0];
            inLane = true;
        end
        velocities(1) = velocities(1) * (1 - .5 * dt) + laneVelocity * .5 * dt;
    else
        % adjust the cars speed according to the average speed
        laneMeasurements = getLaneMeasurements(measurements, 0);
        % get the direction the car should move in case it is too close to
        % other cars
        direction = getLaneChangeDirection({}, laneMeasurements);
        velocities(1) = velocities(1) * (1 - .2 * dt) + averageVelocity(laneMeasurements) * .2 * dt + .1 * dt * direction * velocities(1);
    end
    
    prevMeasurements = measurements;
    refreshdata
    drawnow
end




% helper functions

% returns all sensor measurements aggregated at a certain time
% applies rotation matrix to x, y, vx and vy values based on the road angle
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
            measurement = sensorDets{j, 1}.Measurement;

            % calculates vehicles velocity and stores it in measurement at
            % index 4
            prevMeasurement = getPreviousMeasurement(measurement);
            
            measurement(4) = velocities(1);
            if prevMeasurement ~= 99999
                measurement(4) = measurement(4) + ((measurement(1) - prevMeasurement(1)) / dt);
            end

            measurements{index} = measurement;
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
        % last condition basically checks if the ydiff (which is opposite to the lane in 
        % direction) is the same sign as laneMeasurements{index} = measurements{i};
        if ylow < ydiff & ydiff < yhigh & sign(lane) ~= sign(ydiff)
            laneMeasurements{index} = measurements{i};
            index = index + 1;
        end
    end
end


function direction = getLaneChangeDirection(otherLaneMeasurements, inLaneMeasurements)
    % closest front and closest back are the distance from the closest 
    % possible lane change in front of the egocar and behind the egocar
    closestFront = 99999;
    closestBack = -99999;
    for i = 1:length(otherLaneMeasurements)
        % x is the distance from the egocar to a point where it could
        % potentially change lanes
        frontx = otherLaneMeasurements{i}(1) + laneChangeDistance;
        backx = otherLaneMeasurements{i}(1) - laneChangeDistance;

        % checks if this opening in the road at frontx is a possible point to change lanes
        if canChangeLane(otherLaneMeasurements, frontx)
            % checks whether it is closer to a previous closestFront or
            % closestBack
            if frontx < closestFront & sign(frontx) == sign(1)
                closestFront = frontx;
            elseif frontx > closestBack & sign(frontx) == sign(-1)
                closestBack = frontx;
            end
        end
        % checks if this opening in the road at backx is a possible point to change lanes
        if canChangeLane(otherLaneMeasurements, backx)
            % checks whether it is closer to a previous closestFront or
            % closestBack
            if backx < closestFront & sign(backx) == sign(1)
                closestFront = backx;
            elseif backx > closestBack & sign(backx) == sign(-1)
                closestBack = backx;
            end
        end
    end
    
    % if the closestFront distance is larger than the distance of the car
    % in front of the egocar, it is infeasible, and if the closestBack
    % distance is larger than the distance of the car behind it, it is
    % infeasible. If they are both feasible the function will return
    % whichever direction has a closer lane change spot. additionally, if
    % the front vehicle is closer than safeDistance, the forward direction
    % will be considered infeasible
    for i = 1:length(inLaneMeasurements)
        x = inLaneMeasurements{i}(1);
        if sign(x) == sign(1)
            % if too close to another car in lane, go the opposite
            % direction
            if abs(x) < safeDistance
                direction = -1;
                return
            end
            if x < closestFront
                closestFront = 99999;
            end
        else
            if abs(x) < safeDistance
                direction = 1;
                return
            end
            if x > closestBack
                closestBack = -99999;
            end
        end
    end
    
    if closestBack == -99999 & closestFront == 99999
        direction = 0;
    elseif closestBack == -99999
        direction = 1;
    elseif closestFront == 99999
        direction = -1;
    else
        direction = -1 * sign(closestFront + closestBack);
    end
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

% calculates the average velocity of the cars in the measurements. if no
% measurements, it returns the velocity of the egocar
function avg = averageVelocity(measurements)
    avg = 0;
    for i = 1:length(measurements)
        avg = avg + measurements{i}(4);
    end
    
    if isempty(measurements)
        avg = velocities(1);
    else
        avg = avg / length(measurements);
    end
end

% given a parameter measurement, this will return its previous position
% according to the prevMeasurements array. if no position is found, it will
% return 99999
function previous = getPreviousMeasurement(measurement)
    x = measurement(1);
    y = measurement(2);
    previous = 99999;
    for i = 1:length(prevMeasurements)
        otherx = prevMeasurements{i}(1);
        othery = prevMeasurements{i}(2);
        % checks that othery and y are close to each other (the
        % measurements are in the same lane) and that otherx and x are 
        % close (they are the same car)
        if abs(othery - y) < 2 & abs(otherx - x) < 10
            previous = prevMeasurements{i};
            return
        end
    end
end

% estimates the amount of time to change lane given the amount the car is
% adjusting to the lane velocity, the intial velocity, and the final
% velocity
function time = getLaneChangeTime(p, v0, vf)
    L = 4; % lane change distance
    a = vf / (v0 - vf);
    b = a - (p * L / vf);
    time = -1 * log(a * lambertw(exp(b) / a)) / p;
end

end