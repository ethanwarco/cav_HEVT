% Main Simulation Function
function [allData, scenario, sensors] = cavfirstdraft()
    [scenario, egoVehicle, roadCenters, laneWidth] = createDrivingScenario();
    [sensors, numSensors] = createSensors(scenario);
    allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {});

    % Simulation loop
    running = true;
    while running
        poses = targetPoses(egoVehicle);
        time = scenario.SimulationTime;
        objectDetections = processSensorData(sensors, numSensors, poses, time, egoVehicle.Position);

        if ~isempty(objectDetections)
            [changeLane, targetLane] = decideLaneChange(objectDetections, egoVehicle, scenario, roadCenters, laneWidth);
            if changeLane
                egoVehicle = executeLaneChange(egoVehicle, targetLane, scenario);
            end
            allData(end+1) = struct('Time', time, 'ActorPoses', poses, 'ObjectDetections', objectDetections);
        end

        running = advance(scenario);
    end

    restart(scenario);
    for sensorIndex = 1:numSensors
        release(sensors{sensorIndex});
    end
end


% Function to process sensor data
function objectDetections = processSensorData(sensors, numSensors, poses, time, egoPosition)
    objectDetections = [];
    isValidTime = false(1, numSensors);
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        [detections, numDetections, isValid] = sensor(poses, time);
        if isValid && numDetections > 0  % Check that detections are valid and there are some detections
            detections = filterDetectionsByRelevance(detections, egoPosition);
            objectDetections = [objectDetections; detections(1:min(numDetections, length(detections)))];  % Safely access detections
            isValidTime(sensorIndex) = true;
        end
    end
end

% Correction in the `filterDetectionsByRelevance` function to use ego vehicle position properly:
function detections = filterDetectionsByRelevance(detections, egoPosition)
    filteredDetections = [];
    if ~isempty(detections) && isstruct(detections) % Ensure detections are non-empty and structured
        for i = 1:length(detections)
            if isfield(detections(i), 'Measurement') % Check if 'Measurement' field exists
                relPos = detections(i).Measurement(1:2); % Relative position from detection
                dist = norm(relPos - egoPosition(1:2)); % Calculate distance based on ego position
                if dist < 100 % Example threshold for filtering
                    filteredDetections = [filteredDetections, detections(i)];
                end
            end
        end
    end
    detections = filteredDetections;
end

function lane = getLane(vehicle, roadCenters, laneWidth)
    % This function assumes roadCenters is a matrix with rows as different points along the road
    % and laneWidth is the width of each lane.

    % Find the closest road segment to the vehicle
    distances = sqrt(sum((roadCenters - vehicle.Position(1:2)).^2, 2));
    [~, closestIndex] = min(distances);
    
    % Assuming a straight road segment for simplicity
    if closestIndex < size(roadCenters, 1)
        % Get the direction of the road segment
        directionVector = roadCenters(closestIndex + 1, :) - roadCenters(closestIndex, :);
        vehicleVector = vehicle.Position(1:2) - roadCenters(closestIndex, :);
        
        % Project vehicle position on the road direction vector
        projectionLength = dot(vehicleVector, directionVector) / norm(directionVector);
        projectedPoint = roadCenters(closestIndex, :) + (projectionLength * directionVector / norm(directionVector));
        
        % Determine the lateral position relative to the road centerline
        lateralVector = vehicle.Position(1:2) - projectedPoint;
        lateralDistance = norm(lateralVector);
        
        % Estimate the lane index
        lane = floor(lateralDistance / laneWidth) + 1;
    else
        lane = 1; % Default to lane 1 if at the end of the road array
    end
end


function [changeLane, targetLane] = decideLaneChange(detections, vehicle, scenario, roadCenters, laneWidth)
    currentLane = getLane(vehicle, roadCenters, laneWidth);
    changeLane = false;
    targetLane = currentLane;
    if not(isempty(detections))
        changeLane = true;
        targetLane = currentLane + 1; % Logic to decide lane change
    end
end



function vehicle = executeLaneChange(vehicle, targetLane, scenario)
    % Placeholder function to execute a lane change
    % This should modify the vehicle's trajectory or state
end % End this function

% Ensure all other function definitions are also closed with 'end'
function [sensors, numSensors] = createSensors(scenario)
    % Function to create sensors
    % Assign into each sensor the physical and radar profiles for all actors
    profiles = actorProfiles(scenario);

    % Initializing sensors array
    sensors = cell(4, 1);  % Adjust the size based on the number of sensors you have

    % Define each sensor
    sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
        'SensorLocation', [3.7 0], ...
        'Yaw', -10.304846468766, ...
        'MaxRange', 100, ...
        'DetectorOutput', 'Objects only', ...
        'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
        'ActorProfiles', profiles);

    sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
        'SensorLocation', [2.8 0.9], ...
        'Yaw', 90, ...
        'MaxRange', 50, ...
        'DetectorOutput', 'Objects only', ...
        'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
        'ActorProfiles', profiles);

    sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
        'SensorLocation', [2.8 -0.9], ...
        'Yaw', -92.4486565838244, ...
        'MaxRange', 50, ...
        'DetectorOutput', 'Objects only', ...
        'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
        'ActorProfiles', profiles);

    sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
        'SensorLocation', [-1 0], ...
        'Yaw', 169.024344827187, ...
        'MaxRange', 100, ...
        'DetectorOutput', 'Objects only', ...
        'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
        'ActorProfiles', profiles);

    % Assign the number of sensors
    numSensors = 4;
end

function [scenario, egoVehicle, roadCenters, laneWidth] = createDrivingScenario
    % Construct a drivingScenario object.
    % Construct a drivingScenario object.
    scenario = drivingScenario;

    % Add all road segments
    roadCenters = [33.78 8.91 0;
        -5.38 8.61 0];
    laneSpecification = lanespec(3);
    laneWidth = 3.5;
    road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

    roadCenters = [16.14 25.8 0;
        16.79 -18.99 0];
    road(scenario, roadCenters, 'Name', 'Road1');

    roadCenters = [33.78, 8.91;
        -5.38, 8.61];
    roadWidth = 3.5;
    laneWidth = 3.5;
    road(scenario, roadCenters, roadWidth, 'Name', 'Road2');
    

    % Add the ego vehicle
    egoVehicle = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [5.97 4.89 0.01], ...
        'Mesh', driving.scenario.carMesh, ...
        'PlotColor', [162 20 47] / 255, ...
        'Name', 'Car');
    waypoints = [5.97 4.89 0.01;
        12.43 5.13 0.01;
        12.99 5.19 0.01;
        13.75 5.19 0.01;
        24.05 5.4 0.01;
        26.81 6.72 0.01];
    speed = [30;30;30;30;30;30];
    trajectory(egoVehicle, waypoints, speed);

    % Add the non-ego actors
    car1 = vehicle(scenario, ...
        'ClassID', 1, ...
        'Position', [20.39 9.16 0.01], ...
        'Mesh', driving.scenario.carMesh, ...
        'PlotColor', [0 255 255] / 255, ...
        'Name', 'Car1');
    waypoints = [20.39 9.16 0.01;
        9.46 8.17 0.01];
    speed = [20;20];
    trajectory(car1, waypoints, speed);

    truck = vehicle(scenario, ...
        'ClassID', 2, ...
        'Length', 8.2, ...
        'Width', 2.5, ...
        'Height', 3.5, ...
        'Position', [15.61 -4.61 0.01], ...
        'RearOverhang', 1, ...
        'FrontOverhang', 0.9, ...
        'Mesh', driving.scenario.truckMesh, ...
        'PlotColor', [0 255 255] / 255, ...
        'Name', 'Truck');
    waypoints = [15.61 -4.61 0.01;
        10.22 -5.8 0.01;
        4.41 -5.94 0.01;
        0.95 -6.21 0.01];
    speed = [-5;-5;-5;-5];
    trajectory(truck, waypoints, speed);
    
    % Add pedestrian actor
    actor(scenario, ...
        'ClassID', 4, ...
        'Length', 0.24, ...
        'Width', 0.45, ...
        'Height', 1.7, ...
        'Position', [19 15.22 0.01], ...
        'RCSPattern', [-8 -8;-8 -8], ...
        'Mesh', driving.scenario.pedestrianMesh, ...
        'Name', 'Pedestrian');

    plot(scenario,'Centerline','on','RoadCenters','on');
end
