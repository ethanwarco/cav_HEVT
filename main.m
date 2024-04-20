% Initialize Vehicles
vehicles = [];
for i = 1:10
    % Random parameters: position, speed, lane, isAutonomous
    isAutonomous = false;
    if i == 1
        isAutonomous = true; % Make the first vehicle autonomous
    end
    vehicles = [vehicles, Vehicle(rand * 1000, rand * 30 + 50, randi([1, 3]), isAutonomous)];
end

% Initialize Sensor
sensor = Sensor(150, 30);

% Setup and run Simulation
sim = Simulation(vehicles, sensor);
sim.run();

