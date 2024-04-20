clear;
close all;
clc;
% create a driving scenario
scenario = drivingScenario;
roadCenters = [0 0 0; 50 0 0];
road(scenario,roadCenters);
% create an ego vehicle with waypoints and speed
egoVehicle = vehicle(scenario,'ClassID',1,'Position',[5 0 0]);
waypoints = [5 0 0; 45 0 0];
speed = 30;
smoothTrajectory(egoVehicle,waypoints,speed)
% create radar and camera sensors
radar = drivingRadarDataGenerator('MountingLocation',[0 0 0]);
camera = visionDetectionGenerator('SensorLocation',[0 0],'Yaw',-180);
% create a car actor in front of the egoVehicle
car1 = struct('ActorID',1,'Position',[20 0 0],'Velocity', [30 0 0]);
car = vehicle(scenario, 'ClassID',1,'Position',[20 0 0]);
path = [20 0 0; 55 0 0];
smoothTrajectory(car, path, speed);
% obtain detection data
dets = camera(car1, 1.0);
% open the scenario in drivingScenarioDesigner
drivingScenarioDesigner(scenario, {radar, camera});