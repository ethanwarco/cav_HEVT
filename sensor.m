classdef Sensor
    properties
        Range double
        Angle double
    end
    
    methods
        function obj = Sensor(range, angle)
            obj.Range = range;
            obj.Angle = angle;
        end
        
        function detected = detectVehicles(obj, vehicles, av)
            detected = [];
            for i = 1:numel(vehicles)
                if i ~= av
                    distance = vehicles(i).Position - vehicles(av).Position;
                    if abs(distance) <= obj.Range && abs(vehicles(i).Lane - vehicles(av).Lane) <= tan(deg2rad(obj.Angle))
                        detected = [detected, i];
                    end
                end
            end
        end
    end
end
