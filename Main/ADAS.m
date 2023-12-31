function [allData, scenario, sensors,spd] = ADAS(spd)
%Senaryo - Returns sensor detections
%    allData = Senaryo returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = Senaryo optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.13 (R2022b) and Automated Driving Toolbox 3.6 (R2022b).
% Generated on: 26-Sep-2023 08:32:11
spd1=spd
% Create the drivingScenario object and ego car
[scenario, egoVehicle,spd1] = createDrivingScenario(spd1);

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;

    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    insMeas = {};
    isValidTime = false(1, numSensors);

    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        [objectDets, isValidTime(sensorIndex)] = sensor(poses, time);
        numObjects = length(objectDets);
        objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
    end

    % Aggregate all detections into a structure for later use
    if any(isValidTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end
save("sensorData","allData");
% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0 0], ...
    'Yaw', -90.1010506527452, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 3, ...
    'SensorLocation', [0 0], ...
    'Yaw', 90.4441444331119, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'SensorLocation', [0 0], ...
    'Yaw', 0.525634606457613, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{5} = drivingRadarDataGenerator('SensorIndex', 5, ...
    'MountingLocation', [0 0 0.2], ...
    'MountingAngles', [0.767308315089695 0 0], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [90 5], ...
    'Profiles', profiles);
numSensors = 5;

function [scenario, egoVehicle,spd1] = createDrivingScenario(spd1)
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [-20.4 26.2 0;
    39.5 16.2 0;
    59 -20.1 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Dashed')];
laneSpecification = lanespec(3, 'Width', 6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-18.5 26.6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [-18.5 26.6 0;
    2.6 28.3 0;
    23.9 24.4 0;
    39.7 16.4 0;
    54.1 -4.6 0;
    58.8 -19.7 0];

speed = [30;spd1;spd1+30;spd1+30;spd1+30;spd1+30];



trajectory(egoVehicle, waypoints, speed);


% Add the non-ego actors
truck = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 8.2, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [-6.3 24.5 0], ...
    'Mesh', driving.scenario.truckMesh, ...
    'Name', 'Truck');
waypoints = [-6.3 24.5 0;
    13.2 26.6 0;
    28.7 22.6 0;
    54.2 -5.1 0;
    59 -19.7 0];
speed = [20;20;20;20;20];


trajectory(truck, waypoints, speed);

car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-6.6 33.5 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-6.6 33.5 0;
    19.1 28.9 0;
    36.3 23.9 0;
    46.7 11.8 0;
    53 4.1 0;
    61.7 -19 0];
speed = [30;30;30;30;30;30];

plot(scenario)
trajectory(car1, waypoints, speed);

















