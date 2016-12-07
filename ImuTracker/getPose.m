loadData = 1;
if(loadData == 1)
    
    % load GPS data
    % altitude latitude longitude
    fileID = fopen('gps_data_1123.txt','r');
    formatSpec = '%f %f %f\n';
    fileHandler = fscanf(fileID,formatSpec);
    fclose(fileID);
    [m n] = size(fileHandler);
    GPSRawData = reshape(fileHandler, 3, m/3);
    
    % load IMU data
    % server_time angley anglex anglez
    fileID = fopen('imu_data_1123.txt','r');
    formatSpec = '%f %f %f %f\n';
    fileHandler = fscanf(fileID,formatSpec);
    fclose(fileID);
    [m n] = size(fileHandler);
    IMURawData = reshape(fileHandler, 4, m/4);
    
end

timeStamp = IMURawData(1, :);
% anglex angley anglez
IMUData = IMURawData(3, :);
IMUData = [IMUData; IMURawData(2, :)];
IMUData = [IMUData; IMURawData(4, :)];
% longitude latitude altitude
GPSData = GPSRawData(3,:);
GPSData = [GPSData; GPSRawData(2,:)];
GPSData = [GPSData; GPSRawData(1,:)];

[n, numOfFrame] = size(GPSData);

% earth radius (approx.) in meters
er = 6378137.0;
% compute scale from first lat value
scale = cos(GPSData(2,1) * pi / 180.);

i = 1;

MapPoseAll = [0 0];
t0 = [-1 -1];

plotScale = 1; % scale for plot

for i = 50000:10:numOfFrame

lon = GPSData(1,i);
lat = GPSData(2,i);
alt = GPSData(3,i);
tx = scale * lon * pi * er / 180.0;
ty = scale * er * log(tan((90.0 + lat) * pi / 360.0));
tz = alt;

t = [tx ty tz]';
if(t0(1) < 0)
    t0 = t;
end

%MapPose = [tx - t0(1), ty - t0(2)];
MapPose = [tx, ty];
MapPoseAll = [MapPoseAll; MapPose];

rx = IMUData(1,i);
ry = IMUData(2,i);
rz = IMUData(3,i);

Rx = vrrotvec2mat([1, 0, 0, rx/180*pi]);
Ry = vrrotvec2mat([0, 1, 0, ry/180*pi]);
Rz = vrrotvec2mat([0, 0, 1, rz/180*pi]);
%R = Rx*Ry*Rz;
%R = Rz*Ry*Rx;
R = Rz;

Pose = [R ; 0 0 1];
Pose = [Pose [t; 1]];

%XAxis = [1, 0 0]';
step = 0.1*(2^(plotScale-1));
startP = 0.0;
endP = 0.0 + 10*0.1*(2^(plotScale-1));
XAxis = zeros(3, 11);
XAxis(1,:) = [0.0:step:endP];
XAxis_World = R'*XAxis + kron(t, ones(1,11));
YAxis = zeros(3, 11);
YAxis(2,:) = [0.0:step:endP];
YAxis_World = R'*YAxis + kron(t, ones(1,11));

MapXY = MapPoseAll(2:end,:);
pause(0.01)
clf;
plot(MapXY(:,1), MapXY(:,2));
hold on;
plot(XAxis_World(1,:), XAxis_World(2,:), 'b-');
hold on;
plot(YAxis_World(1,:), YAxis_World(2,:), 'r-');

plotScaleTemp = ceil(log2(abs(t(1) - t0(1))));
plotScale = max(plotScaleTemp, ceil(log2(abs(t(2) - t0(2)))));
if(plotScale <= 0) 
    plotScale = 1;
end
axis([t0(1) - 2^plotScale, t0(1) + 2^plotScale, t0(2) - 2^plotScale, t0(2) + 2^plotScale]);
end
%MapXY = MapPoseAll(2:end,:);
%plot(MapXY(:,1), MapXY(:,2));
