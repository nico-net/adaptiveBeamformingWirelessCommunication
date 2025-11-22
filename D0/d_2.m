close all;
clear;
clc;


%% 1 System Parameters 
Pars.fc = 1e9; 
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;

% Signal Parameters
Pars.Fsin = 600; 
Pars.Fsample = 100e3; 
Pars.Ts = 1/Pars.Fsample;
Pars.SnapshotsPerFrame = 500; 
Pars.TsVect = (0:Pars.SnapshotsPerFrame-1)*Pars.Ts;

Pars.SignalDuration = Pars.SnapshotsPerFrame * Pars.Ts;

% Movement Parameters
Pars.TotalTime_s = 20;       
Pars.PhysicsStep = 0.05;
Pars.numFrame = ceil(Pars.TotalTime_s / Pars.PhysicsStep);
dt = Pars.PhysicsStep;

% Vehicles Parameters
Pars.speed1_kmh = 7;
Pars.speed2_kmh = 5;
Pars.v1_ms = Pars.speed1_kmh * (1000/3600);
Pars.v2_ms = Pars.speed2_kmh * (1000/3600);

% Waveforms
waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect).'; 
waveform2 = sign(sin(2*pi*Pars.Fsin*Pars.TsVect + pi/3)).';


%% 2 Geometry
N_Elements = 16; 
Geometry.BSarray = phased.ULA('NumElements', N_Elements, ...
    'ElementSpacing', Pars.lambda/2);

Geometry.BSPos = [0; 0; 0]; 
Geometry.V1Pos = [20; -10; 0]; 
Geometry.V2Pos = [20; 20; 0]; 

dir1 = [1; 0.5; 0]; dir1 = dir1/norm(dir1);
dir2 = [0.5; -1; 0]; dir2 = dir2/norm(dir2);
Geometry.V1Vel = Pars.v1_ms * dir1;
Geometry.V2Vel = Pars.v2_ms * dir2;

steeringVec = phased.SteeringVector('SensorArray',Geometry.BSarray, ...
    'PropagationSpeed',Pars.c);


%% 3 UE STRUCT
UE_template = struct( ...
    'id',               [], ...   
    'active',           false, ...
    'pos',              [NaN; NaN; 0], ... 
    'vel',              [0; 0; 0], ...     
    'DOA',              [0; 0], ...      
    'DOA_prev',         [0; 0], ...      
    'DOA_variation',    0, ...           
    'SNR',              NaN, ...         
    'power',            NaN, ...                 
    'lastUpdate',       0, ...           
    'nextUpdate',       0, ...           
    'weights',          ones(N_Elements, 1), ... 
    'output',           [], ...          
    'state',            'idle', ...      
    'lifetime',         0 ...            
);

numUsers = 2;
UE = repmat(UE_template, numUsers, 1);

% Array Inizialization
[az1, el1] = rangeangle(Geometry.V1Pos, Geometry.BSPos);
[az2, el2] = rangeangle(Geometry.V2Pos, Geometry.BSPos);
if isempty(el1), el1=0; end
if isempty(el2), el2=0; end

UE(1).id = 1; UE(1).active = true; UE(1).state = 'newly_detected';
UE(1).DOA = [double(az1); double(el1)]; 

UE(2).id = 2; UE(2).active = true; UE(2).state = 'newly_detected';
UE(2).DOA = [double(az2); double(el2)];


%% 4 Objects for the Estimation
freeSpace = phased.FreeSpace('OperatingFrequency',Pars.fc, ...
    'SampleRate',Pars.Fsample, 'TwoWayPropagation', false);
collector = phased.Collector('Sensor',Geometry.BSarray, ...
    'OperatingFrequency',Pars.fc);
doaEstimator = phased.RootMUSICEstimator(...
    'SensorArray',Geometry.BSarray, ...
    'OperatingFrequency',Pars.fc, ...
    'NumSignalsSource','Property', ...
    'NumSignals', numUsers); 


%% 5 Visualization
figure('Position',[50 100 1400 500]);

% Pannel 1: Geometry
subplot(1,3,1); 
hPlotGeo = plot(0,0,'k^','MarkerSize',10,'LineWidth',2,'DisplayName','BS'); hold on;
hPlotV1 = plot(0,0,'bo','MarkerFaceColor','b','DisplayName','UE 1 (Target)');
hPlotV2 = plot(0,0,'rs','MarkerFaceColor','r','DisplayName','UE 2 (Interferer)');
hTrail1 = animatedline('Color','b','LineStyle',':');
hTrail2 = animatedline('Color','r','LineStyle',':');
axis([-10 100 -20 80]); grid on; title('Scenario V2X'); legend('Location','northwest');

% Pannel 2: Pattern UE 1 (Blu)
subplot(1,3,2); 
hPlotPat1 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'b'); hold on;

hLineV1_p1 = polarplot([0 0], [0 -40], 'b--'); 
% hLineV2_p1 = polarplot([0 0], [0 -40], 'r--');
title('Beam UE 1 (Target: BLUE)'); rlim([-40 0]);

% Pannel 3: Pattern UE 2 (Red)
subplot(1,3,3); 
hPlotPat2 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'r'); hold on;

% hLineV1_p2 = polarplot([0 0], [0 -40], 'b--'); 
hLineV2_p2 = polarplot([0 0], [0 -40], 'r--');
title('Beam UE 2 (Target: RED)'); rlim([-40 0]);


%% 6 LOOP
for currentFrame = 1:Pars.numFrame
    
    % MOVEMENT PHYSICS
    Geometry.V1Pos = Geometry.V1Pos + Geometry.V1Vel * dt;
    Geometry.V2Pos = Geometry.V2Pos + Geometry.V2Vel * dt;
    
    UE(1).pos = Geometry.V1Pos(:); UE(1).vel = Geometry.V1Vel(:);
    UE(2).pos = Geometry.V2Pos(:); UE(2).vel = Geometry.V2Vel(:);
    
    if Geometry.V1Pos(1) > 120, break; end

    % SIGNALS
    sig1 = freeSpace(waveform1, Geometry.V1Pos, Geometry.BSPos, Geometry.V1Vel, [0;0;0]);
    sig2 = freeSpace(waveform2, Geometry.V2Pos, Geometry.BSPos, Geometry.V2Vel, [0;0;0]);
    
    ang1 = rangeangle(Geometry.V1Pos, Geometry.BSPos);
    ang2 = rangeangle(Geometry.V2Pos, Geometry.BSPos);
    ang_matrix = [ang1, ang2];
    
    if size(ang_matrix, 1) == 1
        ang_matrix = [ang_matrix; zeros(1, size(ang_matrix, 2))];
    end
    
    ang_matrix(1,:) = mod(ang_matrix(1,:) + 180, 360) - 180;
    ang_matrix(2,:) = max(min(ang_matrix(2,:), 90), -90);
    
    rx_clean = collector([sig1, sig2], ang_matrix);
    noise = 0.01 * (randn(size(rx_clean)) + 1i*randn(size(rx_clean)));
    rx_signal = rx_clean + noise;
    
    % PROCESSING
    estimated_doas = doaEstimator(rx_signal); 
    
    est_used = [false, false];
    for k = 1:numUsers
        UE(k).DOA_prev = UE(k).DOA(:);
        prev_az = UE(k).DOA(1);
        min_dist = Inf; best_idx = -1;
        
        for j = 1:length(estimated_doas)
            dist = abs(estimated_doas(j) - prev_az);
            if dist > 180, dist = 360 - dist; end
            if dist < min_dist && ~est_used(j)
                min_dist = dist;
                best_idx = j;
            end
        end
        
        if best_idx ~= -1
            UE(k).DOA = [estimated_doas(best_idx); 0]; 
            est_used(best_idx) = true;
            UE(k).state = 'tracking';
        else
            UE(k).state = 'lost';
        end
        UE(k).DOA_variation = norm(UE(k).DOA(:)' - UE(k).DOA_prev(:));
        UE(k).lifetime = UE(k).lifetime + 1;
    end
    
    % BEAMFORMING MVDR
    Rxx = (rx_signal' * rx_signal) / Pars.SnapshotsPerFrame;
    Rxx = Rxx + 0.001 * eye(size(Rxx)); 
    R_inv = inv(Rxx);
    
    for k = 1:numUsers
        target_az = UE(k).DOA(1);
        target_az = mod(target_az + 180, 360) - 180;
        
        sv = steeringVec(Pars.fc, [target_az; 0]);
        
        num = R_inv * sv;
        den = sv' * R_inv * sv;
        UE(k).weights = num / den;
        
        UE(k).output = UE(k).weights' * rx_signal.';
    end
    
    % UPDATE THE VISUALIZATION
    set(hPlotV1, 'XData', Geometry.V1Pos(1), 'YData', Geometry.V1Pos(2));
    set(hPlotV2, 'XData', Geometry.V2Pos(1), 'YData', Geometry.V2Pos(2));
    addpoints(hTrail1, Geometry.V1Pos(1), Geometry.V1Pos(2));
    addpoints(hTrail2, Geometry.V2Pos(1), Geometry.V2Pos(2));
    
    if mod(currentFrame, 2) == 0
        % Scan Axis
        scan_az = -90:0.5:90;
        sv_scan = steeringVec(Pars.fc, [scan_az; zeros(1, length(scan_az))]);
        
        % Real Angles
        t_az1 = ang_matrix(1,1);
        t_az2 = ang_matrix(1,2);
        
        % PATTERN UE 1
        pat1 = abs(UE(1).weights' * sv_scan).^2;
        pat1_dB = 10*log10(pat1 + eps);
        pat1_dB = pat1_dB - max(pat1_dB);
        
        set(hPlotPat1, 'ThetaData', deg2rad(scan_az), 'RData', pat1_dB);

        set(hLineV1_p1, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
        % set(hLineV2_p1, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);

        % PATTERN UE 2
        pat2 = abs(UE(2).weights' * sv_scan).^2;
        pat2_dB = 10*log10(pat2 + eps);
        pat2_dB = pat2_dB - max(pat2_dB);
        
        set(hPlotPat2, 'ThetaData', deg2rad(scan_az), 'RData', pat2_dB);

        % set(hLineV1_p2, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
        set(hLineV2_p2, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);
        
        drawnow limitrate;
        pause(Pars.PhysicsStep / 2);
    end
end