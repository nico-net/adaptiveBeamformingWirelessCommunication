close all;
clear;
clc;

%% ============================================================
%%  SCRIPT (DOA note dalla geometria)
%%  - Vehicles in movement
%%  - UE struct extended
%%  - Per-frame estimation of number of UEs
%%  - DOA known from geometry
%%  - LMS beamforming
%%  - Updated visualization layout
%% ============================================================

%% 1) System Parameters
Pars.fc = 1e9;
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;

% Signal parameters
Pars.Fsin = 600;
Pars.Fsample = 100e3;
Pars.Ts = 1/Pars.Fsample;
Pars.SnapshotsPerFrame = 500;
Pars.TsVect = (0:Pars.SnapshotsPerFrame-1)*Pars.Ts;

% Simulation time / physics
Pars.TotalTime_s = 40;
Pars.PhysicsStep = 0.05;
Pars.numFrame = ceil(Pars.TotalTime_s / Pars.PhysicsStep);
dt = Pars.PhysicsStep;

% Noise parameters
Pars.Temp_ant = 293.15; %in Kelvin
Pars.NoiseFactor = 5; %in dB


% Vehicles parameters
Pars.speed1_kmh = 40;
Pars.speed2_kmh = 9;
Pars.v1_ms = Pars.speed1_kmh * (1000/3600);
Pars.v2_ms = Pars.speed2_kmh * (1000/3600);

% Waveforms
waveform1 = exp(1j*(2*pi*(Pars.fc)*Pars.TsVect + pi/6)).';        
waveform2 = exp(1j*(2*pi*(Pars.fc + 1e3)*Pars.TsVect + pi/3)).';       


%SNRs
SINR_UE1 = zeros(Pars.numFrame, 1);
SINR_UE1_dB = zeros(Pars.numFrame, 1);
SINR_UE2 = zeros(Pars.numFrame, 1);
SINR_UE2_dB = zeros(Pars.numFrame, 1);

%% 2) Geometry + Array
N_Elements = 6;   %Number of antennas
Geometry.BSarray = phased.ULA('NumElements', N_Elements, ...
    'ElementSpacing', Pars.lambda/2);

Geometry.BSPos = [0; 0; 0];
Geometry.V2Pos = [40; -20; 0];
Geometry.V1Pos = [20;  60; 0];

dir2 = [0; 0.5; 0]; 
dir1 = [0; -1; 0]; 
dir2 = dir2/norm(dir2);
dir1 = dir1/norm(dir1);
Geometry.V1Vel = Pars.v1_ms * dir1;
Geometry.V2Vel = Pars.v2_ms * dir2;

steeringVec = phased.SteeringVector('SensorArray', Geometry.BSarray, ...
    'PropagationSpeed', Pars.c);

%% 3) UE STRUCT (extended)
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

maxUsers = 2;
UE = repmat(UE_template, maxUsers, 1);
for k = 1:maxUsers
    UE(k).id = k;
end


%% 4) Objects for channel & reception
freeSpace = phased.FreeSpace('OperatingFrequency', Pars.fc, ...
    'SampleRate', Pars.Fsample, ...
    'TwoWayPropagation', false);

collector = phased.Collector('Sensor', Geometry.BSarray, ...
    'OperatingFrequency', Pars.fc);

%% 5) Visualization (Updated Layout from d_3.m)
fig = figure;
set(fig, 'WindowState', 'maximized');
sgtitle('D1 - LMS, friis, DoA')

% Top row: Scenario map
subplot(2,4,[1 2]);
plot(Geometry.BSPos(1), Geometry.BSPos(2), 'w^', ...
    'MarkerSize',10,'LineWidth',2,'DisplayName','BS'); hold on;
hPlotV1 = plot(Geometry.V1Pos(1), Geometry.V1Pos(2), 'bo', ...
    'MarkerFaceColor','b','DisplayName','UE 1');
hPlotV2 = plot(Geometry.V2Pos(1), Geometry.V2Pos(2), 'rs', ...
    'MarkerFaceColor','r','DisplayName','UE 2');
hTrail1 = animatedline('Color','b','LineStyle',':', 'DisplayName', "UE1's trajectory");
hTrail2 = animatedline('Color','r','LineStyle',':', 'DisplayName', "UE2's trajectory");
axis([-40 80 -50 50]); grid on; title('Scenario V2X'); legend('Location','northwest');

% Top row: SINR plot
subplot(2,4,[3 4]);
hSINR_line = plot(NaN, NaN, 'b-', 'LineWidth', 2); hold on;
hSINR_line2 = plot(NaN, NaN, 'r-', 'LineWidth', 2); hold on;
grid on;
xlabel('Time [s]');
ylabel('SINR [dB]');
title('SINR of UE1 and UE2 vs Time');
xlim([0 Pars.TotalTime_s]);
ylim([-10 30]); % Adjust based on expected SINR range

% Bottom row: Beam patterns
subplot(2,4,5);
hPlotPat1 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'b'); hold on;
hLineV1_p1 = polarplot([0 0], [0 -40], 'b--');
title('Beam UE 1'); rlim([-40 0]);

subplot(2,4,6);
hPlotPat2 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'r'); hold on;
hLineV2_p2 = polarplot([0 0], [0 -40], 'r--');
title('Beam UE 2'); rlim([-40 0]);

subplot(2,4,7);
hPlotPatTotal = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'm'); hold on;
hLineV1_total = polarplot([0 0], [0 -40], 'b--');
hLineV2_total = polarplot([0 0], [0 -40], 'r--');
title('Combined Pattern'); rlim([-40 0]);

%% 6) Storage for replay
% Preallocate storage arrays
StoredData.V1Pos = zeros(3, Pars.numFrame);
StoredData.V2Pos = zeros(3, Pars.numFrame);
StoredData.UE1_weights = zeros(N_Elements, Pars.numFrame);
StoredData.UE2_weights = zeros(N_Elements, Pars.numFrame);
StoredData.ang_matrix = zeros(2, 2, Pars.numFrame);
StoredData.validFrames = 0;
StoredData.didUpdate = false(1, Pars.numFrame);


%% 6.1) Noise power computation
%B = 1kHz
% 10log10(kT0) = -174 dBm/Hz with T0 = 293.15 °K
%-30 to convert from dBm to dB
Pars.sigma2 = db2pow(-174 + 10*log10(Pars.Temp_ant/293.15) + 10*log10(1e5)-30 + Pars.NoiseFactor);

%% 7) MAIN LOOP
scan_az = -90:0.5:90;
sv_scan = steeringVec(Pars.fc, [scan_az; zeros(1, numel(scan_az))]);

for currentFrame = 1:Pars.numFrame

    %% (a) Physics: move vehicles
    Geometry.V1Pos = Geometry.V1Pos + Geometry.V1Vel * dt;
    Geometry.V2Pos = Geometry.V2Pos + Geometry.V2Vel * dt;

    UE(1).pos = Geometry.V1Pos(:); UE(1).vel = Geometry.V1Vel(:);
    UE(2).pos = Geometry.V2Pos(:); UE(2).vel = Geometry.V2Vel(:);

    if Geometry.V1Pos(1) > 120
        break;
    end

    %% (b) Channel: received signals at BS
    sig1 = freeSpace(waveform1, Geometry.V1Pos, Geometry.BSPos, Geometry.V1Vel, [0;0;0]);
    sig2 = freeSpace(waveform2, Geometry.V2Pos, Geometry.BSPos, Geometry.V2Vel, [0;0;0]);

    % True angles from geometry
    ang1 = rad2deg(atan2(-Geometry.BSPos(2) + Geometry.V1Pos(2), ...
        -Geometry.BSPos(1) + Geometry.V1Pos(1)));
    ang2 = rad2deg(atan2(-Geometry.BSPos(2) + Geometry.V2Pos(2), ...
        -Geometry.BSPos(1) + Geometry.V2Pos(1)));

    ang_matrix = [ang1, ang2];

    if size(ang_matrix,1) == 1
        ang_matrix = [ang_matrix; zeros(1, size(ang_matrix,2))];
    end

    ang_matrix(1,:) = mod(ang_matrix(1,:) + 180, 360) - 180;
    ang_matrix(2,:) = max(min(ang_matrix(2,:), 90), -90);

    rx_clean = collector([sig1, sig2], ang_matrix);

    %% (c) Add AWGN based on target SNR

    noise = sqrt(Pars.sigma2/2) * (randn(size(rx_clean)) + 1j*randn(size(rx_clean)));
    rx_signal = rx_clean + noise;

    %% (d) Estimate number of UEs via MDL (per frame)
    R = (rx_signal * rx_signal') / size(rx_signal, 2);
    eigvals = sort(real(eig(R)), 'descend');
    eigvals(eigvals < 0) = eps;

    M = size(R,1);
    Nsnap = size(rx_signal,2);
    mdl = zeros(M,1);

    for k = 0:M-1
        gm = geomean(eigvals(k+1:end));
        am = mean(eigvals(k+1:end));
        mdl(k+1) = -Nsnap*(M-k)*log(gm/am) + 0.5*k*(2*M-k)*log(Nsnap);
    end

    [~, idx] = min(mdl);
    numUsers_est = idx-1;
    numUsers_est = min(max(numUsers_est,0), maxUsers);

    %% (e) DOA known from geometry -> update UEs
    for k = 1:maxUsers
        if k > numUsers_est
            UE(k).active = false;
            UE(k).state  = 'lost';
            continue;
        end

        UE(k).DOA    = [ang_matrix(1,k); 0];  % azimuth + elevation=0
        UE(k).active = true;
        UE(k).state  = 'tracking';

        UE(k).DOA_variation = norm(UE(k).DOA(:) - UE(k).DOA_prev(:));
        UE(k).lifetime = UE(k).lifetime + 1;
    end

    %% (f) LMS Adaptive Beamforming
    didUpdateNow = false;
    if UE(k).DOA_variation > 2
        didUpdateNow = true;

        UE(k).DOA_prev = UE(k).DOA(:);
        [N, L] = size(rx_signal);
        
        for k = 1:maxUsers
            UE(k).weights = zeros(L,1);    % spatial weights init
        end
        
        for k = 1:maxUsers
            if ~UE(k).active
                continue; 
            end
        
            d_desired = (k==1)*waveform1 + (k~=1)*waveform2;
        
            w = UE(k).weights;
            y_out = zeros(N,1);
            e_out = zeros(N,1);
        
            for n = 1:N
                x = rx_signal(n,:).';
                mu = trace(x*x');   % adaptive weights
                y_out(n) = w' * x;             % beamformed output
                e_out(n) = -d_desired(n) + y_out(n);  % error
                w = w - mu * x * conj(e_out(n));  %update of the weights
            end
        
            UE(k).weights = w;
            UE(k).output  = y_out.';
            UE(k).power   = mean(abs(y_out).^2);
        end
    end

StoredData.didUpdate(currentFrame) = didUpdateNow;

    %% (g) SINR Calculation for UE1
    if UE(1).active && UE(2).active
        rx_sig1 = collector(sig1, ang_matrix(:,1))';
        rx_sig2 = collector(sig2, ang_matrix(:,2))';
        % Signal power for UE1 (already calculated)
        P_signal_UE1 = mean( (abs(UE(1).weights') * abs(rx_sig1 )).^2 );

        % Interference from UE2 on UE1's beam
        P_interference_UE1 = mean( (abs(UE(1).weights') * abs(rx_sig2) ).^2 );

        % Noise power after beamforming
        P_noise_UE1 = Pars.sigma2 * (UE(1).weights' * UE(1).weights);

        % SINR
        SINR_UE1(currentFrame) = P_signal_UE1 / (P_interference_UE1 + P_noise_UE1);
        SINR_UE1_dB(currentFrame) = 10*log10(SINR_UE1(currentFrame));
    else
        SINR_UE1_dB(currentFrame) = NaN;
    end

    %% (g2) SINR Calculation for UE2
    if UE(1).active && UE(2).active
        rx_sig1 = collector(sig1, ang_matrix(:,1))';
        rx_sig2 = collector(sig2, ang_matrix(:,2))';
        % Signal power for UE2 (already calculated)
        P_signal_UE2 = mean( (abs(UE(2).weights') * abs(rx_sig2 )).^2 );

        % Interference from UE1 on UE2's beam
        P_interference_UE2 = mean( (abs(UE(2).weights') * abs(rx_sig1) ).^2 );

        % Noise power after beamforming
        P_noise_UE2 = Pars.sigma2 * (UE(2).weights' * UE(2).weights);

        % SINR
        SINR_UE2(currentFrame) = P_signal_UE2 / (P_interference_UE2 + P_noise_UE2);
        SINR_UE2_dB(currentFrame) = 10*log10(SINR_UE2(currentFrame));
    else
        SINR_UE2_dB(currentFrame) = NaN;
    end


    %% (h) Store data for replay
    StoredData.V1Pos(:, currentFrame) = Geometry.V1Pos;
    StoredData.V2Pos(:, currentFrame) = Geometry.V2Pos;
    StoredData.UE1_weights(:, currentFrame) = UE(1).weights;
    StoredData.UE2_weights(:, currentFrame) = UE(2).weights;
    StoredData.ang_matrix(:, :, currentFrame) = ang_matrix;
    StoredData.validFrames = currentFrame;

    %% (i) Update plots
    set(hPlotV1, 'XData', Geometry.V1Pos(1), 'YData', Geometry.V1Pos(2));
    set(hPlotV2, 'XData', Geometry.V2Pos(1), 'YData', Geometry.V2Pos(2));
    addpoints(hTrail1, Geometry.V1Pos(1), Geometry.V1Pos(2));
    addpoints(hTrail2, Geometry.V2Pos(1), Geometry.V2Pos(2));

    % Update SINR plot in real-time
    time_axis = (1:currentFrame) * Pars.PhysicsStep;
    set(hSINR_line, 'XData', time_axis, 'YData', SINR_UE1_dB(1:currentFrame));
    set(hSINR_line2, 'XData', time_axis, 'YData', SINR_UE2_dB(1:currentFrame));

    
    if mod(currentFrame, 2) == 0
        % Pattern UE 1
        pat1 = abs(UE(1).weights' * sv_scan).^2;
        pat1_dB = 10*log10(pat1 + eps);
        pat1_dB = pat1_dB - max(pat1_dB);
        set(hPlotPat1, 'ThetaData', deg2rad(scan_az), 'RData', pat1_dB);

        % Pattern UE 2
        pat2 = abs(UE(2).weights' * sv_scan).^2;
        pat2_dB = 10*log10(pat2 + eps);
        pat2_dB = pat2_dB - max(pat2_dB);
        set(hPlotPat2, 'ThetaData', deg2rad(scan_az), 'RData', pat2_dB);

        % Combined Pattern (sum of power patterns)
        pat_total = pat1 + pat2;
        pat_total_dB = 10*log10(pat_total + eps);
        pat_total_dB = pat_total_dB - max(pat_total_dB);
        set(hPlotPatTotal, 'ThetaData', deg2rad(scan_az), 'RData', pat_total_dB);

        % True angle lines (geometry)
        t_az1 = ang_matrix(1,1);
        t_az2 = ang_matrix(1,2);
        set(hLineV1_p1, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
        set(hLineV2_p2, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);

        % Same angle lines on combined pattern
        set(hLineV1_total, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
        set(hLineV2_total, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);

        drawnow limitrate;
    end
end

%% 8) Post-simulation: Add time slider for replay
totalFrames = StoredData.validFrames;
StoredData.V1Pos = StoredData.V1Pos(:, 1:totalFrames);
StoredData.V2Pos = StoredData.V2Pos(:, 1:totalFrames);
StoredData.UE1_weights = StoredData.UE1_weights(:, 1:totalFrames);
StoredData.UE2_weights = StoredData.UE2_weights(:, 1:totalFrames);
StoredData.ang_matrix = StoredData.ang_matrix(:, :, 1:totalFrames);
SINR_UE1_dB = SINR_UE1_dB(1:totalFrames);
SINR_UE2_dB = SINR_UE2_dB(1:totalFrames);

% Add slider panel
sliderPanel = uipanel(fig, 'Position', [0.05 0.01 0.9 0.05], 'BorderType', 'none');
timeSlider = uicontrol(sliderPanel, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.1 0.3 0.75 0.4], ...
    'Min', 1, 'Max', totalFrames, 'Value', totalFrames, ...
    'SliderStep', [1/(totalFrames-1), 10/(totalFrames-1)]);

timeLabel = uicontrol(sliderPanel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.86 0.2 0.12 0.6], ...
    'String', sprintf('t = %.2f s', totalFrames*Pars.PhysicsStep), ...
    'FontSize', 10, 'HorizontalAlignment', 'left');

% Prepare all data needed for callback
sliderData.totalFrames = totalFrames;
sliderData.StoredData = StoredData;
sliderData.Pars = Pars;
sliderData.SINR_UE1_dB = SINR_UE1_dB;
sliderData.SINR_UE2_dB = SINR_UE2_dB;
sliderData.sv_scan = sv_scan;
sliderData.scan_az = scan_az;
sliderData.hPlotV1 = hPlotV1;
sliderData.hPlotV2 = hPlotV2;
sliderData.hTrail1 = hTrail1;
sliderData.hTrail2 = hTrail2;
sliderData.timeLabel = timeLabel;
sliderData.hPlotPat1 = hPlotPat1;
sliderData.hPlotPat2 = hPlotPat2;
sliderData.hPlotPatTotal = hPlotPatTotal;
sliderData.hLineV1_p1 = hLineV1_p1;
sliderData.hLineV2_p2 = hLineV2_p2;
sliderData.hLineV1_total = hLineV1_total;
sliderData.hLineV2_total = hLineV2_total;

timeSlider.UserData = sliderData;
timeSlider.Callback = @updatePlots;

fprintf('Simulazione completata. Usa lo slider per ripercorrere la simulazione.\n');

%% Nested function for slider callback
function updatePlots(src, ~)
    data = src.UserData;
    frameIdx = round(src.Value);
    if frameIdx < 1, frameIdx = 1; end
    if frameIdx > data.totalFrames, frameIdx = data.totalFrames; end
    
    % Update time label
    currentTime = frameIdx * data.Pars.PhysicsStep;
    set(data.timeLabel, 'String', sprintf('t = %.2f s', currentTime));
    
    % Update vehicle positions
    V1Pos_replay = data.StoredData.V1Pos(:, frameIdx);
    V2Pos_replay = data.StoredData.V2Pos(:, frameIdx);
    set(data.hPlotV1, 'XData', V1Pos_replay(1), 'YData', V1Pos_replay(2));
    set(data.hPlotV2, 'XData', V2Pos_replay(1), 'YData', V2Pos_replay(2));
    
    % Update trails
    clearpoints(data.hTrail1);
    clearpoints(data.hTrail2);
    for k = 1:frameIdx
        addpoints(data.hTrail1, data.StoredData.V1Pos(1,k), data.StoredData.V1Pos(2,k));
        addpoints(data.hTrail2, data.StoredData.V2Pos(1,k), data.StoredData.V2Pos(2,k));
    end
    
    % Update SINR plot
    time_axis = (1:data.totalFrames) * data.Pars.PhysicsStep;
    subplot(2,4,[3 4]); cla; hold on;
    plot(time_axis, data.SINR_UE1_dB, 'b-', 'LineWidth', 1.5);
    plot(time_axis, data.SINR_UE2_dB, 'r-', 'LineWidth', 1.5);
    plot(currentTime, data.SINR_UE1_dB(frameIdx), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(currentTime, data.SINR_UE2_dB(frameIdx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on;
    xlabel('Time [s]');
    ylabel('SINR [dB]');
    title('SINR of UE1 and UE2 vs Time');
    xlim([0 data.Pars.TotalTime_s]);
    ylim([-10 30]);
    
    % Update beam patterns
    w1 = data.StoredData.UE1_weights(:, frameIdx);
    w2 = data.StoredData.UE2_weights(:, frameIdx);
    ang_matrix_replay = data.StoredData.ang_matrix(:, :, frameIdx);
    
    % Pattern UE 1
    pat1 = abs(w1' * data.sv_scan).^2;
    pat1_dB = 10*log10(pat1 + eps);
    pat1_dB = pat1_dB - max(pat1_dB);
    set(data.hPlotPat1, 'ThetaData', deg2rad(data.scan_az), 'RData', pat1_dB);
    
    % Pattern UE 2
    pat2 = abs(w2' * data.sv_scan).^2;
    pat2_dB = 10*log10(pat2 + eps);
    pat2_dB = pat2_dB - max(pat2_dB);
    set(data.hPlotPat2, 'ThetaData', deg2rad(data.scan_az), 'RData', pat2_dB);
    
    % Combined Pattern
    pat_total = pat1 + pat2;
    pat_total_dB = 10*log10(pat_total + eps);
    pat_total_dB = pat_total_dB - max(pat_total_dB);
    set(data.hPlotPatTotal, 'ThetaData', deg2rad(data.scan_az), 'RData', pat_total_dB);
    
    % Update angle lines
    t_az1 = ang_matrix_replay(1,1);
    t_az2 = ang_matrix_replay(1,2);
    set(data.hLineV1_p1, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
    set(data.hLineV2_p2, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);
    set(data.hLineV1_total, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
    set(data.hLineV2_total, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);
    
    drawnow;
end


