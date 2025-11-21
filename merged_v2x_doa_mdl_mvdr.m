close all;
clear;
clc;

%% ============================================================
%%  SCRIPT (DOA note dalla geometria)
%%  - Vehicles in movement
%%  - UE struct extended
%%  - Per-frame estimation of number of UEs (MDL)
%%  - DOA known from geometry (NO Root-MUSIC)
%%  - MVDR beamforming
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

% SNR control
Pars.SNR_dB = 10;

% Vehicles parameters
Pars.speed1_kmh = 7;
Pars.speed2_kmh = 5;
Pars.v1_ms = Pars.speed1_kmh * (1000/3600);
Pars.v2_ms = Pars.speed2_kmh * (1000/3600);

% Waveforms
waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect).';
waveform2 = sign(sin(2*pi*Pars.Fsin*Pars.TsVect + pi/3)).';

%SNRs
SINR_UE1 = zeros(Pars.numFrame, 1);
SINR_UE1_dB = zeros(Pars.numFrame, 1);

%% 2) Geometry + Array
N_Elements = 16;
Geometry.BSarray = phased.ULA('NumElements', N_Elements, ...
    'ElementSpacing', Pars.lambda/2);

Geometry.BSPos = [0; 0; 0];
Geometry.V1Pos = [20; -20; 0];
Geometry.V2Pos = [20;  20; 0];

dir1 = [1; 0.5; 0]; dir1 = dir1/norm(dir1);
dir2 = [0.5; -1; 0]; dir2 = dir2/norm(dir2);
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

%% 5) Visualization 
figure('Position',[50 100 1400 500]);

subplot(1,3,1);
plot(Geometry.BSPos(1), Geometry.BSPos(2), 'k^', ...
    'MarkerSize',10,'LineWidth',2,'DisplayName','BS'); hold on;
hPlotV1 = plot(Geometry.V1Pos(1), Geometry.V1Pos(2), 'bo', ...
    'MarkerFaceColor','b','DisplayName','UE 1');
hPlotV2 = plot(Geometry.V2Pos(1), Geometry.V2Pos(2), 'rs', ...
    'MarkerFaceColor','r','DisplayName','UE 2');
hTrail1 = animatedline('Color','b','LineStyle',':', 'DisplayName', "Trajectory UE1");
hTrail2 = animatedline('Color','r','LineStyle',':', 'DisplayName', "Trajectory UE2");
axis([-10 100 -20 80]); grid on; title('Scenario V2X'); legend('Location','northwest');

subplot(1,3,2);
hPlotPat1 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'b'); hold on;
hLineV1_p1 = polarplot([0 0], [0 -40], 'b--');
title('Beam UE 1 (BLU)'); rlim([-40 0]);

subplot(1,3,3);
hPlotPat2 = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'r'); hold on;
hLineV2_p2 = polarplot([0 0], [0 -40], 'r--');
title('Beam UE 2 (RED)'); rlim([-40 0]);

%% 6) MAIN LOOP
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
    SNRlin = 10^(Pars.SNR_dB/10);
    Px = mean(abs(rx_clean(:)).^2);
    sigma2 = Px / SNRlin;

    noise = sqrt(sigma2/2) * (randn(size(rx_clean)) + 1j*randn(size(rx_clean)));
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
        UE(k).DOA_prev = UE(k).DOA(:);

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
        UE(k).SNR = Pars.SNR_dB;
    end

    %% (f) MVDR beamforming (only for active UEs)
    % Ensure rx_signal is Nelem x Nsnap
    if size(rx_signal,1) ~= N_Elements
        rx_signal = rx_signal.'; 
    end
    Nsnap = size(rx_signal,2);

    Rxx = (rx_signal * rx_signal') / Nsnap;  % Nelem x Nelem
    Rxx = Rxx + 1e-3 * eye(N_Elements);
    R_inv = inv(Rxx);

    for k = 1:maxUsers
        if ~UE(k).active
            UE(k).weights = ones(N_Elements,1)/N_Elements;
            UE(k).output  = zeros(1, Nsnap);
            continue;
        end

        target_az = UE(k).DOA(1);
        target_az = mod(target_az + 180, 360) - 180;

        sv = steeringVec(Pars.fc, [target_az; 0]);
        num = R_inv * sv;
        den = sv' * R_inv * sv;
        UE(k).weights = num / den;

        UE(k).output = UE(k).weights' * rx_signal;
        UE(k).power  = mean(abs(UE(k).output).^2);
    end

    %% SINR Calculation for UE1
    if UE(1).active && UE(2).active
        rx_sig1 = collector(sig1, ang_matrix(:,1))';
        rx_sig2 = collector(sig2, ang_matrix(:,2))';
        % Signal power for UE1 (already calculated)
        P_signal_UE1 = mean(abs( UE(1).weights' * rx_sig1 ).^2);
        
        % Interference from UE2 on UE1's beam
        P_interference_UE1 = mean(abs( UE(1).weights' * rx_sig2 ).^2);
        
        % Noise power after beamforming
        P_noise_UE1 = sigma2 * (UE(1).weights' * UE(1).weights);
        
        % SINR
        SINR_UE1(currentFrame) = P_signal_UE1 / (P_interference_UE1 + P_noise_UE1);
        SINR_UE1_dB(currentFrame) = 10*log10(SINR_UE1(currentFrame));
    else
        SINR_UE1_dB(currentFrame) = NaN;
    end
    fprintf("Frame %d: true az = [%.1f %.1f], MDL users = %d\n", ...
        currentFrame, ang_matrix(1,1), ang_matrix(1,2), numUsers_est);

    %% (g) Update plots every 2 frames
    set(hPlotV1, 'XData', Geometry.V1Pos(1), 'YData', Geometry.V1Pos(2));
    set(hPlotV2, 'XData', Geometry.V2Pos(1), 'YData', Geometry.V2Pos(2));
    addpoints(hTrail1, Geometry.V1Pos(1), Geometry.V1Pos(2));
    addpoints(hTrail2, Geometry.V2Pos(1), Geometry.V2Pos(2));

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

        % True angle lines (geometry)
        t_az1 = ang_matrix(1,1);
        t_az2 = ang_matrix(1,2);
        set(hLineV1_p1, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
        set(hLineV2_p2, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);

        drawnow limitrate;
    end
end

%% Plot SINR vs Time
figure;
time_axis = (1:currentFrame) * Pars.PhysicsStep;
plot(time_axis, SINR_UE1_dB, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('SINR [dB]');
title('SINR of UE1 vs Time');
ylim([min(SINR_UE1_dB)-5, max(SINR_UE1_dB)+5]);
