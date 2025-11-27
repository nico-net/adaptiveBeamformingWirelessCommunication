close all; clear; clc;

%% =======================
%  1) OFDM WAVEFORM
%% =======================
pilotIndices = [12;26;40;54];
numSymbols   = 100;
CPLength     = 16;
Nfft         = 64;
Ns           = Nfft + CPLength;

ofdmMod = comm.OFDMModulator( ...
    'FFTLength', Nfft, ...
    'NumGuardBandCarriers', [6;5], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', CPLength, ...
    'Windowing', false, ...
    'OversamplingFactor', 1, ...
    'NumSymbols', numSymbols, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', pilotIndices);

scs = 1e6;       % subcarrier spacing
M   = 16;        % 16-QAM

in = randi([0 1], 19600, 1);
dataInput = qammod(in, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);

ofdmInfo  = info(ofdmMod);
dataInput = reshape(dataInput, ofdmInfo.DataInputSize);

pilotInput = ones(numel(pilotIndices), numSymbols, 1);

waveform = ofdmMod(dataInput, pilotInput);         % [Ns*numSymbols x 1]
Fs       = Nfft * scs;
Bandwidth = scs*Nfft;
tx_sym = reshape(waveform, Ns, numSymbols);

% Second waveform with frequency offset for UE2
in2 = randi([0 1], 19600, 1);
dataInput2 = qammod(in2, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
dataInput2 = reshape(dataInput2, ofdmInfo.DataInputSize);
waveform2 = ofdmMod(dataInput2, pilotInput);
tx_sym2 = reshape(waveform2, Ns, numSymbols);

%% =======================
%  2) Geometry & Motion
%% =======================
Pars.fc      = 3.5e9;
Pars.c       = physconst('LightSpeed');
Pars.lambda  = Pars.c / Pars.fc;

Pars.noiseTemp = 293.15; %in K
Pars.noiseFactor = 5; %in dB

Pars.TotalTime_s  = 30;     % durata simulazione
Pars.PhysicsStep  = 0.1;    % passo di fisica [s]
Pars.numFrame     = ceil(Pars.TotalTime_s / Pars.PhysicsStep);
dt = Pars.PhysicsStep;

% BS
BS.pos = [0; 0; 15];    % 15 m di altezza

% UE1 iniziale
UE1.pos0 = [40; 20; 1.5];
UE1.pos  = UE1.pos0;

% direzione di movimento UE1
dir1 = [-1.5; 0; 0]; 
dir1 = dir1 / norm(dir1);

UE1.speed_kmh = 10;                     % km/h
UE1.v_ms      = UE1.speed_kmh * (1000/3600);
UE1.vel       = UE1.v_ms * dir1;          % m/s

% UE2 iniziale
UE2.pos0 = [60; -20; 0];
UE2.pos  = UE2.pos0;

% direzione di movimento UE2
dir2 = [-1; 0.5; 5]; 
dir2 = dir2 / norm(dir2);

UE2.speed_kmh = 30;                     % km/h
UE2.v_ms      = UE2.speed_kmh * (1000/3600);
UE2.vel       = UE2.v_ms * dir2;          % m/s

%% =======================
%  3) Array, Channel, Collector
%% =======================
antSize = [8, 8];                    % UPA 8x8
Nant    = prod(antSize);

BS.array = phased.URA( ...
    'Size', antSize, ...
    'ElementSpacing', [Pars.lambda/2 Pars.lambda/2], ...
    'Element', phased.IsotropicAntennaElement);

% MODIFIED: Rotate antenna to be parallel to ground (normal pointing down)
BS.array.ArrayNormal = "z";   

figure('Name','Antenna Array Geometry');
pos = getElementPosition(BS.array);   % [3 x Nant] positions of array elements

scatter3(pos(1,:), pos(2,:), pos(3,:), 60, 'filled', 'b');
hold on; grid on;

% Mark array centroid
centroid = mean(pos,2);
plot3(centroid(1), centroid(2), centroid(3), 'ro', 'MarkerSize', 10, 'LineWidth',2);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('UPA 3D Geometry (Parallel to Ground)');

axis equal;
view(3);
hold on;

% length of axis arrows
L = 0.5 * max(range(pos,2));

quiver3(centroid(1),centroid(2),centroid(3), L,0,0,'r','LineWidth',2); % X axis
quiver3(centroid(1),centroid(2),centroid(3), 0,L,0,'g','LineWidth',2); % Y axis
quiver3(centroid(1),centroid(2),centroid(3), 0,0,L,'b','LineWidth',2); % Z axis
legend('Elements','Centroid','X-axis','Y-axis','Z-axis');

collector = phased.Collector('Sensor', BS.array, ...
    'OperatingFrequency', Pars.fc);

channel = phased.FreeSpace( ...
    'OperatingFrequency', Pars.fc, ...
    'SampleRate', Fs, ...
    'TwoWayPropagation', false);

%% =======================
%  4) LMS setup (freq-domain)
%% =======================
numPilots  = numel(pilotIndices);
NumOFDMSymbForLMS = 3;
d_desired1  = zeros(numPilots, ceil(numSymbols/NumOFDMSymbForLMS));
d_desired2  = zeros(numPilots, ceil(numSymbols/NumOFDMSymbForLMS));

cnt = 1;
for k = 1:numSymbols
    if mod(k,NumOFDMSymbForLMS) == 1
        tx_noCP = tx_sym(CPLength+1:CPLength+Nfft, k);
        TXF     = fft(tx_noCP);
        d_desired1(:, cnt) = TXF(pilotIndices);
        
        tx_noCP2 = tx_sym2(CPLength+1:CPLength+Nfft, k);
        TXF2     = fft(tx_noCP2);
        d_desired2(:, cnt) = TXF2(pilotIndices);
        cnt = cnt + 1;
    end
end

w1 = zeros(Nant, numPilots);        % pesi iniziali UE1
w2 = zeros(Nant, numPilots);        % pesi iniziali UE2

%% =======================
%  5) Steering per pattern
%% =======================
steer = phased.SteeringVector('SensorArray', BS.array, ...
                              'PropagationSpeed', Pars.c);

theta = -90:1:90;     % azimuth
phi   = -90:1:90;     % elevation

sv_az = steer(Pars.fc, [theta; zeros(size(theta))]);   % slice elev=0
sv_el = steer(Pars.fc, [zeros(size(phi)); phi]);       % slice az=0

%% =======================
%  6) Figure & Plots (5 subplots)
%% =======================
fig = figure('WindowState','maximized');
tiledlayout(3,2);

% 3D scenario
ax1 = nexttile(1);
hold(ax1,'on'); grid(ax1,'on');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Scenario (BS + UE1 + UE2)');
axis equal;
xlim([-20 80]); ylim([-40 40]); zlim([0 30]);

BSplot = plot3(BS.pos(1), BS.pos(2), BS.pos(3), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName','BS');
UE1plot = plot3(NaN, NaN, NaN, 'bo-', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'b', 'DisplayName','UE1');
UE2plot = plot3(NaN, NaN, NaN, 'ms-', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'm', 'DisplayName','UE2');

legend('Location','northwest');
view(3);

% Polar azimuth
ax2 = nexttile(3);
azPlot1 = polarplot(0,0,'b-','LineWidth',2); hold on;
azPlot2 = polarplot(0,0,'m-','LineWidth',2);
rlim([-40 0]);
title('Elevation pattern (az = 0°)');
legend('UE1','UE2');

% Polar elevation
ax3 = nexttile(4);
elPlot1 = polarplot(0,0,'b-','LineWidth',2); hold on;
elPlot2 = polarplot(0,0,'m-','LineWidth',2);
rlim([-40 0]);
title('Azimuth pattern (elev = 0°)');
legend('UE1','UE2');

% SINR plot
ax4 = nexttile(2);
hold(ax4,'on'); grid(ax4,'on');
SINR1plot = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName','UE1');
SINR2plot = plot(NaN, NaN, 'm-', 'LineWidth', 1.5, 'DisplayName','UE2');
xlabel('Time [s]'); ylabel('SINR [dB]');
title('SINR vs Time');
legend('Location','best');
xlim([0 Pars.TotalTime_s]);

% Beamforming weights magnitude
% ax5 = nexttile(5);
% hold(ax5,'on'); grid(ax5,'on');
% weightsPlot1 = stem(1:Nant, NaN(Nant,1), 'b', 'filled', 'DisplayName','UE1');
% weightsPlot2 = stem(1:Nant, NaN(Nant,1), 'm', 'filled', 'DisplayName','UE2');
% xlabel('Antenna Element'); ylabel('Weight Magnitude');
% title('Beamforming Weights (avg across pilots)');
% legend('Location','best');
% xlim([0 Nant+1]);

% Info box
ax6 = nexttile(6);
axis off;
infoText = text(0.1,0.5,'','FontSize',11);

drawnow;

UE1_pos_hist = zeros(3, Pars.numFrame);
UE2_pos_hist = zeros(3, Pars.numFrame);
SINR1_hist = zeros(1, Pars.numFrame);
SINR2_hist = zeros(1, Pars.numFrame);
time_hist = zeros(1, Pars.numFrame);

%% =======================
%  7) MAIN LOOP (frames)
%% =======================
for frame = 1:Pars.numFrame

    % movimento UE in 3D
    UE1.pos = UE1.pos + UE1.vel * dt;
    UE2.pos = UE2.pos + UE2.vel * dt;
    UE1_pos_hist(:,frame) = UE1.pos;
    UE2_pos_hist(:,frame) = UE2.pos;
    time_hist(frame) = frame * dt;

    % angoli di arrivo
    rel1 = BS.pos - UE1.pos;
    az1 = rad2deg(atan2(rel1(2),rel1(1)));
    el1 = rad2deg(atan2(rel1(3), norm(rel1(1:2))));
    ang1 = [az1; el1];
    
    rel2 = BS.pos - UE2.pos;
    az2 = rad2deg(atan2(rel2(2),rel2(1)));
    el2 = rad2deg(atan2(rel2(3), norm(rel2(1:2))));
    ang2 = [az2; el2];

    % canale sul waveform OFDM per entrambi gli UE
    rx1      = channel(waveform, UE1.pos, BS.pos, UE1.vel, [0;0;0]);
    rx_array1 = collector(rx1, ang1);                 % [Ns*numSymbols x Nant]
    
    rx2      = channel(waveform2, UE2.pos, BS.pos, UE2.vel, [0;0;0]);
    rx_array2 = collector(rx2, ang2);

    % Somma segnali ricevuti + rumore
    sigma_sq = db2pow(-174 + 10*log10(Pars.noiseTemp/293.15) + Pars.noiseFactor ...
                        +10*log10(Bandwidth)-30);
    noise = sqrt(sigma_sq/2) * (randn(size(rx_array1)) + 1j*randn(size(rx_array1)));
    rx_array = rx_array1 + rx_array2 + noise;
    
    % LMS sui simboli OFDM (freq-domain, update ogni 3 simboli)
    cntLMS = 1;
    for k = 1:numSymbols
        idx = (k-1)*Ns + (CPLength+1:CPLength+Nfft);
        rx_noCP = rx_array(idx, :);                % [64 x Nant]
        X = fft(rx_noCP);                          % [64 x Nant]

        if mod(k,NumOFDMSymbForLMS) == 1
            % Update weights for UE1
            for p = 1:numPilots
                Xp = X(pilotIndices(p), :).';      % Nant x 1
                d  = d_desired1(p, cntLMS);        
                y  = w1(:,p)' * Xp;
                e  = d - y;
                mu = 0.01 / (Xp'*Xp + 1e-9);
                w1(:,p) = w1(:,p) + mu * Xp * conj(e);
            end
            
            % Update weights for UE2
            for p = 1:numPilots
                Xp = X(pilotIndices(p), :).';      
                d  = d_desired2(p, cntLMS);        
                y  = w2(:,p)' * Xp;
                e  = d - y;
                mu = 0.01 / (Xp'*Xp + 1e-9);
                w2(:,p) = w2(:,p) + mu * Xp * conj(e);
            end
            cntLMS = cntLMS + 1;
        end
    end

    % Calculate SINR for both UEs
    % Signal power for UE1
    rx1_clean = collector(rx1, ang1);
    P_s1 = mean(abs(mean(w1,2)' * rx1_clean').^2);
    % Interference from UE2
    P_i1 = mean(abs(mean(w1,2)' * rx_array2').^2);
    % Noise power
    P_n1 = sigma_sq * (mean(w1,2)' * mean(w1,2));
    SINR1 = P_s1 / (P_i1 + P_n1);
    SINR1_hist(frame) = 10*log10(SINR1 + eps);
    
    % Signal power for UE2
    rx2_clean = collector(rx2, ang2);
    P_s2 = mean(abs(mean(w2,2)' * rx2_clean').^2);
    % Interference from UE1
    P_i2 = mean(abs(mean(w2,2)' * rx_array1').^2);
    % Noise power
    P_n2 = sigma_sq * (mean(w2,2)' * mean(w2,2));
    SINR2 = P_s2 / (P_i2 + P_n2);
    SINR2_hist(frame) = 10*log10(SINR2 + eps);

    % average pattern
    pattern_azK1 = 0;
    pattern_elK1 = 0;
    pattern_azK2 = 0;
    pattern_elK2 = 0;
    
    for p0 = 1:numel(pilotIndices)
        pattern_azK1 = pattern_azK1 + abs(w1(:,p0)' * sv_az).^2;    
        pattern_elK1 = pattern_elK1 + abs(w1(:,p0)' * sv_el).^2;
        pattern_azK2 = pattern_azK2 + abs(w2(:,p0)' * sv_az).^2;    
        pattern_elK2 = pattern_elK2 + abs(w2(:,p0)' * sv_el).^2;
    end

    pattern_az1 = pattern_azK1/numPilots;
    pattern_el1 = pattern_elK1/numPilots;
    pattern_az2 = pattern_azK2/numPilots;
    pattern_el2 = pattern_elK2/numPilots;

    pattern_az1dB = 10*log10(pattern_az1 / max(pattern_az1));
    pattern_el1dB = 10*log10(pattern_el1 / max(pattern_el1));
    pattern_az2dB = 10*log10(pattern_az2 / max(pattern_az2));
    pattern_el2dB = 10*log10(pattern_el2 / max(pattern_el2));

    % aggiorna 3D UE/BS
    set(UE1plot, ...
        'XData', UE1_pos_hist(1,1:frame), ...
        'YData', UE1_pos_hist(2,1:frame), ...
        'ZData', UE1_pos_hist(3,1:frame));
    
    set(UE2plot, ...
        'XData', UE2_pos_hist(1,1:frame), ...
        'YData', UE2_pos_hist(2,1:frame), ...
        'ZData', UE2_pos_hist(3,1:frame));

    % aggiorna polar azimuth
    set(azPlot1, 'ThetaData', deg2rad(theta), 'RData', pattern_az1dB);
    set(azPlot2, 'ThetaData', deg2rad(theta), 'RData', pattern_az2dB);

    % aggiorna polar elevation
    set(elPlot1, 'ThetaData', deg2rad(phi), 'RData', pattern_el1dB);
    set(elPlot2, 'ThetaData', deg2rad(phi), 'RData', pattern_el2dB);
    
    % aggiorna SINR plot
    set(SINR1plot, 'XData', time_hist(1:frame), 'YData', SINR1_hist(1:frame));
    set(SINR2plot, 'XData', time_hist(1:frame), 'YData', SINR2_hist(1:frame));
    
    % aggiorna weights magnitude
    % w1_avg = mean(abs(w1), 2);
    % w2_avg = mean(abs(w2), 2);
    % set(weightsPlot1, 'YData', w1_avg);
    % set(weightsPlot2, 'YData', w2_avg);

    % info testo
    set(infoText, 'String', sprintf(['Frame %d / %d\nTime = %.1f s\n\n' ...
        'UE1: Az = %.1f°, El = %.1f°\nSINR1 = %.1f dB\n\n' ...
        'UE2: Az = %.1f°, El = %.1f°\nSINR2 = %.1f dB'], ...
        frame, Pars.numFrame, frame*dt, az1, el1, SINR1_hist(frame), ...
        az2, el2, SINR2_hist(frame)));

    drawnow limitrate;
    pause(0.05);
end

fprintf('Simulation Complete!\n');
fprintf('Final SINR - UE1: %.2f dB, UE2: %.2f dB\n', SINR1_hist(end), SINR2_hist(end));