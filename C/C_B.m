close all; clear; clc;

%% =======================
%  1) OFDM WAVEFORM
%% =======================
pilotIndices = [12;26;40;54];
scs = 1e6;       % subcarrier spacing
M   = 16;        % 16-QAM
numSymbols   = 100;
CPLength     = 16;
Nfft         = 64;
Ns           = Nfft + CPLength;
GuardBands   = [6;6];  %Equal numbers!
NumCarriersData = Nfft - sum(GuardBands);
LenSubBands = NumCarriersData/numel(pilotIndices);
NumBitsPerSymbol = (Nfft-numel(pilotIndices)-sum(GuardBands))*log2(M);

ofdmMod = comm.OFDMModulator( ...
    'FFTLength', Nfft, ...
    'NumGuardBandCarriers', GuardBands, ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', CPLength, ...
    'Windowing', false, ...
    'OversamplingFactor', 1, ...
    'NumSymbols', numSymbols, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', pilotIndices);

in = randi([0 1], numSymbols*NumBitsPerSymbol, 1);
dataInput = qammod(in, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);

ofdmInfo  = info(ofdmMod);
dataInput = reshape(dataInput, ofdmInfo.DataInputSize);

pilotInput = ones(numel(pilotIndices), numSymbols, 1);

waveform = ofdmMod(dataInput, pilotInput);         % [Ns*numSymbols x 1]
Fs       = Nfft * scs;
Bandwidth = scs*Nfft;
tx_sym = reshape(waveform, Ns, numSymbols);

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
BS.pos = [0; 0; 10];    % 10 m di altezza

% UE iniziale
UE.pos0 = [40; 20; 1.5];
UE.pos  = UE.pos0;

% direzione di movimento 
dir = [-1.5; 0; 0]; 
dir = dir / norm(dir);

UE.speed_kmh = 10;                     % km/h
UE.v_ms      = UE.speed_kmh * (1000/3600);
UE.vel       = UE.v_ms * dir;          % m/s

%% =======================
%  3) Array, Channel, Collector
%% =======================
antSize = [8, 8];                    % UPA 6x6
Nant    = prod(antSize);

BS.array = phased.URA( ...
    'Size', antSize, ...
    'ElementSpacing', [Pars.lambda/2 Pars.lambda/2], ...
    'Element', phased.IsotropicAntennaElement);
% rotate antenna toward -z
BS.array.ArrayNormal = "y";   
figure;
pos = getElementPosition(BS.array);   % [3 x Nant] positions of array elements

scatter3(pos(1,:), pos(2,:), pos(3,:), 60, 'filled', 'b');
hold on; grid on;

% Mark array centroid
centroid = mean(pos,2);
plot3(centroid(1), centroid(2), centroid(3), 'ro', 'MarkerSize', 10, 'LineWidth',2);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('UPA 3D Geometry');

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
NumOFDMSymbForLMS = 2;
d_desired  = zeros(numPilots, ceil(numSymbols/NumOFDMSymbForLMS));

cnt = 1;
for k = 1:numSymbols
    if mod(k,NumOFDMSymbForLMS) == 1
        tx_noCP = tx_sym(CPLength+1:CPLength+Nfft, k);
        TXF     = fft(tx_noCP);
        d_desired(:, cnt) = TXF(pilotIndices);
        cnt = cnt + 1;
    end
end

w = zeros(Nant, numPilots);        % pesi iniziali (Nant x numPilots)

%% =======================
%  5) Steering per pattern
%% =======================
steer = phased.SteeringVector('SensorArray', BS.array, ...
                              'PropagationSpeed', Pars.c);

theta = -90:1:90;     % azimuth
phi   = -90:1:90;       % elevation

sv_az = steer(Pars.fc, [theta; zeros(size(theta))]);   % slice elev=0
sv_el = steer(Pars.fc, [zeros(size(phi)); phi]);       % slice az=0

%% =======================
%  6) Figure & 3D Plot
%% =======================
fig = figure('WindowState','maximized');
tiledlayout(2,2);

% 3D scenario
ax1 = nexttile(1);
hold(ax1,'on'); grid(ax1,'on');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Scenario (BS + UE)');
axis equal;
xlim([-20 40]); ylim([-20 20]); zlim([0 20]);

BSplot = plot3(BS.pos(1), BS.pos(2), BS.pos(3), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName','BS');
UEplot = plot3(NaN, NaN, NaN, 'bo-', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'b', 'DisplayName','UE');

legend('Location','northwest');
view(3);

% Polar azimuth
ax2 = nexttile(2);
azPlot = polarplot(0,0,'LineWidth',2);
rlim([-40 0]);
title('Azimuth pattern (elev = 0°)');

% Polar elevation
ax3 = nexttile(3);
elPlot = polarplot(0,0,'LineWidth',2);
rlim([-40 0]);
title('Elevation pattern (az = 0°)');

% Info box
ax4 = nexttile(4);
axis off;
infoText = text(0.1,0.5,'','FontSize',12);

drawnow;

UE_pos_hist = zeros(3, Pars.numFrame);

avgBerTot = 0.5*ones(Pars.numFrame,1);
%% =======================
%  7) MAIN LOOP (frames)
%% =======================
for frame = 1:Pars.numFrame
    w = zeros(Nant, numPilots);        % pesi iniziali (Nant x numPilots)

    % movimento UE in 3D
    UE.pos = UE.pos + UE.vel * dt;
    UE_pos_hist(:,frame) = UE.pos;

    % angoli di arrivo
    rel = -BS.pos + UE.pos;
    az = rad2deg(atan2(rel(2),rel(1)));
    el = rad2deg(atan2(rel(3), norm(rel(1:2))));
    ang = [az; el];

    % canale sul waveform OFDM
    rx      = channel(waveform, UE.pos, BS.pos, UE.vel, [0;0;0]);
    rx_array = collector(rx, ang);                 % [Ns*numSymbols x Nant]

    sigma_sq = db2pow(-174 + 10*log10(Pars.noiseTemp/293.15) + Pars.noiseFactor ...
                        +10*log10(Bandwidth)-30);
    %noise = sqrt(sigma_sq/2) * (randn(size(rx_array)) + 1j*randn(size(rx_array)));
    noise = zeros(size(rx_array));
    rx_array = rx_array + noise;
    % LMS sui simboli OFDM (freq-domain, update ogni 3 simboli)
    cntLMS = 1;
    for k = 1:numSymbols
        if mod(k,NumOFDMSymbForLMS) == 1
            idx = (k-1)*Ns + (CPLength+1:CPLength+Nfft);
            rx_noCP = rx_array(idx, :);                % [64 x Nant]
            X = fft(rx_noCP);                          % [64 x Nant]
            for p = 1:numPilots
                Xp = X(pilotIndices(p), :).';      % Nant x 1
                d  = d_desired(p, cntLMS);         % pilot noto TX (freq)
                y  = w(:,p)' * Xp;
                e  = d - y;
                mu = 0.01 / (Xp'*Xp + 1e-9);
                w(:,p) = w(:,p) + mu * Xp * conj(e);
            end
            cntLMS = cntLMS + 1;
        end
    end

% Precompute bits transmitted
bitGrid = qamdemod(dataInput, M, 'gray', 'OutputType','bit'); 
% bitGrid(i,k) = bits of subcarrier i, symbol k

ber = zeros(numel(pilotIndices),1);

for p0 = 1:numel(pilotIndices)

    % sub-band around pilot p0
    interval = pilotIndices(p0)-floor(LenSubBands/2) : ...
               pilotIndices(p0)+floor(LenSubBands/2);
    interval = interval(interval>=1 & interval<=Nfft);   % safety
    %interval(ceil(numel(interval)/2)) = [];

    err_tot = 0;
    bit_tot = 0;

    for k = 1:numSymbols

        % remove CP
        idx = (k-1)*Ns + (CPLength+1:CPLength+Nfft);
        rx_noCP = rx_array(idx,:);     % [64 × Nant]

        % FFT
        X = fft(rx_noCP);              % [64 × Nant]  
        X(pilotIndices, :) = [];        % now X = [64 - numPilots] × Nant

        % beamforming su sub-band con pesi del pilot p0
        Ysb = (w(:,p0)' * X(interval,:).').';  
        % Ysb -> vector [|interval| × 1]

        % QAM demod
        bits_est = qamdemod(Ysb, M, 'gray', 'OutputType','bit');

        intBits = (p0-1)*log2(M)*(LenSubBands-1)+1:p0*log2(M)*(LenSubBands-1);
        % True bits
        bits_true = bitGrid(intBits, k);

        % Errori
        err_tot = err_tot + sum(bits_est(:) ~= bits_true(:));
        bit_tot = bit_tot + numel(bits_true);

    end

    ber(p0) = err_tot / bit_tot;
end

avgBerTot(frame) = mean(ber);

    
    % info testo
    set(infoText, 'String', sprintf('Frame %d / %d\nTime = %.1f s\nAz = %.1f°, El = %.1f°', ...
        frame, Pars.numFrame, frame*dt, az, el));

    drawnow limitrate;
    pause(0.05);
end

figure;
plot(avgBerTot);
yaxis("BER");
xaxis("numFrame");