close all; clear; clc;

%% =======================
%  1) OFDM WAVEFORM
%% =======================
pilotIndices = [12;26;40;54];
CPLength     = 16;
Nfft         = 64;
Ns           = Nfft + CPLength;

% Creiamo un modulatore OFDM per SINGOLO SIMBOLO
ofdmMod = comm.OFDMModulator( ...
    'FFTLength', Nfft, ...
    'NumGuardBandCarriers', [6;5], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', CPLength, ...
    'Windowing', false, ...
    'OversamplingFactor', 1, ...
    'NumSymbols', 1, ...              % UN SIMBOLO ALLA VOLTA
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', pilotIndices);

scs = 1e6;       % subcarrier spacing
M   = 16;        % 16-QAM

ofdmInfo  = info(ofdmMod);
numDataCarriers = ofdmInfo.DataInputSize(1);

Fs        = Nfft * scs;
Bandwidth = scs*Nfft;

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
UE.pos0 = [20; -20; 1.5];
UE.pos  = UE.pos0;

% direzione di movimento 
dir = [0; 2; 0.5]; 
dir = dir / norm(dir);

UE.speed_kmh = 10;                     % km/h
UE.v_ms      = UE.speed_kmh * (1000/3600);
UE.vel       = UE.v_ms * dir;          % m/s

%% =======================
%  3) Array, Channel, Collector
%% =======================
antSize = [8, 8];                    % UPA 8x8
Nant    = prod(antSize);

BS.array = phased.URA( ...
    'Size', antSize, ...
    'ElementSpacing', [Pars.lambda/2 Pars.lambda/2], ...
    'Element', phased.IsotropicAntennaElement);

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

% Pesi LMS: un vettore per ogni sottoportante pilota
w = zeros(Nant, numPilots);        % pesi iniziali (Nant x numPilots)

% Pilot symbols noti (riferimento per LMS)
pilotSymbols = ones(numPilots, 1);  % nel dominio della frequenza

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
xlim([-20 60]); ylim([-30 60]); zlim([0 20]);

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

% Calcolo rumore
sigma_sq = db2pow(-174 + 10*log10(Pars.noiseTemp/293.15) + Pars.noiseFactor ...
                    +10*log10(Bandwidth)-30);

%% =======================
%  7) MAIN LOOP (frames)
%% =======================
for frame = 1:Pars.numFrame

    % movimento UE in 3D
    UE.pos = UE.pos + UE.vel * dt;
    UE_pos_hist(:,frame) = UE.pos;

    % angoli di arrivo
    rel = UE.pos - BS.pos;  % Vettore DA BS VERSO UE
    dist_xy = norm(rel(1:2));
    
    % Azimuth: angolo sul piano XY rispetto all'asse X
    az = rad2deg(atan2(rel(2), rel(1)));
    
    % Elevation: angolo rispetto al piano XY
    el = rad2deg(atan2(rel(3), dist_xy));
    
    ang = [az; el];

    % ========================================
    % GENERA UN SIMBOLO OFDM E PROCESSALO
    % ========================================
    
    % Genera dati casuali per questo simbolo
    dataSymbols = qammod(randi([0 M-1], numDataCarriers, 1), M, ...
                         'UnitAveragePower', true);
    
    % Genera waveform OFDM per questo singolo simbolo
    waveform = ofdmMod(dataSymbols, pilotSymbols);  % [Ns x 1]
    
    % Applica canale
    rx = channel(waveform, UE.pos, BS.pos, UE.vel, [0;0;0]);  % [Ns x 1]
    
    % Raccogli sulle antenne
    rx_array = collector(rx, ang);  % [Ns x Nant]
    
    % Aggiungi rumore
    noise = sqrt(sigma_sq/2) * (randn(size(rx_array)) + 1j*randn(size(rx_array)));
    rx_array = rx_array + noise;
    
    % ========================================
    % PROCESSA IL SIMBOLO OFDM RICEVUTO
    % ========================================
    
    % Rimuovi CP
    rx_noCP = rx_array(CPLength+1:end, :);  % [Nfft x Nant]
    
    % FFT per ottenere sottoportanti
    X = fft(rx_noCP);  % [Nfft x Nant]
    
    % ========================================
    % AGGIORNA LMS SUI PILOT
    % ========================================
    mu = 0.01;  % step size aumentato leggermente
    
    for p = 1:numPilots
        pilotIdx = pilotIndices(p);
        Xp = X(pilotIdx, :).';  % [Nant x 1] - segnale ricevuto su pilot p
        
        d = pilotSymbols(p);    % simbolo pilota noto (riferimento)
        
        % Output del beamformer
        y = w(:,p)' * Xp;
        
        % Errore
        e = d - y;
        
        % Normalizzazione (NLMS)
        norm_factor = Xp'*Xp + 1e-6;
        step = mu / norm_factor;
        
        % Aggiornamento peso
        w(:,p) = w(:,p) + step * Xp * conj(e);
        
        % NORMALIZZAZIONE DEI PESI per stabilità
        w(:,p) = w(:,p) / (norm(w(:,p)) + 1e-10);
    end
    
    % ========================================
    % CALCOLA PATTERN DI RADIAZIONE
    % ========================================
    
    % Media dei pesi su tutti i pilot
    w_avg = mean(w, 2);  % [Nant x 1]
    
    % Calcola pattern usando il peso medio
    pattern_az = abs(w_avg' * sv_az).^2;
    pattern_el = abs(w_avg' * sv_el).^2;
    
    % Normalizzazione in dB
    pattern_azdB = 10*log10(pattern_az / max(pattern_az) + 1e-10);
    pattern_eldB = 10*log10(pattern_el / max(pattern_el) + 1e-10);

    % ========================================
    % AGGIORNA VISUALIZZAZIONE
    % ========================================
    
    % aggiorna 3D UE/BS
    set(UEplot, ...
        'XData', UE_pos_hist(1,1:frame), ...
        'YData', UE_pos_hist(2,1:frame), ...
        'ZData', UE_pos_hist(3,1:frame));

    % aggiorna polar azimuth
    set(azPlot, 'ThetaData', deg2rad(theta), 'RData', pattern_azdB);

    % aggiorna polar elevation
    set(elPlot, 'ThetaData', deg2rad(phi), 'RData', pattern_eldB);

    % info testo
    set(infoText, 'String', sprintf(['Frame %d / %d\nTime = %.1f s\n' ...
                                     'Az = %.1f°, El = %.1f°\n' ...
                                     'UE pos: [%.1f, %.1f, %.1f] m'], ...
        frame, Pars.numFrame, frame*dt, az, el, UE.pos(1), UE.pos(2), UE.pos(3)));

    drawnow limitrate;
    pause(0.05);
end

function [pl_uma_los] = f_pl_uma_los(hue,hbs,d2d,d3d,fc)
%###########################################################%
%                                                           %
%  author: Wenbo Wang @TUT, Finland                         %
%  date: 14.03.2018                                         %
%  date: 01.06.2018 version 1                               %
%  * This function calculates the pathloss in the urban     %
%  macrocell LoS scenarios                                  %
%  date: 27.07.2022 @TAU, Finland                           %
%  * This version consider 3D space                         %
%                                                           %
%###########################################################%
%###########################################################%
%                                                           %
%  hue: the altitude of user equipment [m]                  %
%  hbs: the altitude of base station [m]                    %
%  d2d: the horizontal distance [m]                         %
%  d3d: the T-R separation distance [m]                     %
%  fc: carrier frequency [GHz] n*1 or 1*n vector            %
%                                                           % 
%  pl_uma_los: the output pathlos of LoS in urban           %
%              (n*1) row*col                                %
%                                                           %
%###########################################################%
% convert fc into n*1 vector
fc=fc(:);

% speed of light
c=3e8; % or alternatively c=physconst('LightSpeed');

% conditions
cond10=+(hue>=13); % hue>=13 meters
cond11=+(hue<=22.5); % hue <=22.5 meters

% effective environment height
he=1./(1+(cond10.*cond11).*((hue-13)/10).^1.5.*(d2d>18)*5/4.*(d2d/1e2).^3.*...
    exp(-1*d2d/150));

% breakpoint distance
dbp=4*(hbs-he).*(hue-he).*fc*1e9/c;

% conditions
cond1=+(hue>=0); % hue>=0 meters
cond2=+(hue<=22.5); % hue<=22.5 meters
cond3=+(d2d>=10); % d2d>=10 meters
cond4=+(d2d<=dbp); % d2d<=breaking point
cond5=+(d2d>dbp); % d2d>breaking point
cond6=+(hue>22.5); % hue>22.5 meters

% in case that the altitude of aircraft is zero
if dbp<=0
    cond4=ones(cond5);
    cond5=zeros(cond5);
    
    dbp=eps;
end

% path loss components
pl1=cond3.*cond4.*(28+22*log10(d3d)+20*log10(fc));
pl2=cond5.*(28+40*log10(d3d)+20*log10(fc)-9*log10(dbp.^2+(hbs-hue).^2));
pl3=cond6.*(28+22*log10(d3d)+20*log10(fc));

% overall LoS path loss
pl_uma_los=cond1.*cond2.*(pl1+pl2 )+pl3;

end