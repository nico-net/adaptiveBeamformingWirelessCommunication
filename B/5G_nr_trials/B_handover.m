close all; clear; clc;
% rng(123);
%% =======================
%  1) OFDM & SYSTEM PARAMETERS
%% =======================
scs = 30e3;             
M   = 16;               
Nfft = 64;              
CPLength = 16;          
Ns = Nfft + CPLength;   
GuardBands = [6; 6];
pilotIndices = [12; 26; 40; 54]; 
pilotVal = 1+1j;        

all_indices = (1:Nfft).';
guard_lower = (1:GuardBands(1)).';
guard_upper = (Nfft - GuardBands(2) + 1 : Nfft).';
guards = [guard_lower; guard_upper];
occupied_indices = setdiff(all_indices, guards);
dataIndices = setdiff(occupied_indices, pilotIndices); 

%% =======================
%  2) GEOMETRY & PHYSICS
%% =======================
Pars.fc = 3.5e9;                    
Pars.c  = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
dt = 0.0005;                        
nSymbols = 20000;                   
antSize = [4, 4];       
Nant = prod(antSize);
BS1.pos = [0; 0; 10];    
BS2.pos = [50; 0; 10];

% === CONFIGURAZIONE CONSIGLIATA PER OSSERVARE HANDOVER ===
% UE1 parte vicino a BS1 e si muove verso BS2 lungo l'asse X
% Questo garantisce:
%   1. Partenza nella cella di BS1 (SINR_BS1 > SINR_BS2)
%   2. Attraversamento del punto medio (~25m) dove avviene l'handover
%   3. Arrivo nella cella di BS2 (SINR_BS2 > SINR_BS1)

UE1.pos = [-5; 5; 1.5];   % Parte vicino a BS1
UE1.vel_kmh = 30;         % Velocità moderata (30 km/h) per osservare bene
UE1.vel_ms = UE1.vel_kmh / 3.6;
UE1.dir = [1; 0; 0];      % Direzione pura lungo X (verso BS2)
UE1.dir = UE1.dir / norm(UE1.dir);

%% =======================
%  HANDOVER PARAMETERS
%% =======================
% Stato di connessione UE1: 1 = connesso a BS1, 2 = connesso a BS2
UE1.connected_bs = 1;  % Inizialmente connesso a BS1

% Parametri Handover
handover.hysteresis_dB = 3;      % Margine di isteresi per evitare ping-pong
handover.TTT_symbols = 10;      % Time-to-Trigger in numero di simboli (50ms @ 0.5ms/symbol)
handover.ttt_counter = 0;        % Contatore per TTT
handover.pending_target = 0;     % BS target durante TTT (0 = nessun handover pending)
handover.in_progress = false;    % Flag handover in corso
handover.execution_time = 10;    % Tempo esecuzione handover in simboli
handover.exec_counter = 0;       % Contatore esecuzione

% Log handover events
handover.log = [];               % [symbol_time, from_bs, to_bs, sinr_old, sinr_new]
handover.total_count = 0;

% UE 2: INTERFERER
UE2.radius = 20;        
UE2.angle = 0;          
UE2.vel_kmh = -25;       
UE2.vel_ms = UE2.vel_kmh / 3.6;
UE2.omega = UE2.vel_ms / UE2.radius; 
UE2.pos = [UE2.radius;0; 1.5];

% UE 3: INTERFERER
UE3.radius = 30;
UE3.angle  = pi/3;
UE3.vel_kmh = -40;
UE3.vel_ms  = UE3.vel_kmh / 3.6;
UE3.omega   = UE3.vel_ms / UE3.radius;
UE3.pos = [UE3.radius*cos(UE3.angle); UE3.radius*sin(UE3.angle); 1.5];

% Multipath
nPaths = 1;
path_gains_dB = [0, -4, -7, -10, -15];
path_gains = 10.^(path_gains_dB/20);
path_delays_samples = [0, 1, 2, 3, 4];
env_az = deg2rad([-10, 45, -60, 80, -30]); 
env_el = deg2rad([0, 5, -5, 10, 0]);

% Noise & Power
kB = physconst('Boltzmann');
T_amb = 290;                        
NF_dB = 7;                          
noise_bw = scs;                     
noise_power_per_sc = kB * T_amb * noise_bw * 10^(NF_dB/10); 

TxPower_Total_dBm = 3;              
TxPower_Total_Lin = 10^((TxPower_Total_dBm-30)/10);
num_active_subcarriers = length(dataIndices) + length(pilotIndices);
TxPower_per_SC = TxPower_Total_Lin / num_active_subcarriers; 
Tx_Amplitude = sqrt(TxPower_per_SC);

%% =======================
%  3) DATA GENERATION
%% =======================
NumDataCarriers = length(dataIndices);
bitsTx1 = randi([0 1], NumDataCarriers * log2(M) * nSymbols, 1);
symTx1 = qammod(bitsTx1, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
matTx1 = reshape(symTx1, NumDataCarriers, nSymbols);
% Interfering Data UE2 & UE3
bitsTx2 = randi([0 1], NumDataCarriers * log2(M) * nSymbols, 1);
symTx2 = qammod(bitsTx2, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
matTx2 = reshape(symTx2, NumDataCarriers, nSymbols);

bitsTx3 = randi([0 1], NumDataCarriers * log2(M) * nSymbols, 1);
symTx3  = qammod(bitsTx3, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
matTx3  = reshape(symTx3, NumDataCarriers, nSymbols);

TX_Grid1 = zeros(Nfft, nSymbols);
TX_Grid1(dataIndices, :) = matTx1 * Tx_Amplitude;   
TX_Grid1(pilotIndices, :) = pilotVal * Tx_Amplitude; 

TX_Grid2 = zeros(Nfft, nSymbols);
TX_Grid2(dataIndices, :) = matTx2 * Tx_Amplitude; 
TX_Grid2(pilotIndices, :) = (rand(4, nSymbols)+1j*rand(4, nSymbols)) * Tx_Amplitude;
TX_Grid3 = zeros(Nfft, nSymbols);
TX_Grid3(dataIndices, :) = matTx3 * Tx_Amplitude;
TX_Grid3(pilotIndices, :) = (rand(4, nSymbols)+1j*rand(4, nSymbols)) * Tx_Amplitude;

%% =======================
%  4) GRAPHICS SETUP
%% =======================
fig = figure('WindowState', 'maximized', 'Color', 'k');
tiledlayout(2, 4, 'TileSpacing', 'compact');
ax1 = nexttile([1 1]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
plot3(BS1.pos(1), BS1.pos(2), BS1.pos(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor','r');
plot3(BS2.pos(1), BS2.pos(2), BS2.pos(3), 'g^', 'MarkerSize', 10, 'MarkerFaceColor','g');
hUE1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), 'bo', 'MarkerFaceColor','b', 'DisplayName', 'Target');
hTraj1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), 'b-', 'LineWidth', 0.5);
hUE2 = plot3(UE2.pos(1), UE2.pos(2), UE2.pos(3), 'mo', 'MarkerFaceColor','m', 'DisplayName', 'Interferer');
hTraj2 = plot3(UE2.pos(1), UE2.pos(2), UE2.pos(3), 'm-', 'LineWidth', 0.5);
hUE3 = plot3(UE3.pos(1), UE3.pos(2), UE3.pos(3), 'mo', 'MarkerFaceColor','m', 'DisplayName', 'Interferer');
hTraj3 = plot3(UE3.pos(1), UE3.pos(2), UE3.pos(3), 'm-', 'LineWidth', 0.5);
% Linea di connessione UE1-BS (mostra quale BS è connessa)
hConnLine = plot3([UE1.pos(1), BS1.pos(1)], [UE1.pos(2), BS1.pos(2)], [UE1.pos(3), BS1.pos(3)], ...
    'c-', 'LineWidth', 2, 'DisplayName', 'Connection');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Scenario: Target (Blue) vs Interferers (Magenta)');
view(2); xlim([-30 90]); ylim([-20 50]); zlim([0 15]); legend([hUE1, hUE2, hUE3, hConnLine]);

ax2 = nexttile([1 1]);
hConst = plot(NaN, NaN, '.', 'Color', [1 0 0], 'MarkerSize', 4);
grid(ax2, 'on'); axis(ax2, 'square'); xlim([-2 2]); ylim([-2 2]);
title('RX Constellation (Serving BS)');
xline(0); yline(0);

ax2b = nexttile([1 1]);
hConst2 = plot(NaN, NaN, '.', 'Color', [0 0.8 0.4], 'MarkerSize', 4);
grid(ax2b, 'on'); axis(ax2b, 'square'); xlim([-2 2]); ylim([-2 2]);
title('RX Constellation (Neighbor BS - Monitoring)');
xline(0); yline(0);

axInfo = nexttile([1 1]); axis(axInfo, 'off');
hText = text(0.05, 0.5, '', 'FontSize', 11, 'Interpreter', 'none', 'Color', 'w');

ax3 = nexttile([1 1]);
hMSE1 = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hMSE2 = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
grid(ax3, 'on'); title('RLS Cost Function (MSE)');
xlabel('Symbol'); ylabel('MSE'); ylim([1e-5 10]); legend;

ax4 = nexttile([1 1]);
hSINR1 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hSINR2 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
hSINR_thresh = yline(NaN, '--', 'Color', 'y', 'LineWidth', 1, 'DisplayName', 'HO Threshold');
grid(ax4, 'on'); title('Output SINR (Post-Beamforming)');
xlabel('Symbol'); ylabel('dB'); ylim([0 60]); legend;

ax5 = nexttile([1 1]);
hEVM1 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hEVM2 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
grid(ax5, 'on'); title('EVM');
xlabel('Symbol'); ylim([0 1.5]); legend;

% Nuovo grafico per visualizzare stato connessione
ax6 = nexttile([1 1]);
hConnState = stairs(NaN, NaN, 'LineWidth', 2, 'Color', 'c');
grid(ax6, 'on'); title('UE1 Connected BS');
xlabel('Symbol'); ylabel('BS Index'); ylim([0.5 2.5]); yticks([1 2]); yticklabels({'BS1', 'BS2'});

%% =======================
%  5) MAIN SIMULATION LOOP (With Handover Logic)
%% =======================

% Initialize weights
W1 = zeros(Nant, Nfft); W1(1, :) = 1;
W2 = zeros(Nant, Nfft); W2(1, :) = 1;

% RLS Parameters
lambda_rls = 0.94;      % Forgetting factor
delta_rls  = 100;       % Init value for P matrix
P_rls1 = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));
P_rls2 = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));

% Buffers
mse_hist = []; sinr_hist = []; const_buf = []; evm_hist = [];
mse_hist2 = []; sinr_hist2 = []; const_buf2 = []; evm_hist2 = []; 
conn_state_hist = [];  % History of connection state

accum_phase1 = zeros(nPaths, 1);
accum_phase2 = zeros(nPaths, 1);
accum_phase3 = zeros(nPaths, 1);

accum_phase1_bs2 = zeros(nPaths, 1);
accum_phase2_bs2 = zeros(nPaths, 1);
accum_phase3_bs2 = zeros(nPaths, 1); 

% Pre-calculate UPA element positions
[y_idx, z_idx] = meshgrid(0:antSize(1)-1, 0:antSize(2)-1);
y_pos = y_idx(:) * Pars.lambda/2;
z_pos = z_idx(:) * Pars.lambda/2;

% Init metrics placeholders (used in plotting text)
sinr_db1 = NaN; sinr_db2 = NaN;
evm_curr1 = NaN; evm_curr2 = NaN;

fprintf('=== Simulation Started ===\n');
fprintf('Initial connection: UE1 -> BS%d\n', UE1.connected_bs);
fprintf('Handover parameters: Hysteresis=%.1f dB, TTT=%d symbols\n\n', ...
    handover.hysteresis_dB, handover.TTT_symbols);

for t = 1:nSymbols
    
    % --- A. PHYSICS: POSITION UPDATE ---
    % Update UE1 (Target) Position - Linear Motion
    UE1.pos = UE1.pos + UE1.dir * UE1.vel_ms * dt;
    
    % Update UE2 (Interferer) Position - Circular Motion
    UE2.angle = UE2.angle + UE2.omega * dt;
    UE2.pos(1) = UE2.radius * cos(UE2.angle);
    UE2.pos(2) = UE2.radius * sin(UE2.angle);

    % Update UE3 (Interferer) Position - Circular Motion
    UE3.angle = UE3.angle + UE3.omega * dt;
    UE3.pos(1) = UE3.radius * cos(UE3.angle);
    UE3.pos(2) = UE3.radius * sin(UE3.angle);
    
    % --- B. CHANNEL GENERATION (Multipath + Doppler + Element Pattern) ---
    
    % === Channel UE1 (Target) BS1 ===
    rel1_bs1 = UE1.pos - BS1.pos;     % Relative position vector
    dist1_bs1 = norm(rel1_bs1);          % Distance
    pl1_bs1 = (Pars.lambda / (4*pi*dist1_bs1))^2; % Free Space Path Loss
    H1_BS1 = zeros(Nant, Nfft);
    
    % Calculate Angle of Arrival relative to Array Boresight (X-axis)
    % Use LOS angle for the Element Pattern (BS1)
    az_geo_bs1 = atan2(rel1_bs1(2), rel1_bs1(1));
    
    % Element Pattern Logic:
    % 1. Backplane: If x < 0 (|az| > 90), attenuate signal (back of array)
    % 2. Directivity: If x > 0, apply cosine pattern (max gain at boresight)
    if abs(rad2deg(az_geo_bs1)) > 90
        elem_gain_bs1 = 1e-4; % -80 dB attenuation (Backplane blocks signal)
    else
        elem_gain_bs1 = cos(az_geo_bs1); % Cosine pattern (Patch antenna)
    end
    
    
    for p = 1:nPaths
        % Determine Azimuth/Elevation (LOS or Environmental Scatterer)
        az = (p==1)*atan2(rel1_bs1(2),rel1_bs1(1)) + (p>1)*env_az(p);
        el = (p==1)*asin(rel1_bs1(3)/dist1_bs1)    + (p>1)*env_el(p);
        
        % Calculate Doppler Shift
        % Project velocity vector onto the wave direction vector
        k_wave = [cos(el)*cos(az); cos(el)*sin(az); sin(el)];
        dop = (dot(UE1.dir*UE1.vel_ms, k_wave)) / Pars.lambda;
        accum_phase1(p) = accum_phase1(p) + 2*pi*dop*dt;
        
        % UPA Steering Vector Calculation
        % Phase shift depends on Y and Z positions of elements
        ky = sin(az)*cos(el); 
        kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        
        % Frequency Domain Delay
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        
        % Accumulate Channel Component
        % Note: We multiply by 'elem_gain' to simulate physical antenna properties
        H1_BS1 = H1_BS1 + (path_gains(p) * elem_gain_bs1 * exp(1j*accum_phase1(p)) * sv) * del;
    end
    H1_BS1 = H1_BS1 * sqrt(pl1_bs1);

    
    % === Channel UE1 (Target) BS2 ===
    rel1_bs2 = UE1.pos - BS2.pos;           % Relative position vector
    dist1_bs2 = norm(rel1_bs2);             % Distance
    pl1_bs2 = (Pars.lambda / (4*pi*dist1_bs2))^2; % Free Space Path Loss
    H1_BS2 = zeros(Nant, Nfft);
    
    for p = 1:nPaths
        % Determine Azimuth/Elevation (LOS or Environmental Scatterer)
        % BS2 rotated 180 degrees
        az = (p==1)*(atan2(rel1_bs2(2),rel1_bs2(1)) + pi) + (p>1)*(env_az(p) + pi);
        el = (p==1)*asin(rel1_bs2(3)/dist1_bs2)    + (p>1)*env_el(p);
        
        % Element Pattern usando l'azimut DOPO la rotazione
        % Normalizza azimut in [-pi, pi]
        az_norm = mod(az + pi, 2*pi) - pi;
        if abs(az_norm) > pi/2
            elem_gain_bs2 = 1e-4; % Backplane
        else
            elem_gain_bs2 = cos(az_norm); % Cosine taper
        end
        
        k_wave = [cos(el)*cos(az); cos(el)*sin(az); sin(el)];
        
        % Calculate Doppler Shift
        dop = dot(UE1.dir * UE1.vel_ms, k_wave) / Pars.lambda;
        accum_phase1_bs2(p) = accum_phase1_bs2(p) + 2*pi*dop*dt;
        
        % UPA Steering Vector Calculation
        ky = sin(az)*cos(el); 
        kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        
        % Frequency Domain Delay
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        
        % Accumulate Channel Component
        % Note: We multiply by 'elem_gain' to simulate physical antenna properties
        H1_BS2 = H1_BS2 + (path_gains(p) * elem_gain_bs2 * exp(1j*accum_phase1_bs2(p)) * sv) * del;
    end
    
    H1_BS2 = H1_BS2 * sqrt(pl1_bs2);

    
    % === Channel UE2 (Interferer) ===
    % Tangential velocity vector for circular motion
    v_vec2 = [-UE2.omega*UE2.pos(2); UE2.omega*UE2.pos(1); 0]; 
    
    % UE2 BS1
    rel2 = UE2.pos - BS1.pos; 
    dist2 = norm(rel2);
    pl2 = (Pars.lambda / (4*pi*dist2))^2;
    H2_BS1 = zeros(Nant, Nfft);

    % Element Pattern for UE2 BS1
    az_geo2 = atan2(rel2(2), rel2(1));
    if abs(rad2deg(az_geo2)) > 90
        elem_gain2_bs1 = 1e-4; % Backplane attenuation
    else
        elem_gain2_bs1 = cos(az_geo2);
    end

    for p = 1:nPaths
        az = (p==1)*atan2(rel2(2),rel2(1)) + (p>1)*env_az(p);
        el = (p==1)*asin(rel2(3)/dist2)    + (p>1)*env_el(p);
        
        dop = (dot(v_vec2, [cos(el)*cos(az); cos(el)*sin(az); sin(el)])) / Pars.lambda;
        accum_phase2(p) = accum_phase2(p) + 2*pi*dop*dt;
        
        ky = sin(az)*cos(el); kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        
        H2_BS1 = H2_BS1 + (path_gains(p) * elem_gain2_bs1 * exp(1j*accum_phase2(p)) * sv) * del;
    end
    H2_BS1 = H2_BS1 * sqrt(pl2);

    % UE2 BS2
    rel2_bs2 = UE2.pos - BS2.pos;
    dist2_bs2 = norm(rel2_bs2);
    pl2_bs2 = (Pars.lambda / (4*pi*dist2_bs2))^2;
    H2_BS2 = zeros(Nant, Nfft);
    
    for p = 1:nPaths
        % BS2 rotated 180 degrees
        az = (p==1)*(atan2(rel2_bs2(2),rel2_bs2(1)) + pi) + (p>1)*(env_az(p) + pi);
        el = (p==1)*asin(rel2_bs2(3)/dist2_bs2)    + (p>1)*env_el(p);

        % Element Pattern coerente
        az_norm = mod(az + pi, 2*pi) - pi;
        if abs(az_norm) > pi/2
            elem_gain2_bs2 = 1e-4;
        else
            elem_gain2_bs2 = cos(az_norm);
        end

        dop = (dot(v_vec2, [cos(el)*cos(az); cos(el)*sin(az); sin(el)])) / Pars.lambda;
        accum_phase2_bs2(p) = accum_phase2_bs2(p) + 2*pi*dop*dt;

        ky = sin(az)*cos(el); kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';

        H2_BS2 = H2_BS2 + (path_gains(p) * elem_gain2_bs2 * exp(1j*accum_phase2_bs2(p)) * sv) * del;
    end
    H2_BS2 = H2_BS2 * sqrt(pl2_bs2);
    
    

    % Element Pattern for UE3
    v_vec3 = [-UE3.omega*UE3.pos(2); UE3.omega*UE3.pos(1); 0];

    % UE3 BS1
    rel3 = UE3.pos - BS1.pos; 
    dist3 = norm(rel3);
    pl3 = (Pars.lambda / (4*pi*dist3))^2;
    H3_BS1 = zeros(Nant, Nfft);
    az_geo3 = atan2(rel3(2), rel3(1));
    if abs(rad2deg(az_geo3)) > 90, elem_gain3_bs1 = 1e-4; else, elem_gain3_bs1 = cos(az_geo3); end
    for p = 1:nPaths
        az = (p==1)*atan2(rel3(2),rel3(1)) + (p>1)*env_az(p);
        el = (p==1)*asin(rel3(3)/dist3)    + (p>1)*env_el(p);
        dop = (dot(v_vec3, [cos(el)*cos(az); cos(el)*sin(az); sin(el)])) / Pars.lambda;
        accum_phase3(p) = accum_phase3(p) + 2*pi*dop*dt;
        ky = sin(az)*cos(el); kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        H3_BS1 = H3_BS1 + (path_gains(p) * elem_gain3_bs1 * exp(1j*accum_phase3(p)) * sv) * del;
    end
    H3_BS1 = H3_BS1 * sqrt(pl3);
    
    % UE3 BS2
    v_vec3 = [-UE3.omega*UE3.pos(2); UE3.omega*UE3.pos(1); 0];
    rel3_bs2 = UE3.pos - BS2.pos;
    dist3_bs2 = norm(rel3_bs2);
    pl3_bs2 = (Pars.lambda / (4*pi*dist3_bs2))^2;
    H3_BS2 = zeros(Nant, Nfft);
    
    for p = 1:nPaths
        % BS2 rotated 180 degrees
        az = (p==1)*(atan2(rel3_bs2(2),rel3_bs2(1)) + pi) + (p>1)*(env_az(p) + pi);
        el = (p==1)*asin(rel3_bs2(3)/dist3_bs2)    + (p>1)*env_el(p);
        
        % Element Pattern coerente
        az_norm = mod(az + pi, 2*pi) - pi;
        if abs(az_norm) > pi/2
            elem_gain3_bs2 = 1e-4;
        else
            elem_gain3_bs2 = cos(az_norm);
        end
        
        dop = (dot(v_vec3, [cos(el)*cos(az); cos(el)*sin(az); sin(el)])) / Pars.lambda;
        accum_phase3_bs2(p) = accum_phase3_bs2(p) + 2*pi*dop*dt;
        ky = sin(az)*cos(el); kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        H3_BS2 = H3_BS2 + (path_gains(p) * elem_gain3_bs2 * exp(1j*accum_phase3_bs2(p)) * sv) * del;
    end
    H3_BS2 = H3_BS2 * sqrt(pl3_bs2);


    
    % --- C. RECEPTION (Signal + Interference + Noise) ---
    % Generate Thermal Noise (scaled per subcarrier)
    noise_std = sqrt(noise_power_per_sc / 2);
    noise_BS1 = noise_std * (randn(Nant, Nfft) + 1j*randn(Nant, Nfft));
    noise_BS2 = noise_std * (randn(Nant, Nfft) + 1j*randn(Nant, Nfft));
    
    tx1_vec = TX_Grid1(:, t);
    tx2_vec = TX_Grid2(:, t);
    tx3_vec = TX_Grid3(:, t);
    
    % Total Received Signal BS1
    RX_BS1 = zeros(Nant, Nfft);
    for k = 1:Nfft
        RX_BS1(:,k) = H1_BS1(:,k)*tx1_vec(k) + H2_BS1(:,k)*tx2_vec(k) + H3_BS1(:,k)*tx3_vec(k) + noise_BS1(:,k);
    end
    
    % Total Received Signal BS2
    RX_BS2 = zeros(Nant, Nfft);
    for k = 1:Nfft
        RX_BS2(:,k) = H1_BS2(:,k)*tx1_vec(k) + H2_BS2(:,k)*tx2_vec(k) + H3_BS2(:,k)*tx3_vec(k) + noise_BS2(:,k);
    end
    
    % --- D. RLS BEAMFORMING (SOLO PER BS CONNESSA) ---
    % La BS connessa fa beamforming attivo sull'UE1
    % La BS non connessa mantiene i pesi per monitoring (ma non li usa per trasmettere)
    
    mse_s1 = 0;
    mse_s2 = 0;
    
    if UE1.connected_bs == 1 || handover.in_progress
        % BS1 è la serving cell OPPURE handover in corso (entrambe attive temporaneamente)
        
        % --- RLS BEAMFORMING BS1 ---
        y_p1 = zeros(length(pilotIndices),1);
        for i=1:length(pilotIndices)
            k = pilotIndices(i);
            y_p1(i) = W1(:,k)' * RX_BS1(:,k);
        end
        cpe1 = mean(angle(y_p1 .* conj(pilotVal))); 
        if isnan(cpe1), cpe1=0; end
        W1 = W1 * exp(1j*cpe1);

        for i=1:length(pilotIndices)
            k = pilotIndices(i);
            x = RX_BS1(:, k);
            P_curr = P_rls1(:, :, i);
            Px = P_curr * x;
            g = Px / (lambda_rls + x' * Px);
            y1 = W1(:, k)' * x;
            e1 = pilotVal - y1;
            mse_s1 = mse_s1 + abs(e1)^2;
            W1(:, k) = W1(:, k) + g * conj(e1);
            P_curr = (P_curr - g * x' * P_curr) / lambda_rls;
            P_rls1(:, :, i) = P_curr;
        end
    end
    
    if UE1.connected_bs == 2 || handover.in_progress
        % BS2 è la serving cell OPPURE handover in corso
        
        % --- RLS BEAMFORMING BS2 ---
        y_p2 = zeros(length(pilotIndices),1);
        for i=1:length(pilotIndices)
            k = pilotIndices(i);
            y_p2(i) = W2(:,k)' * RX_BS2(:,k);
        end
        cpe2 = mean(angle(y_p2 .* conj(pilotVal))); 
        if isnan(cpe2), cpe2=0; end
        W2 = W2 * exp(1j*cpe2);

        for i=1:length(pilotIndices)
            k = pilotIndices(i);
            x2 = RX_BS2(:, k);
            P_curr2 = P_rls2(:, :, i);
            Px2 = P_curr2 * x2;
            g2 = Px2 / (lambda_rls + x2' * Px2);
            y2 = W2(:, k)' * x2;
            e2 = pilotVal - y2;
            mse_s2 = mse_s2 + abs(e2)^2;
            W2(:, k) = W2(:, k) + g2 * conj(e2);
            P_curr2 = (P_curr2 - g2 * x2' * P_curr2) / lambda_rls;
            P_rls2(:, :, i) = P_curr2;
        end
    end
    
    mse_hist(end+1) = max(mse_s1, 1e-10);
    mse_hist2(end+1) = max(mse_s2, 1e-10);

    % Interpolazione pesi per tutte le sottoportanti
    w_p1 = W1(:, pilotIndices).';
    mag_i1 = interp1(pilotIndices, abs(w_p1), 1:Nfft, 'linear', 'extrap');
    phs_i1 = interp1(pilotIndices, unwrap(angle(w_p1)), 1:Nfft, 'linear', 'extrap');
    W1_int = (mag_i1 .* exp(1j*phs_i1)).';

    w_p2 = W2(:, pilotIndices).';
    mag_i2 = interp1(pilotIndices, abs(w_p2), 1:Nfft, 'linear', 'extrap');
    phs_i2 = interp1(pilotIndices, unwrap(angle(w_p2)), 1:Nfft, 'linear', 'extrap');
    W2_int = (mag_i2 .* exp(1j*phs_i2)).';
    
    % --- E. OUTPUT & METRICS (BS1/BS2) ---
    % Beamformer outputs per subcarrier (per monitoring SINR di entrambe)
    y_out1 = sum(conj(W1_int) .* RX_BS1, 1);
    y_out2 = sum(conj(W2_int) .* RX_BS2, 1);

    % SINR estimation per BS: average power on pilots (signal) vs interferers+noise
    % Decompose received components to estimate powers
    sig1 = sum(conj(W1_int) .* (H1_BS1 .* TX_Grid1(:, t).'), 1);
    int1 = sum(conj(W1_int) .* (H2_BS1 .* TX_Grid2(:, t).' + H3_BS1 .* TX_Grid3(:, t).'), 1);
    noi1 = sum(conj(W1_int) .* noise_BS1, 1);
    sig2 = sum(conj(W2_int) .* (H1_BS2 .* TX_Grid1(:, t).'), 1);
    int2 = sum(conj(W2_int) .* (H2_BS2 .* TX_Grid2(:, t).' + H3_BS2 .* TX_Grid3(:, t).'), 1);
    noi2 = sum(conj(W2_int) .* noise_BS2, 1);

    % Use pilots to estimate signal power, and all subcarriers for interference+noise
    Ps1 = mean(abs(sig1(pilotIndices)).^2);
    Pi1 = mean(abs(int1(dataIndices)).^2) + mean(abs(noi1).^2);
    Ps2 = mean(abs(sig2(pilotIndices)).^2);
    Pi2 = mean(abs(int2(dataIndices)).^2) + mean(abs(noi2).^2);
    sinr_db1 = 10*log10(max(Ps1,1e-12)/max(Pi1,1e-12));
    sinr_db2 = 10*log10(max(Ps2,1e-12)/max(Pi2,1e-12));
    sinr_hist(end+1) = sinr_db1;
    sinr_hist2(end+1) = sinr_db2;

    % --- F. HANDOVER DECISION LOGIC ---
    % UE1 monitora SINR da entrambe le BS e decide se fare handover
    
    if ~handover.in_progress
        % Determina SINR della BS corrente e della BS target
        if UE1.connected_bs == 1
            sinr_current = sinr_db1;
            sinr_target = sinr_db2;
            target_bs = 2;
        else
            sinr_current = sinr_db2;
            sinr_target = sinr_db1;
            target_bs = 1;
        end
        
        % Condizione di handover: SINR_target > SINR_current + hysteresis
        handover_condition = sinr_target > (sinr_current + handover.hysteresis_dB);
        
        if handover_condition
            % Incrementa TTT counter
            if handover.pending_target == target_bs
                handover.ttt_counter = handover.ttt_counter + 1;
            else
                % Nuova BS target, reset counter
                handover.pending_target = target_bs;
                handover.ttt_counter = 1;
            end
            
            % Verifica se TTT è stato raggiunto
            if handover.ttt_counter >= handover.TTT_symbols
                % ====== INIZIO PROCEDURA HANDOVER ======
                fprintf('[Symbol %d] HANDOVER TRIGGERED: BS%d -> BS%d\n', t, UE1.connected_bs, target_bs);
                fprintf('           SINR_current=%.1f dB, SINR_target=%.1f dB (margin=%.1f dB)\n', ...
                    sinr_current, sinr_target, sinr_target - sinr_current);
                
                handover.in_progress = true;
                handover.exec_counter = 0;
                
                % Log handover event
                handover.log = [handover.log; t, UE1.connected_bs, target_bs, sinr_current, sinr_target];
                handover.total_count = handover.total_count + 1;
            end
        else
            % Condizione non soddisfatta, reset TTT
            handover.ttt_counter = 0;
            handover.pending_target = 0;
        end
    else
        % Handover in corso - simula tempo di esecuzione
        handover.exec_counter = handover.exec_counter + 1;
        
        if handover.exec_counter >= handover.execution_time
            % ====== COMPLETAMENTO HANDOVER ======
            old_bs = UE1.connected_bs;
            UE1.connected_bs = handover.pending_target;
            
            fprintf('[Symbol %d] HANDOVER COMPLETE: Now connected to BS%d\n', t, UE1.connected_bs);
            
            % Reset handover state
            handover.in_progress = false;
            handover.exec_counter = 0;
            handover.ttt_counter = 0;
            handover.pending_target = 0;
            
            % Reset RLS della vecchia BS (rilascio risorse)
            if old_bs == 1
                P_rls1 = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));
                W1 = zeros(Nant, Nfft); W1(1, :) = 1;
            else
                P_rls2 = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));
                W2 = zeros(Nant, Nfft); W2(1, :) = 1;
            end
        end
    end
    
    % Salva stato connessione
    conn_state_hist(end+1) = UE1.connected_bs;

    % --- G. CONSTELLATION & EVM (solo per BS connessa) ---
    % Usa l'output della BS attualmente connessa per constellation e EVM
    
    if UE1.connected_bs == 1
        y_out_serving = y_out1;
        pilot_out_serving = y_out1(pilotIndices).';
    else
        y_out_serving = y_out2;
        pilot_out_serving = y_out2(pilotIndices).';
    end
    
    % Constellation buffer per serving BS
    const_buf = [const_buf; y_out_serving(dataIndices).'];
    if length(const_buf) > 600, const_buf = const_buf(end-600:end); end
    
    % Constellation buffer per neighbor BS (monitoring)
    if UE1.connected_bs == 1
        const_buf2 = [const_buf2; y_out2(dataIndices).'];
    else
        const_buf2 = [const_buf2; y_out1(dataIndices).'];
    end
    if length(const_buf2) > 600, const_buf2 = const_buf2(end-600:end); end

    % EVM on data subcarriers using known transmitted symbols of UE1
    % Normalize beamformer output using pilot subcarriers to correct phase/amplitude
    pilot_ref1 = pilotVal * Tx_Amplitude;
    pilot_out1 = y_out1(pilotIndices).';
    norm_factor1 = mean(pilot_out1 ./ pilot_ref1);
    if abs(norm_factor1) < 1e-9, norm_factor1 = 1; end
    
    pilot_ref2 = pilotVal * Tx_Amplitude;
    pilot_out2 = y_out2(pilotIndices).';
    norm_factor2 = mean(pilot_out2 ./ pilot_ref2);
    if abs(norm_factor2) < 1e-9, norm_factor2 = 1; end
    
    ref1 = TX_Grid1(dataIndices, t);
    y_data1 = y_out1(dataIndices).' / norm_factor1;
    err1 = y_data1 - ref1;
    evm_curr1 = sqrt(mean(abs(err1).^2) / max(mean(abs(ref1).^2),1e-12));
    evm_hist(end+1) = evm_curr1;

    ref2 = TX_Grid1(dataIndices, t);
    y_data2 = y_out2(dataIndices).' / norm_factor2;
    err2 = y_data2 - ref2;
    evm_curr2 = sqrt(mean(abs(err2).^2) / max(mean(abs(ref2).^2),1e-12));
    evm_hist2(end+1) = evm_curr2;
    
    % --- H. PLOTTING ---
    if mod(t, 20) == 0
        % Update Map
        set(hUE1, 'XData', UE1.pos(1), 'YData', UE1.pos(2), 'ZData', UE1.pos(3));
        set(hUE2, 'XData', UE2.pos(1), 'YData', UE2.pos(2), 'ZData', UE2.pos(3));
        set(hUE3, 'XData', UE3.pos(1), 'YData', UE3.pos(2), 'ZData', UE3.pos(3));

        % Update Connection Line (mostra a quale BS è connesso UE1)
        if UE1.connected_bs == 1
            set(hConnLine, 'XData', [UE1.pos(1), BS1.pos(1)], ...
                           'YData', [UE1.pos(2), BS1.pos(2)], ...
                           'ZData', [UE1.pos(3), BS1.pos(3)], ...
                           'Color', 'r');
        else
            set(hConnLine, 'XData', [UE1.pos(1), BS2.pos(1)], ...
                           'YData', [UE1.pos(2), BS2.pos(2)], ...
                           'ZData', [UE1.pos(3), BS2.pos(3)], ...
                           'Color', 'g');
        end

        % Update Trajectories
        new_traj1_x = [get(hTraj1,'XData'), UE1.pos(1)];
        new_traj1_y = [get(hTraj1,'YData'), UE1.pos(2)];
        new_traj1_z = [get(hTraj1,'ZData'), UE1.pos(3)];
        set(hTraj1, 'XData', new_traj1_x, 'YData', new_traj1_y, 'ZData', new_traj1_z);
        
        new_traj2_x = [get(hTraj2,'XData'), UE2.pos(1)];
        new_traj2_y = [get(hTraj2,'YData'), UE2.pos(2)];
        new_traj2_z = [get(hTraj2,'ZData'), UE2.pos(3)];
        set(hTraj2, 'XData', new_traj2_x, 'YData', new_traj2_y, 'ZData', new_traj2_z);

        new_traj3_x = [get(hTraj3,'XData'), UE3.pos(1)];
        new_traj3_y = [get(hTraj3,'YData'), UE3.pos(2)];
        new_traj3_z = [get(hTraj3,'ZData'), UE3.pos(3)];
        set(hTraj3, 'XData', new_traj3_x, 'YData', new_traj3_y, 'ZData', new_traj3_z);
        
        % Update Constellation (Normalized)
        if ~isempty(const_buf)
            pt = const_buf(~isnan(const_buf));
            if ~isempty(pt)
                n = mean(abs(pt)); if n<1e-9, n=1; end
                set(hConst, 'XData', real(pt)/n, 'YData', imag(pt)/n);
            end
        end
        if ~isempty(const_buf2)
            pt2 = const_buf2(~isnan(const_buf2));
            if ~isempty(pt2)
                n2 = mean(abs(pt2)); if n2<1e-9, n2=1; end
                set(hConst2, 'XData', real(pt2)/n2, 'YData', imag(pt2)/n2);
            end
        end
        
        % Update Graphs (BS1/BS2)
        set(hMSE1, 'XData', 1:length(mse_hist),  'YData', mse_hist);
        set(hMSE2, 'XData', 1:length(mse_hist2), 'YData', mse_hist2);
        set(hSINR1,'XData', 1:length(sinr_hist),  'YData', sinr_hist);
        set(hSINR2,'XData', 1:length(sinr_hist2), 'YData', sinr_hist2);
        set(hEVM1, 'XData', 1:length(evm_hist),  'YData', evm_hist);
        set(hEVM2, 'XData', 1:length(evm_hist2), 'YData', evm_hist2);
        
        % Update Connection State graph
        set(hConnState, 'XData', 1:length(conn_state_hist), 'YData', conn_state_hist);
        
        % Determina SINR corrente e EVM corrente per display
        if UE1.connected_bs == 1
            sinr_serving = sinr_db1;
            sinr_neighbor = sinr_db2;
            evm_serving = evm_curr1;
        else
            sinr_serving = sinr_db2;
            sinr_neighbor = sinr_db1;
            evm_serving = evm_curr2;
        end
        
        % Update Text con info handover
        ho_status = '';
        if handover.in_progress
            ho_status = sprintf('HANDOVER IN PROGRESS (%d/%d)', handover.exec_counter, handover.execution_time);
        elseif handover.ttt_counter > 0
            ho_status = sprintf('TTT: %d/%d', handover.ttt_counter, handover.TTT_symbols);
        else
            ho_status = 'Stable';
        end
        
        set(hText, 'String', sprintf(['Sym: %d\n' ...
            '-------------------\n' ...
            'Connected to: BS%d\n' ...
            'Serving SINR: %.1f dB\n' ...
            'Neighbor SINR: %.1f dB\n' ...
            'Serving EVM: %.3f\n' ...
            '-------------------\n' ...
            'HO Status: %s\n' ...
            'Total Handovers: %d\n' ...
            'Hysteresis: %.1f dB'], ...
            t, UE1.connected_bs, sinr_serving, sinr_neighbor, evm_serving, ...
            ho_status, handover.total_count, handover.hysteresis_dB));
        
        drawnow limitrate;
    end
end

%% =======================
%  6) FINAL SUMMARY
%% =======================
fprintf('\n=== Simulation Complete ===\n');
fprintf('Total symbols: %d\n', nSymbols);
fprintf('Total handovers: %d\n', handover.total_count);
fprintf('Final connection: BS%d\n', UE1.connected_bs);

if ~isempty(handover.log)
    fprintf('\nHandover Log:\n');
    fprintf('%-10s %-10s %-10s %-15s %-15s\n', 'Symbol', 'From', 'To', 'SINR_old(dB)', 'SINR_new(dB)');
    for i = 1:size(handover.log, 1)
        fprintf('%-10d BS%-8d BS%-8d %-15.1f %-15.1f\n', ...
            handover.log(i,1), handover.log(i,2), handover.log(i,3), ...
            handover.log(i,4), handover.log(i,5));
    end
end
