close all; clear; clc;

%% =======================
%  1) OFDM & SYSTEM PARAMETERS 
%% =======================
scs = 30e3; % 50Mhz          
M   = 16;               
Nfft = 2048;              
CPLength = 512;          
Ns = Nfft + CPLength;   
GuardBands = [191; 191];
start_active = GuardBands(1) + 1;
end_active = Nfft - GuardBands(2);
pilotIndices = (start_active:14:end_active).';
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
BS.pos = [0; 0; 10];    

% UE 1: TARGET 
UE1.pos = [-10;-10; 1.5]; 
UE1.vel_kmh = 50;        
UE1.vel_ms = UE1.vel_kmh / 3.6;
UE1.dir = [1; 0.5; 0]; 
UE1.dir = UE1.dir / norm(UE1.dir);

% UE 2: INTERFERER
UE2.radius = 20;        
UE2.angle = 0;          
UE2.vel_kmh = -25;       
UE2.vel_ms = UE2.vel_kmh / 3.6;
UE2.omega = UE2.vel_ms / UE2.radius; 
UE2.pos = [UE2.radius;0; 1.5];

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

bitsTx2 = randi([0 1], NumDataCarriers * log2(M) * nSymbols, 1);
symTx2 = qammod(bitsTx2, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
matTx2 = reshape(symTx2, NumDataCarriers, nSymbols);

TX_Grid1 = zeros(Nfft, nSymbols);
TX_Grid1(dataIndices, :) = matTx1 * Tx_Amplitude;   
TX_Grid1(pilotIndices, :) = pilotVal * Tx_Amplitude; 

TX_Grid2 = zeros(Nfft, nSymbols);
TX_Grid2(dataIndices, :) = matTx2 * Tx_Amplitude; 
TX_Grid2(pilotIndices, :) = (rand(numel(pilotIndices), nSymbols)+1j*rand(numel(pilotIndices), nSymbols)) * Tx_Amplitude;

%% =======================
%  4) GRAPHICS SETUP
%% =======================
fig = figure('WindowState', 'maximized', 'Color', 'w');
tiledlayout(2, 3, 'TileSpacing', 'compact');

ax1 = nexttile([1 1]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
plot3(BS.pos(1), BS.pos(2), BS.pos(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor','r');
hUE1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), 'bo', 'MarkerFaceColor','b', 'DisplayName', 'Target');
hTraj1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), 'b-', 'LineWidth', 0.5);
hUE2 = plot3(UE2.pos(1), UE2.pos(2), UE2.pos(3), 'mo', 'MarkerFaceColor','m', 'DisplayName', 'Interferer');
hTraj2 = plot3(UE2.pos(1), UE2.pos(2), UE2.pos(3), 'm-', 'LineWidth', 0.5);
xlabel('X'); ylabel('Y'); title('Scenario: Target (Blue) vs Interferer (Magenta)');
view(2); xlim([-50 60]); ylim([-40 40]); legend([hUE1, hUE2]);

ax2 = nexttile([1 1]);
hConst = plot(NaN, NaN, '.', 'Color', [0 0.4 0.8], 'MarkerSize', 4);
grid(ax2, 'on'); axis(ax2, 'square'); xlim([-2 2]); ylim([-2 2]);
title('RX Constellation (RLS Beamforming)');
xline(0); yline(0);

axInfo = nexttile([1 1]); axis(axInfo, 'off');
hText = text(0.1, 0.5, '', 'FontSize', 12, 'Interpreter', 'none');

ax3 = nexttile([1 1]);
hMSE = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r');
grid(ax3, 'on'); title('RLS Cost Function (MSE)');
xlabel('Symbol'); ylabel('MSE'); ylim([1e-5 10]);

ax4 = nexttile([1 1]);
hSINR = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g');
grid(ax4, 'on'); title('Output SINR (Post-Beamforming)');
xlabel('Symbol'); ylabel('dB'); ylim([0 60]); % Alzato il limite a 60dB per RLS

ax5 = nexttile([1 1]);
hEVM = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'm');
grid(ax5, 'on'); title('EVM');
xlabel('Symbol'); ylim([0 1.5]);

%% =======================
%  5) MAIN SIMULATION LOOP (With Element Pattern Fix)
%% =======================
W = zeros(Nant, Nfft); W(1, :) = 1; % Initialize weights

% RLS Parameters
lambda_rls = 0.94;      % Forgetting factor
delta_rls  = 100;       % Init value for P matrix
P_rls = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));

% Buffers
mse_hist = []; sinr_hist = []; const_buf = []; evm_hist = [];
accum_phase1 = zeros(nPaths, 1);
accum_phase2 = zeros(nPaths, 1);

% Pre-calculate UPA element positions
[y_idx, z_idx] = meshgrid(0:antSize(1)-1, 0:antSize(2)-1);
y_pos = y_idx(:) * Pars.lambda/2;
z_pos = z_idx(:) * Pars.lambda/2;

for t = 1:nSymbols
    
    % --- A. PHYSICS: POSITION UPDATE ---
    % Update UE1 (Target) Position - Linear Motion
    UE1.pos = UE1.pos + UE1.dir * UE1.vel_ms * dt;
    
    % Update UE2 (Interferer) Position - Circular Motion
    UE2.angle = UE2.angle + UE2.omega * dt;
    UE2.pos(1) = UE2.radius * cos(UE2.angle);
    UE2.pos(2) = UE2.radius * sin(UE2.angle);
    
    % --- B. CHANNEL GENERATION (Multipath + Doppler + Element Pattern) ---
    
    % === Channel UE1 (Target) ===
    rel1 = UE1.pos - BS.pos;     % Relative position vector
    dist1 = norm(rel1);          % Distance
    pl1 = (Pars.lambda / (4*pi*dist1))^2; % Free Space Path Loss
    H1 = zeros(Nant, Nfft);
    
    % Calculate Angle of Arrival relative to Array Boresight (X-axis)
    % Ideally, we use the LOS angle for the Element Pattern
    az_geo = atan2(rel1(2), rel1(1));
    
    % Element Pattern Logic:
    % 1. Backplane: If x < 0 (|az| > 90), attenuate signal (back of array)
    % 2. Directivity: If x > 0, apply cosine pattern (max gain at boresight)
    if abs(rad2deg(az_geo)) > 90
        elem_gain = 1e-4; % -80 dB attenuation (Backplane blocks signal)
    else
        elem_gain = cos(az_geo); % Cosine pattern (Patch antenna)
    end
    
    for p = 1:nPaths
        % Determine Azimuth/Elevation (LOS or Environmental Scatterer)
        az = (p==1)*atan2(rel1(2),rel1(1)) + (p>1)*env_az(p);
        el = (p==1)*asin(rel1(3)/dist1)    + (p>1)*env_el(p);
        
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
        H1 = H1 + (path_gains(p) * elem_gain * exp(1j*accum_phase1(p)) * sv) * del;
    end
    H1 = H1 * sqrt(pl1);
    
    % === Channel UE2 (Interferer) ===
    % Tangential velocity vector for circular motion
    v_vec2 = [-UE2.omega*UE2.pos(2); UE2.omega*UE2.pos(1); 0]; 
    
    rel2 = UE2.pos - BS.pos; 
    dist2 = norm(rel2);
    pl2 = (Pars.lambda / (4*pi*dist2))^2;
    H2 = zeros(Nant, Nfft);
    
    % Element Pattern for UE2
    az_geo2 = atan2(rel2(2), rel2(1));
    if abs(rad2deg(az_geo2)) > 90
        elem_gain2 = 1e-4; % Backplane attenuation
    else
        elem_gain2 = cos(az_geo2);
    end
    
    for p = 1:nPaths
        az = (p==1)*atan2(rel2(2),rel2(1)) + (p>1)*env_az(p);
        el = (p==1)*asin(rel2(3)/dist2)    + (p>1)*env_el(p);
        
        dop = (dot(v_vec2, [cos(el)*cos(az); cos(el)*sin(az); sin(el)])) / Pars.lambda;
        accum_phase2(p) = accum_phase2(p) + 2*pi*dop*dt;
        
        ky = sin(az)*cos(el); kz = sin(el);
        sv = exp(1j * 2*pi/Pars.lambda * (y_pos*ky + z_pos*kz));
        del = exp(-1j*2*pi*(0:Nfft-1).'*path_delays_samples(p)/Nfft).';
        
        H2 = H2 + (path_gains(p) * elem_gain2 * exp(1j*accum_phase2(p)) * sv) * del;
    end
    H2 = H2 * sqrt(pl2);
    
    % --- C. RECEPTION (Signal + Interference + Noise) ---
    % Generate Thermal Noise (scaled per subcarrier)
    noise_std = sqrt(noise_power_per_sc / 2);
    noise = noise_std * (randn(Nant, Nfft) + 1j*randn(Nant, Nfft));
    
    tx1_vec = TX_Grid1(:, t);
    tx2_vec = TX_Grid2(:, t);
    
    % Total Received Signal
    RX_Total = zeros(Nant, Nfft);
    for k = 1:Nfft
        RX_Total(:,k) = H1(:,k)*tx1_vec(k) + H2(:,k)*tx2_vec(k) + noise(:,k);
    end
    
    % --- D. RLS BEAMFORMING ---
    
    % 1. Pilot Extraction & CPE Correction
    y_p = zeros(length(pilotIndices),1);
    for i=1:length(pilotIndices)
        k = pilotIndices(i);
        y_p(i) = W(:,k)' * RX_Total(:,k);
    end
    
    % Phase De-rotation (Locking to Pilot Phase)
    cpe = mean(angle(y_p .* conj(pilotVal))); 
    if isnan(cpe), cpe=0; end
    W = W * exp(1j*cpe); 
    
    % 2. RLS Update
    mse_s = 0;
    for i=1:length(pilotIndices)
        k = pilotIndices(i);     
        x = RX_Total(:, k);      
        
        % Retrieve Inverse Correlation Matrix for this pilot
        P_curr = P_rls(:, :, i);
        
        % Calculate Kalman Gain
        Px = P_curr * x;
        g = Px / (lambda_rls + x' * Px);
        
        % Calculate Output & Error
        y = W(:, k)' * x;
        e = pilotVal - y;
        mse_s = mse_s + abs(e)^2;
        
        % Update Weights
        W(:, k) = W(:, k) + g * conj(e);
        
        % Update P Matrix (Inverse Correlation)
        P_curr = (P_curr - g * x' * P_curr) / lambda_rls;
        P_rls(:, :, i) = P_curr;
    end
    mse_hist(end+1) = mse_s;
    
    % 3. Interpolation (Frequency Domain)
    w_p = W(:, pilotIndices).';
    mag_i = interp1(pilotIndices, abs(w_p), 1:Nfft, 'linear', 'extrap');
    phs_i = interp1(pilotIndices, unwrap(angle(w_p)), 1:Nfft, 'linear', 'extrap');
    W_int = (mag_i .* exp(1j*phs_i)).';
    
    % --- E. SINR CALCULATION ---
    sinr_avg = 0;
    for idx = 1:length(dataIndices)
        k = dataIndices(idx);
        w_k = W_int(:, k);
        
        % Calculate Powers (Including Tx Power Scaling)
        P_S = abs(w_k' * H1(:,k))^2 * TxPower_per_SC; 
        P_I = abs(w_k' * H2(:,k))^2 * TxPower_per_SC; 
        P_N = (norm(w_k)^2) * noise_power_per_sc;
        
        sinr_avg = sinr_avg + P_S / (P_I + P_N);
    end
    sinr_db = 10*log10(sinr_avg / length(dataIndices));
    sinr_hist(end+1) = sinr_db;
    
    % --- F. OUTPUT & METRICS ---
    y_out = zeros(length(dataIndices), 1);
    for i=1:length(dataIndices)
        k = dataIndices(i);
        y_out(i) = W_int(:,k)' * RX_Total(:,k);
    end
    
    % Buffer Update
    const_buf = [const_buf; y_out];
    if length(const_buf)>600, const_buf=const_buf(end-600:end); end
    
    % EVM Calculation
    err_v = y_out - matTx1(:, t);
    evm_curr = mean(abs(err_v).^2);
    evm_hist(end+1) = evm_curr;
    
    % --- G. PLOTTING ---
    if mod(t, 20) == 0
        % Update Map
        set(hUE1, 'XData', UE1.pos(1), 'YData', UE1.pos(2), 'ZData', UE1.pos(3));
        set(hUE2, 'XData', UE2.pos(1), 'YData', UE2.pos(2), 'ZData', UE2.pos(3));
        
        % Update Trajectories
        new_traj1_x = [get(hTraj1,'XData'), UE1.pos(1)];
        new_traj1_y = [get(hTraj1,'YData'), UE1.pos(2)];
        new_traj1_z = [get(hTraj1,'ZData'), UE1.pos(3)];
        set(hTraj1, 'XData', new_traj1_x, 'YData', new_traj1_y, 'ZData', new_traj1_z);
        
        new_traj2_x = [get(hTraj2,'XData'), UE2.pos(1)];
        new_traj2_y = [get(hTraj2,'YData'), UE2.pos(2)];
        new_traj2_z = [get(hTraj2,'ZData'), UE2.pos(3)];
        set(hTraj2, 'XData', new_traj2_x, 'YData', new_traj2_y, 'ZData', new_traj2_z);
        
        % Update Constellation (Normalized)
        if ~isempty(const_buf)
            pt = const_buf(~isnan(const_buf));
            if ~isempty(pt)
                n = mean(abs(pt)); if n<1e-9, n=1; end
                set(hConst, 'XData', real(pt)/n, 'YData', imag(pt)/n);
            end
        end
        
        % Update Graphs
        set(hMSE, 'XData', 1:length(mse_hist), 'YData', mse_hist);
        set(hSINR, 'XData', 1:length(sinr_hist), 'YData', sinr_hist);
        set(hEVM, 'XData', 1:length(evm_hist), 'YData', evm_hist);
        
        % Update Text
        set(hText, 'String', sprintf('Sym: %d\nDist: %.1fm\nSINR: %.1fdB\nEVM: %.3f', ...
            t, dist1, sinr_db, evm_curr));
        drawnow limitrate;
    end
end