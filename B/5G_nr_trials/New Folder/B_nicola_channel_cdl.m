close all; clear; clc;

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

% Noise & Power
kB = physconst('Boltzmann');
T_amb = 290;                        
NF_dB = 7;                          
noise_bw = scs;                     
noise_power_per_sc = kB * T_amb * noise_bw * 10^(NF_dB/10); 

TxPower_Total_dBm = 23;  % Aumentato da 3 a 23 dBm per avere SNR migliore            
TxPower_Total_Lin = 10^((TxPower_Total_dBm-30)/10);
num_active_subcarriers = length(dataIndices) + length(pilotIndices);
TxPower_per_SC = TxPower_Total_Lin / num_active_subcarriers; 
Tx_Amplitude = sqrt(TxPower_per_SC);

%% =======================
%  3) CDL CHANNEL SETUP (3GPP TR 38.901) - SIMPLIFIED
%% =======================
SR = scs * Nfft;  % 1.92 MHz

% === CHANNEL UE1 (Target) - CDL-D (Urban LOS) con parametri meno aggressivi ===
cdl1 = nrCDLChannel;
cdl1.DelayProfile = 'CDL-D';          % Urban LOS (ha componente dominante)
cdl1.DelaySpread = 30e-9;             % Ridotto a 30 ns (meno dispersivo)
cdl1.CarrierFrequency = Pars.fc;      
cdl1.MaximumDopplerShift = UE1.vel_ms / Pars.lambda;

% Array singola antenna TX, array BS RX
cdl1.TransmitAntennaArray.Size = [1 1 1 1 1];  
cdl1.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl1.ReceiveAntennaArray.ElementSpacing = [0.5 0.5 1 1];

cdl1.SampleRate = SR;
cdl1.Seed = 42;
cdl1.NormalizePathGains = false;  % Disabilitiamo normalizzazione per controllare SNR

% Direzione di movimento
az_UE1 = atan2d(UE1.dir(2), UE1.dir(1));
el_UE1 = asind(UE1.dir(3));
cdl1.UTDirectionOfTravel = [az_UE1; el_UE1];

% Configurazione angoli medi per steering verso UE1
rel1_init = UE1.pos - BS.pos;
cdl1.AngleScaling = true;
cdl1.MeanAngles = [atan2d(rel1_init(2), rel1_init(1)), ...  % AoA azimuth
                   90-acosd(rel1_init(3)/norm(rel1_init)), ... % AoA elevation
                   atan2d(-rel1_init(2), -rel1_init(1)), ...  % AoD azimuth  
                   90-acosd(-rel1_init(3)/norm(rel1_init))];  % AoD elevation

% === CHANNEL UE2 (Interferer) - CDL-C (NLOS) ===
cdl2 = nrCDLChannel;
cdl2.DelayProfile = 'CDL-C';          
cdl2.DelaySpread = 100e-9;            % Più dispersivo ma non troppo
cdl2.CarrierFrequency = Pars.fc;
cdl2.MaximumDopplerShift = abs(UE2.vel_ms) / Pars.lambda;

cdl2.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl2.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl2.ReceiveAntennaArray.ElementSpacing = [0.5 0.5 1 1];

cdl2.SampleRate = SR;
cdl2.Seed = 123;
cdl2.NormalizePathGains = false;

v_vec2_init = [-sin(UE2.angle); cos(UE2.angle); 0];
az_UE2 = atan2d(v_vec2_init(2), v_vec2_init(1));
cdl2.UTDirectionOfTravel = [az_UE2; 0];

rel2_init = UE2.pos - BS.pos;
cdl2.AngleScaling = true;
cdl2.MeanAngles = [atan2d(rel2_init(2), rel2_init(1)), ...
                   90-acosd(rel2_init(3)/norm(rel2_init)), ...
                   atan2d(-rel2_init(2), -rel2_init(1)), ...
                   90-acosd(-rel2_init(3)/norm(rel2_init))];

%% =======================
%  4) DATA GENERATION
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
TX_Grid2(pilotIndices, :) = (rand(4, nSymbols)+1j*rand(4, nSymbols)) * Tx_Amplitude;

%% =======================
%  5) GRAPHICS SETUP
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
xlabel('Symbol'); ylabel('dB'); ylim([-10 50]);

ax5 = nexttile([1 1]);
hEVM = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'm');
grid(ax5, 'on'); title('EVM');
xlabel('Symbol'); ylim([0 1.5]);

%% =======================
%  6) MAIN SIMULATION LOOP (CDL Channel - FIXED)
%% =======================
W = zeros(Nant, Nfft); W(1, :) = 1; 

% RLS Parameters (più conservativi per canale realistico)
lambda_rls = 0.98;      % Aumentato per tracking più lento
delta_rls  = 1000;      % Aumentata inizializzazione
P_rls = repmat(eye(Nant) * delta_rls, 1, 1, length(pilotIndices));

% Buffers
mse_hist = []; sinr_hist = []; const_buf = []; evm_hist = [];

fprintf('Inizio simulazione con canale CDL 3GPP...\n');

for t = 1:nSymbols
    
    % --- A. PHYSICS: POSITION UPDATE ---
    UE1.pos = UE1.pos + UE1.dir * UE1.vel_ms * dt;
    
    UE2.angle = UE2.angle + UE2.omega * dt;
    UE2.pos(1) = UE2.radius * cos(UE2.angle);
    UE2.pos(2) = UE2.radius * sin(UE2.angle);
    
    % --- B. PATH LOSS CALCULATION ---
    rel1 = UE1.pos - BS.pos;
    dist1 = norm(rel1);
    pl1_dB = 20*log10(4*pi*dist1/Pars.lambda);  % Free Space Path Loss in dB
    pl1_linear = 10^(-pl1_dB/10);
    
    rel2 = UE2.pos - BS.pos;
    dist2 = norm(rel2);
    pl2_dB = 20*log10(4*pi*dist2/Pars.lambda);
    pl2_linear = 10^(-pl2_dB/10);
    
    % --- C. GENERATE OFDM SYMBOLS ---
    tx1_vec = TX_Grid1(:, t);
    tx2_vec = TX_Grid2(:, t);
    
    % IFFT per dominio temporale
    tx1_time = ifft(tx1_vec, Nfft) * sqrt(Nfft);
    tx2_time = ifft(tx2_vec, Nfft) * sqrt(Nfft);
    
    % Aggiungi CP
    tx1_with_cp = [tx1_time(end-CPLength+1:end); tx1_time];
    tx2_with_cp = [tx2_time(end-CPLength+1:end); tx2_time];
    
    % --- D. APPLY CDL CHANNEL ---
    % Canale UE1 (con path loss)
    rx1_with_cp = cdl1(tx1_with_cp);  % Output: [Ns x Nant]
    rx1_with_cp = rx1_with_cp * sqrt(pl1_linear);
    
    % Canale UE2 (con path loss)
    rx2_with_cp = cdl2(tx2_with_cp);  % Output: [Ns x Nant]
    rx2_with_cp = rx2_with_cp * sqrt(pl2_linear);
    
    % Rimuovi CP
    rx1_no_cp = rx1_with_cp(CPLength+1:end, :);  % [Nfft x Nant]
    rx2_no_cp = rx2_with_cp(CPLength+1:end, :);
    
    % FFT per tornare in frequenza
    RX1_freq = fft(rx1_no_cp, Nfft, 1) / sqrt(Nfft);  % [Nfft x Nant]
    RX2_freq = fft(rx2_no_cp, Nfft, 1) / sqrt(Nfft);
    
    % Trasponi: [Nant x Nfft]
    RX1_freq = RX1_freq.';
    RX2_freq = RX2_freq.';
    
    % --- E. ADD NOISE ---
    noise_std = sqrt(noise_power_per_sc / 2);
    noise = noise_std * (randn(Nant, Nfft) + 1j*randn(Nant, Nfft));
    
    % Segnale totale ricevuto
    RX_Total = RX1_freq + RX2_freq + noise;
    
    % --- F. RLS BEAMFORMING ---
    
    % 1. Pilot Extraction & CPE Correction
    y_p = zeros(length(pilotIndices),1);
    for i=1:length(pilotIndices)
        k = pilotIndices(i);
        y_p(i) = W(:,k)' * RX_Total(:,k);
    end
    
    % Phase correction
    cpe = mean(angle(y_p .* conj(pilotVal * Tx_Amplitude))); 
    if isnan(cpe), cpe=0; end
    W = W * exp(1j*cpe); 
    
    % 2. RLS Weight Update sui pilot
    mse_s = 0;
    for i=1:length(pilotIndices)
        k = pilotIndices(i);     
        x = RX_Total(:, k);      
        d = pilotVal * Tx_Amplitude;  % Desired signal (reference)
        
        P_curr = P_rls(:, :, i);
        
        % Kalman gain
        Px = P_curr * x;
        denom = lambda_rls + x' * Px;
        if abs(denom) < 1e-10, denom = 1e-10; end  % Evita divisione per zero
        g = Px / denom;
        
        % Output ed errore
        y = W(:, k)' * x;
        e = d - y;
        mse_s = mse_s + abs(e)^2;
        
        % Update weights
        W(:, k) = W(:, k) + g * conj(e);
        
        % Update P matrix
        P_curr = (P_curr - g * x' * P_curr) / lambda_rls;
        P_rls(:, :, i) = P_curr;
    end
    mse_hist(end+1) = mse_s / length(pilotIndices);
    
    % 3. Interpolazione pesi per subcarrier dati
    w_p = W(:, pilotIndices).';  % [Npilots x Nant]
    W_int = zeros(Nant, Nfft);
    
    for ant = 1:Nant
        mag_i = interp1(pilotIndices, abs(w_p(:,ant)), 1:Nfft, 'linear', 'extrap');
        phs_i = interp1(pilotIndices, unwrap(angle(w_p(:,ant))), 1:Nfft, 'linear', 'extrap');
        W_int(ant, :) = mag_i .* exp(1j*phs_i);
    end
    
    % --- G. EQUALIZATION & OUTPUT ---
    y_out = zeros(length(dataIndices), 1);
    for i=1:length(dataIndices)
        k = dataIndices(i);
        y_out(i) = W_int(:,k)' * RX_Total(:,k);
    end
    
    % Normalizzazione (AGC semplice)
    scale_factor = mean(abs(y_out)) / mean(abs(matTx1(:,t) * Tx_Amplitude));
    if scale_factor > 1e-6
        y_out = y_out / scale_factor;
    end
    
    % Buffer constellation
    const_buf = [const_buf; y_out];
    if length(const_buf)>600, const_buf=const_buf(end-600:end); end
    
    % --- H. METRICS ---
    
    % EVM
    ref_signal = matTx1(:, t) * Tx_Amplitude;
    err_v = y_out - ref_signal;
    evm_curr = sqrt(mean(abs(err_v).^2)) / sqrt(mean(abs(ref_signal).^2));
    evm_hist(end+1) = evm_curr;
    
    % SINR (stima semplificata)
    P_signal = mean(abs(y_out).^2);
    P_error = mean(abs(err_v).^2);
    if P_error > 1e-12
        sinr_db = 10*log10(P_signal / P_error);
    else
        sinr_db = 50;
    end
    sinr_hist(end+1) = sinr_db;
    
    % --- I. PLOTTING ---
    if mod(t, 20) == 0
        set(hUE1, 'XData', UE1.pos(1), 'YData', UE1.pos(2), 'ZData', UE1.pos(3));
        set(hUE2, 'XData', UE2.pos(1), 'YData', UE2.pos(2), 'ZData', UE2.pos(3));
        
        new_traj1_x = [get(hTraj1,'XData'), UE1.pos(1)];
        new_traj1_y = [get(hTraj1,'YData'), UE1.pos(2)];
        new_traj1_z = [get(hTraj1,'ZData'), UE1.pos(3)];
        set(hTraj1, 'XData', new_traj1_x, 'YData', new_traj1_y, 'ZData', new_traj1_z);
        
        new_traj2_x = [get(hTraj2,'XData'), UE2.pos(1)];
        new_traj2_y = [get(hTraj2,'YData'), UE2.pos(2)];
        new_traj2_z = [get(hTraj2,'ZData'), UE2.pos(3)];
        set(hTraj2, 'XData', new_traj2_x, 'YData', new_traj2_y, 'ZData', new_traj2_z);
        
        % Update constellation
        if ~isempty(const_buf)
            pt = const_buf(~isnan(const_buf));
            if ~isempty(pt)
                set(hConst, 'XData', real(pt), 'YData', imag(pt));
            end
        end
        
        set(hMSE, 'XData', 1:length(mse_hist), 'YData', mse_hist);
        set(hSINR, 'XData', 1:length(sinr_hist), 'YData', sinr_hist);
        set(hEVM, 'XData', 1:length(evm_hist), 'YData', evm_hist);
        
        set(hText, 'String', sprintf('Symbol: %d/%d\nDist UE1: %.1f m\nSINR: %.1f dB\nEVM: %.2f%%\nCDL: D(LOS)/C(NLOS)', ...
            t, nSymbols, dist1, sinr_db, evm_curr*100));
        drawnow limitrate;
        
        if mod(t, 500) == 0
            fprintf('Progress: %d/%d symbols (%.1f%%) - SINR: %.1f dB\n', ...
                t, nSymbols, 100*t/nSymbols, sinr_db);
        end
    end
end

fprintf('\nSimulazione completata!\n');
fprintf('SINR medio finale: %.1f dB\n', mean(sinr_hist(end-100:end)));
fprintf('EVM medio finale: %.2f%%\n', mean(evm_hist(end-100:end))*100);