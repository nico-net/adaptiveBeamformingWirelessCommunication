%% ==================================================================
%  NLMS BEAMFORMING + PERIODIC UPLINK JAMMER
% The Jammer acts as interference only in uplink and periodically. The goal
% is to show how the beamforming algorith behaves in response to
% considerable fluctuations in the received DMRSs. We noticed that if
% remarkable variations in the UL constellation occur the NLMS algorithm is
% not able to track properly the target and responds with an increase of
% the normalized factor mu to be more aggressive. After one or some blocks
% of slots it's able to find again the target even in strong jamming
% condition.
% It's preferable to run this code with only 1 UE, otherwise the simulation
% will render very slowly. To keep the bandwidth constant the parameter
% carrier.NSizeGrid should follow an inverse trend wrt
% carrier.SubcarrierSpacing.
%% ==================================================================
close all; clear; clc;
%% --- 1. CONFIGURATION ---
% Basic 5G NR Carrier Setup
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 30; % 30 kHz spacing
carrier.NSizeGrid = 51;         % 51 Resource Blocks (about 20MHz of Bandwidth)
carrier.CyclicPrefix = 'Normal';
carrier.NSlot = 0;
ofdmInfo = nrOFDMInfo(carrier);
K = carrier.NSizeGrid * 12;     % Total subcarriers (612)
SampleRate = ofdmInfo.SampleRate;
% --- ACTIVE BANDWIDTH (Band Edge Protection) ---
Guard_Size = 16; 
Active_Indices = (Guard_Size + 1) : (K - Guard_Size);
% Frame Structure Configuration
FRAME_STRUCT = struct();
FRAME_STRUCT.PSS_SYM = 1;
FRAME_STRUCT.DATA_SYMS = 12;
FRAME_STRUCT.GUARD_SYMS = 1;
FRAME_STRUCT.TOTAL = 14;
% TDD Pattern Setup
SlotsPerPattern = 20;
Pattern_DL = 19; % 19 Slots DL, 1 Slot UL
% DMRS (Pilot) Configuration
DMRS_positions = [2, 6, 10];      
DMRS_RE_spacing = 4;              
DMRS_indices = (Active_Indices(1):DMRS_RE_spacing:Active_Indices(end)).';
Data_indices = setdiff(Active_Indices.', DMRS_indices);
nData_DMRS_Sym = length(Data_indices);        
nData_NoDMRS_Sym = length(Active_Indices);    
M_DL = 64; % Downlink Modulation (16-QAM)
M_UL = 4;  % Uplink Modulation (QPSK)
% NLMS Adaptive Filter Parameters
mu_min = 0.05;  
mu_max = 0.9;   
alpha_mu = 0.7; %Weight for updating mu
mu_current = 0.3; 
%% --- 2. SEQUENCE GENERATION ---
% Generate Synchronization Signals (PSS/SSS)
pss_seq = zadoffChuSeq(1, 63); pss_seq = [pss_seq; zeros(K-63, 1)];
sss_seq = zadoffChuSeq(2, 63); sss_seq = [sss_seq; zeros(K-63, 1)];
sync_grid = zeros(K, 1);
sync_grid(1:K/2) = pss_seq(1:K/2); sync_grid(K/2+1:K) = sss_seq(K/2+1:K);
% Generate known DMRS Sequence
rng(42);
dmrs_seq = (2*randi([0 1], length(DMRS_indices), 1)-1) + 1j*(2*randi([0 1], length(DMRS_indices), 1)-1);
dmrs_seq = dmrs_seq / sqrt(2);
rng('shuffle');
%% --- 3. SYSTEM SETUP ---
N_UE = 1; % Fixed N_UE = 1
nTotalSlots = 2000;
Pars.fc = 6e9; % 6 GHz Carrier Frequency
slotDuration = 1e-3 / (carrier.SubcarrierSpacing/15); 
BS_Pos = [0;0;35]; %in m
AntSize = [6,6]; % 36 Antenna Elements
N_ant_BS = prod(AntSize);
% Initialize Beamforming Weights
w_beam = ones(N_ant_BS, 1) ./ sqrt(N_ant_BS); 
plCfg = nrPathLossConfig; plCfg.Scenario = 'UMa';
UEs = repmat(struct('pos', [], 'vel', [], 'channel_dl', [], 'channel_ul', [], 'hist_ber', []), N_UE, 1);
colors = lines(N_UE);
for u = 1:N_UE
    UEs(u).pos = [50; 20; 1.5]; %in m
    UEs(u).vel = [100; -100; 0]/3.6; %in m/s
    UEs(u).color = colors(u,:);  %different color for more UEs 
    
    % Configure Downlink Channel (CDL-E Model describes a dense urban scenario)
    cdl_dl = nrCDLChannel; cdl_dl.DelayProfile = 'CDL-E'; cdl_dl.DelaySpread = 30e-9;
    cdl_dl.CarrierFrequency = Pars.fc; cdl_dl.TransmitAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1];
    cdl_dl.ReceiveAntennaArray.Size = [1 1 1 1 1]; cdl_dl.SampleRate = SampleRate;
    cdl_dl.Seed = u*123; cdl_dl.ChannelFiltering = true; 
    [~,~] = cdl_dl(complex(zeros(10,N_ant_BS))); 
    
    % Configure Uplink Channel
    cdl_ul = nrCDLChannel; cdl_ul.DelayProfile = 'CDL-C'; cdl_ul.DelaySpread = 30e-9;
    cdl_ul.CarrierFrequency = Pars.fc; cdl_ul.TransmitAntennaArray.Size = [1 1 1 1 1];
    cdl_ul.ReceiveAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1]; cdl_ul.SampleRate = SampleRate;
    cdl_ul.Seed = u*123; cdl_ul.ChannelFiltering = true;
    
    UEs(u).channel_dl = cdl_dl; UEs(u).channel_ul = cdl_ul; UEs(u).hist_ber = nan(1, nTotalSlots);
end
% Transmission Power Config
TxPower_dBm_DL = 40; TxPower_dBm_UL = 30;
Tx_AmpDL = sqrt(10^((TxPower_dBm_DL-30)/10)); 
Tx_AmpUL = sqrt(10^((TxPower_dBm_UL-30)/10));
% Noise Config
NoiseFigure_dB = 7;
Temp_ant = 290; %in °K
noise_power_watts = (physconst('Boltzmann') * Temp_ant * SampleRate * 10^(NoiseFigure_dB/10));
noise_std = sqrt(noise_power_watts / 2);
NoiseFloor_dBm = 10*log10(noise_power_watts) + 30; % Used for SNR Calculation
% --- JAMMER SETUP ---
Jammer_Pos = [50; 50; 1.5]; % Fixed Position of Jammer
Jammer_Power_dBm = 36;      % Stronger than UE to be effective
Jammer_Amp = sqrt(10^((Jammer_Power_dBm-30)/20));
%% --- 4. GRAPHICS ---
figure('WindowState', 'maximized', 'Color', 'w');
tlo = tiledlayout(2, 3, 'TileSpacing', 'compact');
axEnv = nexttile; 
hold(axEnv, 'on'); grid(axEnv, 'on'); axis(axEnv, 'equal'); view(axEnv, 3);
title(axEnv, 'Scenario (Blue=BS, Red=Jammer)');
xlabel(axEnv, 'X (m)'); ylabel(axEnv, 'Y (m)'); zlabel(axEnv, 'Z (m)');
plot3(axEnv, BS_Pos(1), BS_Pos(2), BS_Pos(3), '^b', ...
      'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'b', 'DisplayName', 'BS Antenna');
plot3(axEnv, Jammer_Pos(1), Jammer_Pos(2), Jammer_Pos(3), 'xr', ...
      'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Jammer');
hUE_scatters = gobjects(N_UE,1);
for u=1:N_UE, hUE_scatters(u) = plot3(axEnv, UEs(u).pos(1), UEs(u).pos(2), UEs(u).pos(3), 'o', 'Color', UEs(u).color, 'MarkerFaceColor', UEs(u).color); end

axBER = nexttile; 
hold(axBER, 'on'); grid(axBER, 'on'); set(axBER, 'YScale', 'log');
title(axBER, 'BER Performance'); ylim([1e-6 1]); xlim([0 nTotalSlots]);
ylabel(axBER, 'Bit Error Rate (BER)');
xlabel(axBER, 'Slot index');
hBER_DL = plot(axBER, NaN, NaN, 'o-', 'Color', 'b', 'LineWidth', 1.0, 'DisplayName', 'DL BER');
hBER_UL = plot(axBER, NaN, NaN, 's-', 'Color', 'r', 'LineWidth', 1.0, 'DisplayName', 'UL BER');
legend(axBER, 'show', 'Location', 'southwest');

axSNR = nexttile; 
hold(axSNR, 'on');grid(axSNR, 'on');
ylabel(axSNR, "SNR (dB)");title(axSNR, "Theoretical SNR");xlabel(axSNR, "DL slot");
ylim(axSNR, [0 60]); xlim(axSNR, [0 nTotalSlots]); 
% Green Line = Power based (Theoretical)
hSNR_Theory = plot(axSNR, NaN, NaN, '-', 'Color', [0 0.5 0], 'LineWidth', 2, 'DisplayName', 'Theo. SNR');
% Magenta Line = EVM based (Actual Quality)
hSNR_Eff    = plot(axSNR, NaN, NaN, '-', 'Color', 'm',       'LineWidth', 1.5, 'DisplayName', 'Eff. SNR (EVM)');
legend(axSNR, 'Location', 'best');
% -----------------------------
ber_dl_trace = nan(1, nTotalSlots); ber_ul_trace = nan(1, nTotalSlots);
snr_theory_trace = nan(1, nTotalSlots); % Store theoretical SNR values
snr_eff_trace    = nan(1, nTotalSlots); % Store actual SNR values
mse_nlms_trace   = nan(1, nTotalSlots); % Store NLMS MSE trace
axConstDL = nexttile; hConstDL = plot(axConstDL, NaN, NaN, '.b'); 
grid(axConstDL,'on'); axis(axConstDL,'square'); xlim([-2 2]); ylim([-2 2]); title(axConstDL, 'DL RX (Clean)');
axConstUL = nexttile; hConstUL = plot(axConstUL, NaN, NaN, '.r'); 
grid(axConstUL,'on'); axis(axConstUL,'square'); xlim([-2 2]); ylim([-2 2]); title(axConstUL, 'UL RX (Jammed)');
axMSE = nexttile; 
hold(axMSE, 'on'); grid(axMSE, 'on');
title(axMSE, 'NLMS Convergence MSE');
ylabel(axMSE, 'MSE (dB)'); xlabel(axMSE, 'Slot Index');
xlim(axMSE, [0 nTotalSlots]);
hMSE = plot(axMSE, NaN, NaN, '-','Color','k','LineWidth', 1, 'DisplayName', 'NLMS MSE');
% -----------------------------

%% --- 5. SIMULATION LOOP ---
fprintf('\n=== Starting Simulation with Uplink Jammer ===\n');
simTime = 0;
k=1;
for slot_idx = 1:nTotalSlots
    slotInPattern = mod(slot_idx-1, SlotsPerPattern);
    isDownlink = (slotInPattern < Pattern_DL);
    simTime = simTime + slotDuration;
    
    for u = 1:N_UE
        UEs(u).pos = UEs(u).pos + UEs(u).vel * slotDuration;
        set(hUE_scatters(u), 'XData', UEs(u).pos(1), 'YData', UEs(u).pos(2), 'ZData', UEs(u).pos(3));
        
        UEs(u).channel_dl.InitialTime = simTime;
        UEs(u).channel_ul.InitialTime = simTime;
        
        if isDownlink
            %% --- DOWNLINK ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, N_ant_BS);
            all_bits = []; release(UEs(u).channel_dl);
            
            % Applying beamforming weights
            w_tx = conj(w_beam); w_tx = w_tx / norm(w_tx);
            %Building data symbols
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                grid_sym = zeros(K,1); 
                if ismember(sym, DMRS_positions)
                    grid_sym(DMRS_indices) = dmrs_seq;
                    bits_sym = randi([0 1], nData_DMRS_Sym*log2(M_DL), 1);
                    grid_sym(Data_indices) = qammod(bits_sym, M_DL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
                else
                    bits_sym = randi([0 1], nData_NoDMRS_Sym*log2(M_DL), 1);
                    grid_sym(Active_Indices) = qammod(bits_sym, M_DL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
                end
                all_bits = [all_bits; bits_sym];
                grid_precoded = grid_sym * w_tx.'; 
                data_grids(:, sym, :) = reshape(grid_precoded, K, 1, N_ant_BS);
            end
            
            tx_slot = build_slot(carrier, sync_grid, data_grids, FRAME_STRUCT.GUARD_SYMS, N_ant_BS); 
            current_tx_power = sum(mean(abs(tx_slot).^2, 1));
            % 2. Calculate required scaling factor to match Target Power (TxPower_dBm_DL)
            target_power_watts = 10^((TxPower_dBm_DL-30)/10);
            scaling_factor = sqrt(target_power_watts / ((current_tx_power + 1e-9)*N_ant_BS));
            % 3. Apply Scaling
            tx_slot = tx_slot * scaling_factor;
            
            % Applying pathloss and fast fading channel
            [rx_faded, ~] = UEs(u).channel_dl(tx_slot);
            pl_dB = nrPathLoss(plCfg, Pars.fc, 1, BS_Pos, UEs(u).pos);
            rx_total = rx_faded * 10^(-pl_dB/20) + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            offset_dl = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_dl:end, :);
            neededLen = sum(ofdmInfo.SymbolLengths(1:FRAME_STRUCT.TOTAL));
            if size(rx_sync,1) < neededLen
                rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, 1)]; 
            end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL
                continue; 
            end
            rx_data_grids = rx_grid_full(1:K, 2:13, 1);
            
            % DL Estimation: Channel estimation with polar interpolation
            H_dmrs_snapshots = zeros(K, length(DMRS_positions));
            for i = 1:length(DMRS_positions)
                sym_idx = DMRS_positions(i);
                rx_dmrs = rx_data_grids(DMRS_indices, sym_idx);
                H_raw = rx_dmrs ./ dmrs_seq;
                H_dmrs_snapshots(:, i) = interp1(double(DMRS_indices), double(H_raw), (1:K).', 'linear', 'extrap');
            end
            mag_snaps = abs(H_dmrs_snapshots).'; phase_snaps = unwrap(angle(H_dmrs_snapshots).');
            mag_interp = interp1(DMRS_positions, mag_snaps, (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            phase_interp = interp1(DMRS_positions, phase_snaps, (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            H_est_total = (mag_interp .* exp(1j * phase_interp)).'; 
            
            rx_bits_total = []; current_const = [];
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                %ZF equalization
                rx_eq_full = rx_data_grids(:, sym) ./ (H_est_total(:, sym) + 1e-8);
                if ismember(sym, DMRS_positions)
                    rx_useful = rx_eq_full(Data_indices); 
                else
                    rx_useful = rx_eq_full(Active_Indices); 
                end
                rx_bits_total = [rx_bits_total; qamdemod(rx_useful, M_DL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true)];
                current_const = [current_const; rx_useful];
            end
        else
            %% --- UPLINK (WITH JAMMER) ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, 1);
            all_bits = []; release(UEs(u).channel_ul);
            
            % Generate Legitimate UE Signal
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                grid_sym = zeros(K,1);
                if ismember(sym, DMRS_positions)
                    grid_sym(DMRS_indices) = dmrs_seq;
                    bits_sym = randi([0 1], nData_DMRS_Sym*log2(M_UL), 1);
                    grid_sym(Data_indices) = qammod(bits_sym, M_UL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
                else
                    bits_sym = randi([0 1], nData_NoDMRS_Sym*log2(M_UL), 1);
                    grid_sym(Active_Indices) = qammod(bits_sym, M_UL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
                end
                all_bits = [all_bits; bits_sym]; data_grids(:, sym, 1) = grid_sym;
            end
            
            tx_slot = build_slot(carrier, sync_grid, data_grids, FRAME_STRUCT.GUARD_SYMS, 1);
            tx_slot = tx_slot * Tx_AmpUL;
            
            % UE Channel + Pathloss
            [rx_faded, ~] = UEs(u).channel_ul(tx_slot);
            pl_dB_UE = nrPathLoss(plCfg, Pars.fc, 1, BS_Pos, UEs(u).pos);
            pl_dB = pl_dB_UE; % Save for SNR Plotting
            rx_ue = rx_faded * 10^(-pl_dB_UE/20);
            
            % Generation Jammer's signal
            % Generate random noise signal for duration of slot
            % We assume Jammer is omnidirectional simple noise source
            if mod(slot_idx, 200) >= 101
                jammer_signal = (randn(size(tx_slot)) + 1j*randn(size(tx_slot))) / sqrt(2);
            else
                jammer_signal = zeros(size(tx_slot));
            end
            jammer_signal = jammer_signal * Jammer_Amp;
            
            % Geometric attenuation for Jammer
            % Simple Friis/Free Space Path Loss for Jammer
            % Distance calculation
            dist_jammer = norm(BS_Pos - Jammer_Pos);
            lambda = physconst('LightSpeed') / Pars.fc;
            % FSPL Formula: (lambda / (4 * pi * d))^2
            % PathLoss (linear power gain) = (lambda / (4 * pi * dist))^2
            pl_lin_jammer = (lambda / (4 * pi * dist_jammer))^2;
            % Convert to amplitude attenuation
            atten_jammer = sqrt(pl_lin_jammer);
            
            % Jammer arrives at BS (Assumed LOS / No Fading for simplicity)
            rx_jammer = repmat(jammer_signal * atten_jammer, 1, N_ant_BS);
            
            rx_total = rx_ue + rx_jammer + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            % Synchronization & Demodulation
            offset_ul = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_ul:end, :);
            neededLen = sum(ofdmInfo.SymbolLengths(1:FRAME_STRUCT.TOTAL));
            if size(rx_sync,1) < neededLen, rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, size(rx_sync,2))]; end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL, continue; end
            rx_data_grids = rx_grid_full(1:K, 2:13, :);
            
            % Doppler compensation
            % Objective: Stabilize the pilot symbols so NLMS sees a "static" channel.
            % Without this, NLMS chases the Doppler rotation instead of the spatial beam.
            
            % Extract Raw DMRS (Pilots)
            rx_dmrs_raw = rx_data_grids(DMRS_indices, DMRS_positions, :); 
            
            % Calculate average power of received pilots
            rx_power_avg = mean(abs(rx_dmrs_raw(:)).^2);
            % Normalize input to unity magnitude. This helps NLMS converge instantly.
            agc_factor = 1 / sqrt(rx_power_avg);
            rx_dmrs_raw = rx_dmrs_raw * agc_factor;
            
            % Estimate Raw Channel per Antenna Element
            H_element_raw = rx_dmrs_raw ./ dmrs_seq; 
            
            % Calculate Phase Drift relative to First Symbol
            % Use the first DMRS symbol as the time reference (t=0)
            H_ref = H_element_raw(:, 1, :); 
            
            rx_dmrs_stabilized = rx_dmrs_raw; % Copy for output
            
            for i = 2:length(DMRS_positions)
                H_curr = H_element_raw(:, i, :);
                
                % Calculate raw correlation
                raw_corr = H_curr .* conj(H_ref);
                
                % This calculates the "Common Phase Error" (CPE) of the array.
                avg_corr_global = mean(mean(raw_corr, 1), 3); 
                
                % Calculate the global rotation angle
                phase_drift = angle(avg_corr_global); 
                
                % Create scalar counter-rotation factor
                correction_factor = exp(-1j * phase_drift);
                
                % Apply de-rotation to ALL antennas identically
                rx_dmrs_stabilized(:, i, :) = rx_dmrs_raw(:, i, :) * correction_factor;
            end
            
            % --- NLMS ALGORITHM ---
            
            avg_sq_error = 0; count_samples = 0;
            
            % Iterate through stabilized DMRS to update beam weights
            for i = 1:length(DMRS_positions)
                rx_vecs = squeeze(rx_dmrs_stabilized(:, i, :)); 
                
                for re = 1:size(rx_vecs, 1)
                    u_in = rx_vecs(re, :).';    % Input vector 
                    d_des = dmrs_seq(re);       % Desired signal (known pilot)
                    y_out = w_beam' * u_in;     % Beamformer output
                    e = d_des - y_out;          % Error
                    
                    % NLMS Weight Update
                    w_beam = w_beam + (mu_current / (u_in' * u_in + 1e-6)) * u_in * conj(e);
                    
                    avg_sq_error = avg_sq_error + abs(e)^2;
                    count_samples = count_samples + 1;
                end
            end
            w_norm = norm(w_beam);
            if w_norm > 1e-6
                w_beam = w_beam / w_norm;
            else
                % Reset if weights collapsed to zero
                disp("Error in weights calculation. Using dumb beamforming");
                w_beam = ones(N_ant_BS, 1) ./ sqrt(N_ant_BS);
            end
            % Update Adaptive Mu
            if count_samples > 0
                mse_current = avg_sq_error / count_samples;
                mse_nlms_trace(slot_idx) = 10*log10(mse_current); 
                
                % Limit how fast Mu can grow
                target_mu = min(1.0, 3 * mse_current); 
                target_mu = max(mu_min, min(mu_max, target_mu));
                mu_current = alpha_mu * mu_current + (1 - alpha_mu) * target_mu;
            end
            
            
           % --- UL DATA DETECTION (MRC) ---
            rx_bits_total = []; current_const = [];
            H_dmrs_raw = zeros(length(DMRS_indices), length(DMRS_positions), N_ant_BS);
            
            for ant = 1:N_ant_BS
                for i = 1:length(DMRS_positions)
                    sym = DMRS_positions(i);
                    rx_val = squeeze(rx_data_grids(DMRS_indices, sym, ant));
                    H_inst = rx_val ./ dmrs_seq;
                    H_dmrs_raw(:, i, ant) = smoothdata(H_inst, 1, 'movmean', 12); 
                end
            end
            
            H_est_full = zeros(K, FRAME_STRUCT.DATA_SYMS, N_ant_BS);
            for ant = 1:N_ant_BS
                mag_snaps   = abs(H_dmrs_raw(:, :, ant)).'; 
                phase_snaps = unwrap(angle(H_dmrs_raw(:, :, ant)).'); 
                mag_i   = interp1(DMRS_positions, mag_snaps,   (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
                phase_i = interp1(DMRS_positions, phase_snaps, (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
                H_time_temp = (mag_i .* exp(1j * phase_i)).'; 
                for sym = 1:FRAME_STRUCT.DATA_SYMS
                    H_est_full(:, sym, ant) = interp1(double(DMRS_indices), H_time_temp(:, sym), (1:K).', 'linear', 'extrap');
                end
            end
            
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                rx_slice = squeeze(rx_data_grids(:, sym, :)); 
                H_slice  = squeeze(H_est_full(:, sym, :)); 
                rx_combined = sum(rx_slice .* conj(H_slice), 2); 
                H_power = sum(abs(H_slice).^2, 2) + 1e-8;
                rx_eq = rx_combined ./ H_power;
                avg_pwr = mean(abs(rx_eq).^2);
                if avg_pwr > 0
                    rx_eq = rx_eq / sqrt(avg_pwr);
                end
                if ismember(sym, DMRS_positions), rx_useful = rx_eq(Data_indices);
                else, rx_useful = rx_eq(Active_Indices); end
                rx_bits_total = [rx_bits_total; qamdemod(rx_useful, M_UL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true)];
                current_const = [current_const; rx_useful];
            end
        end
        
        % --- CALCULATE THEORETICAL SNR ---
        % Formula: P_rx(dBm) = P_tx(dBm) + ArrayGain(dB) - PathLoss(dB)
        % SNR(dB) = P_rx(dBm) - NoiseFloor(dBm)
        
        if isDownlink
            % DL: Power at the single UE antenna
            sig_power_watts = mean(abs(rx_faded).^2, 'all');
        else
            % UL: Total power collected by the array (approximates MRC gain)
            sig_power_watts = sum(mean(abs(rx_faded).^2, 1)); 
        end
        
        % 2. Convert to dBm
        sig_power_dBm = 10*log10(sig_power_watts) + 30;
        
        % 3. Apply Path Loss
        % The actual received power = FadedPower - PathLoss
        rx_power_actual_dBm = sig_power_dBm - pl_dB;
       
        % 4. SNR = Signal - Noise
        Theoretical_SNR = rx_power_actual_dBm - NoiseFloor_dBm;
        if isDownlink
            if ~isempty(current_const) && ~isempty(all_bits)
                current_M = M_DL;
                ideal_const = qammod(all_bits, current_M, 'gray', ...
                                     'InputType', 'bit', 'UnitAveragePower', true);
                
                % 2. Calculate Error Vector Magnitude (EVM)
                %    EVM = Average distance between Received and Ideal points
                error_vector = current_const - ideal_const;
                evm_power    = mean(abs(error_vector).^2);
                
                % 3. Convert to SNR
                %    SNR ≈ 1 / EVM^2
                Effective_SNR = -10 * log10(evm_power + 1e-12);
                
                snr_eff_trace(k) = Effective_SNR;
            end
            snr_theory_trace(k) = Theoretical_SNR;
            k = k+1;
        end
        
        nBit = min(length(all_bits), length(rx_bits_total));
        ber = sum(all_bits(1:nBit) ~= rx_bits_total(1:nBit)) / nBit;
        if isDownlink, ber_dl_trace(slot_idx) = ber; else, ber_ul_trace(slot_idx) = ber; end
        distance = sqrt(sum((UEs(u).pos - BS_Pos).^2));
        if mod(slot_idx, 8) == 0
            fprintf('Slot %d | %s | BER: %.5f | Mu: %.3f | Distance: %.2f | SNR: %.2f dB\n', ...
                slot_idx, iif(isDownlink,'DL','UL'), ber, mu_current, ...
                distance, Theoretical_SNR);        
        end
        
        if ~isempty(current_const)
            if isDownlink, set(hConstDL, 'XData', real(current_const), 'YData', imag(current_const));
            else, set(hConstUL, 'XData', real(current_const), 'YData', imag(current_const)); end
        end
        
        if mod(slot_idx, 5) == 0
            set(hBER_DL, 'XData', 1:slot_idx, 'YData', ber_dl_trace(1:slot_idx));
            set(hBER_UL, 'XData', 1:slot_idx, 'YData', ber_ul_trace(1:slot_idx));
            current_mse_data = mse_nlms_trace(1:slot_idx);
            valid_idx = find(~isnan(current_mse_data));
            set(hMSE, 'XData', valid_idx, 'YData', current_mse_data(valid_idx));
            
            drawnow limitrate;
        end
        if isDownlink
            set(hSNR_Theory, 'XData', 1:k, 'YData',snr_theory_trace(1:k));
            set(hSNR_Eff,    'XData', 1:k, 'YData', snr_eff_trace(1:k));
            drawnow limitrate; 
        end
    end
end
%% HELPER FUNCITONS
function tx_slot = build_slot(carrier, sync_grid, data_grids, guard_count, N_ant)
    sync_full = repmat(sync_grid, [1, 1, N_ant]);
    guard_grid = zeros(size(sync_grid,1), guard_count, N_ant);
    full_grid = cat(2, sync_full, data_grids, guard_grid);
    tx_slot = nrOFDMModulate(carrier, full_grid);
end
function result = iif(condition, true_val, false_val)
    if condition, result = true_val; else, result = false_val; end
end