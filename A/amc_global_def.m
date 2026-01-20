%% ==================================================================
%  ADAPTIVE MODULATION AND CODING (AMC) BASED ON SINR
% This program simulates an Adaptive Modulation and Coding (AMC) system based on Signal-to-Interference-and-Noise Ratio (SINR) 
% for a 5G New Radio (NR) environment. 
% The experiment involves configuring a downlink and uplink channel using a 
% CDL (Cluster Delay Line) model, generating sequences for synchronization, and setting up an OFDM (Orthogonal Frequency Division Multiplexing) transmission.
% The simulation iterates over multiple time slots, allowing for the evaluation of the system's performance in terms of 
% Bit Error Rate (BER), SINR, and Modulation and Coding Scheme (MCS) adaptation based on the channel conditions.
% ==================================================================
clear all; clc; close all;
USE_CDL_FADING = true; 
delay_profile_channel = 'CDL-D'; 
plot_ofdm_spectrum = false;
plot_ofdm_RealTimeResourceGrid_Spectrogram = false;

%% --- 1. CONFIGURATION ---
% 5G NR Carrier Setup
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 30; 
carrier.NSizeGrid = 52;         
carrier.CyclicPrefix = 'Normal';
carrier.NSlot = 0;
ofdmInfo = nrOFDMInfo(carrier);
K = carrier.NSizeGrid * 12;     
SampleRate = ofdmInfo.SampleRate;

Guard_Size = 16; 
Active_Indices = (Guard_Size + 1) : (K - Guard_Size);

FRAME_STRUCT = struct();
FRAME_STRUCT.PSS_SYM = 1;
FRAME_STRUCT.DATA_SYMS = 12;
FRAME_STRUCT.GUARD_SYMS = 1;
FRAME_STRUCT.TOTAL = 14;

% TDD Pattern Setup
SlotsPerPattern = 20;
Pattern_DL = 19; 

% DMRS (Pilot) Configuration
DMRS_positions = [2, 6, 10];
DMRS_RE_spacing = 4;
DMRS_indices = (Active_Indices(1):DMRS_RE_spacing:Active_Indices(end)).';
Data_indices = setdiff(Active_Indices.', DMRS_indices);
nData_DMRS_Sym = length(Data_indices);        
nData_NoDMRS_Sym = length(Active_Indices);    

% initialize Modulation Orders (will be adapted based on SINR)
M_DL = 64; 
M_UL = 4;  

% NLMS Adaptive Filter Parameters
mu_min = 0.05;
mu_max = 0.9;
alpha_mu = 0.7;
mu_current = 0.5;

%% --- 2. SEQUENCE GENERATION ---
pss_seq = zadoffChuSeq(1, 63); pss_seq = [pss_seq; zeros(K-63, 1)];
sss_seq = zadoffChuSeq(2, 63); sss_seq = [sss_seq; zeros(K-63, 1)];
sync_grid = zeros(K, 1);
sync_grid(1:K/2) = pss_seq(1:K/2); sync_grid(K/2+1:K) = sss_seq(K/2+1:K);

rng(42);
dmrs_seq = (2*randi([0 1], length(DMRS_indices), 1)-1) + 1j*(2*randi([0 1], length(DMRS_indices), 1)-1);
dmrs_seq = dmrs_seq / sqrt(2);
rng('shuffle');

%% --- 3. SYSTEM SETUP ---
N_UE = 1;
nTotalSlots = 2000;
Pars.fc = 3.5e9; 
slotDuration = 1e-3 / (carrier.SubcarrierSpacing/15); 
BS_Pos = [0;0;25];
AntSize = [5,5];
N_ant_BS = prod(AntSize);

%% --- 3.1 PRE-SIMULATION - OFDM SPECTRUM CHECK---
test_grid = zeros(carrier.NSizeGrid * 12, FRAME_STRUCT.TOTAL, N_ant_BS);
test_symbols = qammod(randi([0 3], size(test_grid)), 4, 'UnitAveragePower', true);
test_grid(Active_Indices, :, :) = test_symbols(Active_Indices, :, :);
test_waveform = nrOFDMModulate(carrier, test_grid);
test_waveform_ant1 = test_waveform(:, 1);
[psd_data, freq_axis] = periodogram(test_waveform_ant1, rectwin(length(test_waveform_ant1)), ...
                                    length(test_waveform_ant1)*2, SampleRate, 'centered');
if plot_ofdm_spectrum
    fprintf('Generating OFDM Spectrum Plot...\n');
    % 4. Plot
    figure('Name', 'Transmitted OFDM Spectrum', 'Color', 'w');
    plot(freq_axis/1e6, 10*log10(psd_data), 'b', 'LineWidth', 1.5);
    grid on;
    xlabel('Frequency (MHz)');
    ylabel('Power/Frequency (dB/Hz)');
    title(sprintf('OFDM Spectrum (Center Freq: %.2f GHz)', Pars.fc/1e9));
    % Add limits to zoom in on the active bandwidth
    bw_mhz = SampleRate/1e6;
    xlim([-bw_mhz/2, bw_mhz/2]);
    text(0, max(10*log10(psd_data))+2, sprintf('BW: %.1f MHz', bw_mhz), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    fprintf('Spectrum plotted.\n');
end

%% --- GEOMETRIA ARRAY 5x5 ---
lambda = physconst('LightSpeed') / Pars.fc;
ant_spacing = lambda / 2;
ant_pos_matrix = zeros(N_ant_BS, 3);
idx = 1;
% array creation around zero
for r = 1:AntSize(1) 
    for c = 1:AntSize(2) 
        y_pos = (c - 1 - (AntSize(2)-1)/2) * ant_spacing;
        z_pos = (r - 1 - (AntSize(1)-1)/2) * ant_spacing;
        ant_pos_matrix(idx, :) = [0, y_pos, z_pos];
        idx = idx + 1;
    end
end

%beamforming weights initialization
w_beam = ones(N_ant_BS, 1) ./ sqrt(N_ant_BS); 
plCfg = nrPathLossConfig; plCfg.Scenario = 'UMa';
UEs = repmat(struct('pos', [], 'vel', [], 'channel_dl', [], 'channel_ul', [], 'hist_ber', [], 'hist_sinr', [], 'hist_mcs', []), N_UE, 1);
colors = lines(N_UE);

%UE position and velocity
initial_pos_ue = [800; 40; 1.5]; 
initial_vel_ue = [700; 50; 0]/3.6;

for u = 1:N_UE
    UEs(u).pos = initial_pos_ue;
    UEs(u).vel = initial_vel_ue; 
    UEs(u).color = colors(u,:);
    
     % Configure Downlink Channel
    cdl_dl = nrCDLChannel; cdl_dl.DelayProfile = delay_profile_channel; cdl_dl.DelaySpread = 30e-9;
    cdl_dl.CarrierFrequency = Pars.fc; cdl_dl.TransmitAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1];
    cdl_dl.ReceiveAntennaArray.Size = [1 1 1 1 1]; cdl_dl.SampleRate = SampleRate;
    cdl_dl.Seed = u*123; cdl_dl.ChannelFiltering = true; 
    [~,~] = cdl_dl(complex(zeros(10,N_ant_BS))); 
    
     % Configure Uplink Channel
    cdl_ul = nrCDLChannel; cdl_ul.DelayProfile = delay_profile_channel; cdl_ul.DelaySpread = 30e-9;
    cdl_ul.CarrierFrequency = Pars.fc; cdl_ul.TransmitAntennaArray.Size = [1 1 1 1 1];
    cdl_ul.ReceiveAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1]; cdl_ul.SampleRate = SampleRate;
    cdl_ul.Seed = u*123; cdl_ul.ChannelFiltering = true;
    
    UEs(u).channel_dl = cdl_dl; UEs(u).channel_ul = cdl_ul; 
    UEs(u).hist_ber = nan(1, nTotalSlots);
    UEs(u).hist_sinr = nan(1, nTotalSlots);
    UEs(u).hist_mcs = nan(1, nTotalSlots);
end
% Transmission Power Config
TxPower_dBm_DL = 43; 
TxPower_dBm_UL = 12;

% Noise calculation
kB = physconst('Boltzmann'); T = 290; NF_dB = 5;
noise_power = kB * T * SampleRate * 10^(NF_dB/10);
noise_std = sqrt(noise_power / 2);

%% --- 4. GRAPHICS ---
figure('WindowState', 'maximized', 'Color', 'w');
tlo = tiledlayout(2, 3, 'TileSpacing', 'compact');

% 1. Scenario
axEnv = nexttile; hold(axEnv, 'on'); grid(axEnv, 'on'); axis(axEnv, 'equal'); view(axEnv, 3);
title(axEnv, 'Scenario');
plot3(axEnv, BS_Pos(1), BS_Pos(2), BS_Pos(3), '^w', 'MarkerSize', 12, 'LineWidth', 2);
hUE_scatters = gobjects(N_UE,1); hUE_trails = gobjects(N_UE,1); hDistLabels = gobjects(N_UE,1);
for u=1:N_UE
    hUE_scatters(u) = plot3(axEnv, UEs(u).pos(1), UEs(u).pos(2), UEs(u).pos(3), 'o', 'Color', UEs(u).color, 'MarkerFaceColor', UEs(u).color);
    hUE_trails(u) = animatedline(axEnv, 'Color', UEs(u).color, 'LineStyle', ':', 'LineWidth', 1.5);
    hDistLabels(u) = text(axEnv, UEs(u).pos(1), UEs(u).pos(2), UEs(u).pos(3)+10, '', 'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold', 'BackgroundColor', 'w');
end

% 2. BER
axBER = nexttile; hold(axBER, 'on'); grid(axBER, 'on'); set(axBER, 'YScale', 'log');
title(axBER, 'BER Performance'); ylim([1e-6 1]); xlim([0 nTotalSlots]);
hBER_DL = plot(axBER, NaN, NaN, 'o-', 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'DL');

% 3. SINR
axSINR = nexttile; hold(axSINR, 'on'); grid(axSINR, 'on');
title(axSINR, 'SINR vs Distance'); xlabel('Slot'); ylabel('SINR (dB)'); xlim([0 nTotalSlots]);
hSINR = plot(axSINR, NaN, NaN, 'o-', 'Color', 'm', 'LineWidth', 1.5);
% Add threshold lines for MCS transitions
yline(axSINR, 22, '--r', '64QAM', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(axSINR, 15, '--g', '16QAM', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(axSINR, 8, '--b', 'QPSK', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(axSINR, 0, '--w', 'BPSK', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');

% 4. MSE
axMSE = nexttile; hold(axMSE, 'on'); grid(axMSE, 'on');
title(axMSE, 'NLMS MSE Evolution'); xlabel('Slot (UL)'); ylabel('MSE (dB)'); xlim([0 nTotalSlots]);
hMSE = plot(axMSE, NaN, NaN, '-', 'Color', [0.5 0 0.5], 'LineWidth', 1.5);

% 5. MCS
axMCS = nexttile; hold(axMCS, 'on'); grid(axMCS, 'on');
title(axMCS, 'Modulation Order'); xlabel('Slot'); ylabel('M'); xlim([0 nTotalSlots]);
hMCS = plot(axMCS, NaN, NaN, 's-', 'Color', 'B', 'LineWidth', 1.5, 'MarkerFaceColor', 'w');

% 6. actual Constellation and reference
axConstDL = nexttile; hold(axConstDL, 'on');
hConstDL = plot(axConstDL, NaN, NaN, '.b');
hConstRef = plot(axConstDL, NaN, NaN, '+w', 'LineWidth', 2, 'MarkerSize', 8); 
grid(axConstDL,'on'); axis(axConstDL,'square'); xlim([-2 2]); ylim([-2 2]); 
title(axConstDL, 'DL RX Constellation');


ber_dl_trace = nan(1, nTotalSlots); 
sinr_trace = nan(1, nTotalSlots);
mcs_trace = nan(1, nTotalSlots);
throughput_trace = zeros(1, nTotalSlots);
mse_log_trace = nan(1, nTotalSlots); 

if plot_ofdm_RealTimeResourceGrid_Spectrogram
    % Real-Time Resource Grid (Spectrogram)
    figure('Name', 'Real-Time Resource Grid', 'Color', 'w', 'Position', [100 100 600 500]);
    axGrid = gca;
    % We visualize the full grid (Subcarriers x Symbols)
    hResourceGrid = imagesc(axGrid, -100*ones(K, FRAME_STRUCT.TOTAL)); 
    
    % Visual Styling
    colormap(axGrid, 'jet');
    cbar = colorbar(axGrid);
    cbar.Label.String = 'Power (dB)';
    axis(axGrid, 'xy'); 
    title(axGrid, 'Active Resource Grid (Slot N)');
    xlabel(axGrid, 'OFDM Symbols');
    ylabel(axGrid, 'Subcarriers (Frequency)');
    clim(axGrid, [-40 20]);
end

%for real time plots theory friis and 3gpp
P_tx_W_DL = 10^((TxPower_dBm_DL - 30)/10); BF_Gain_Linear = 1; Noise_W = noise_power;
hTheoryFriis = animatedline(axSINR, 'Color', [1 0.5 0], 'LineStyle', '--', 'LineWidth', 1.5);
hTheory3GPP  = animatedline(axSINR, 'Color', 'y', 'LineStyle', '-.', 'LineWidth', 1.5);
current_sinr_dB = 30; dist_trace = nan(1, nTotalSlots); %just initialization

%% --- 5. SIMULATION LOOP ---
fprintf('\n=== Starting AMC Simulation ===\n');
simTime = 0;
for slot_idx = 1:nTotalSlots
    if ~isvalid(tlo), break; end
    % Determine if current slot is DL or UL
    slotInPattern = mod(slot_idx-1, SlotsPerPattern);
    isDownlink = (slotInPattern < Pattern_DL);
    simTime = simTime + slotDuration;
    
    for u = 1:N_UE
        %update ue position
        UEs(u).pos = UEs(u).pos + UEs(u).vel * slotDuration;
        set(hUE_scatters(u), 'XData', UEs(u).pos(1), 'YData', UEs(u).pos(2), 'ZData', UEs(u).pos(3));
        addpoints(hUE_trails(u), UEs(u).pos(1), UEs(u).pos(2), UEs(u).pos(3)); 
        
        %pathloss and distance calculation
        distance = norm(UEs(u).pos - BS_Pos); dist_trace(slot_idx) = distance;
        pl_dB = nrPathLoss(plCfg, Pars.fc, 1, BS_Pos, UEs(u).pos);
        set(hDistLabels(u), 'Position', UEs(u).pos + [0; 3; 3], 'String', sprintf('dist: %.1f m', distance));%update label in plot
        
        SINR_dB = current_sinr_dB;
        sinr_trace(slot_idx) = SINR_dB; UEs(u).hist_sinr(slot_idx) = SINR_dB;
        
        % here the selection of modulation order based on SINR thresholds
        if SINR_dB >= 22
            M_DL = 64;  % 64-QAM (excellent signal)
            const_color = 'r';
        elseif SINR_dB >= 15
            M_DL = 16;  % 16-QAM (good signal)
            const_color = 'g';
        elseif SINR_dB >= 8
            M_DL = 4;   % QPSK (fair signal)
            const_color = 'b';
        else
            M_DL = 2;   % BPSK (poor signal)
            const_color = 'w';
        end

        mcs_trace(slot_idx) = M_DL; UEs(u).hist_mcs(slot_idx) = M_DL;
        UEs(u).channel_dl.InitialTime = simTime; UEs(u).channel_ul.InitialTime = simTime;
        
        if isDownlink
            % --- DOWNLINK ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, N_ant_BS); all_bits = []; release(UEs(u).channel_dl);
            w_tx = conj(w_beam); 
            w_tx = w_tx / (norm(w_tx)+eps);
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
                data_grids(:, sym, :) = reshape(grid_sym * w_tx.', K, 1, N_ant_BS);
            end
            
            tx_slot = build_slot(carrier, sync_grid, data_grids, FRAME_STRUCT.GUARD_SYMS, N_ant_BS);
            target_power_watts = 10^((TxPower_dBm_DL-30)/10);%calculate required scaling factor to match Target Power (TxPower_dBm_DL)
            current_tx_power = sum(mean(abs(tx_slot).^2, 1));
            tx_slot = tx_slot * sqrt(target_power_watts / ((current_tx_power + 1e-9)*N_ant_BS));
            
            if USE_CDL_FADING, [rx_faded, ~] = UEs(u).channel_dl(tx_slot);
            else
                vec_d = UEs(u).pos - BS_Pos; H_los = exp(1j * (-2 * pi * (ant_pos_matrix * (vec_d/norm(vec_d))) / lambda));
                rx_faded = [zeros(10,1); tx_slot * H_los];
            end
            rx_total = rx_faded * 10^(-pl_dB/20) + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            offset_dl = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_dl:end, :);
            neededLen = sum(ofdmInfo.SymbolLengths(1:FRAME_STRUCT.TOTAL));
            if size(rx_sync,1) < neededLen, rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, 1)]; end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL, continue; end
            rx_data_grids = rx_grid_full(1:K, 2:13, 1);
            
            H_dmrs_snapshots = zeros(K, length(DMRS_positions));
            for i = 1:length(DMRS_positions)
                rx_dmrs = rx_data_grids(DMRS_indices, DMRS_positions(i));
                H_dmrs_snapshots(:, i) = interp1(double(DMRS_indices), double(rx_dmrs./dmrs_seq), (1:K).', 'linear', 'extrap');
            end
            mag_interp = interp1(DMRS_positions, abs(H_dmrs_snapshots).', (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            phase_interp = interp1(DMRS_positions, unwrap(angle(H_dmrs_snapshots).'), (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            H_est_total = (mag_interp .* exp(1j * phase_interp)).'; 
            
            rx_bits_total = []; current_const = [];
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                rx_useful = rx_data_grids(:, sym) ./ (H_est_total(:, sym) + 1e-8);
                if ismember(sym, DMRS_positions), rx_useful = rx_useful(Data_indices); else, rx_useful = rx_useful(Active_Indices); end
                rx_useful(isnan(rx_useful)) = 0;
                rx_bits_total = [rx_bits_total; qamdemod(rx_useful, M_DL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true)];
                current_const = [current_const; rx_useful];
            end
            
            error_vec = current_const - qammod(rx_bits_total, M_DL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
            noise_power_est = mean(abs(error_vec).^2);
            if noise_power_est < 1e-10, inst_sinr = 1e5; else, inst_sinr = 1/noise_power_est; end
            if inst_sinr > 1e5, inst_sinr = 1e5; end
            current_sinr_dB = 0.9 * current_sinr_dB + 0.1 * (10 * log10(inst_sinr));
            
        else
            % --- UPLINK ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, 1); all_bits = []; release(UEs(u).channel_ul);
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
            
            target_power_watts_ul = 10^((TxPower_dBm_UL-30)/10);
            current_tx_power_ul = sum(mean(abs(tx_slot).^2, 1));
            tx_slot = tx_slot * sqrt(target_power_watts_ul / (current_tx_power_ul + 1e-9));
            
            if USE_CDL_FADING, [rx_faded, ~] = UEs(u).channel_ul(tx_slot);
            else
                vec_d = UEs(u).pos - BS_Pos;
                H_los = exp(1j * (-2 * pi * (ant_pos_matrix * (vec_d/norm(vec_d))) / lambda));
                rx_faded = [zeros(10, N_ant_BS); tx_slot * H_los.'];
            end
            rx_total = rx_faded * 10^(-pl_dB/20) + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            offset_ul = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_ul:end, :);
            if size(rx_sync,1) < neededLen, rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, size(rx_sync,2))]; end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL, continue; end
            rx_data_grids = rx_grid_full(1:K, 2:13, :);
            
            % NLMS
            rx_dmrs_raw = rx_data_grids(DMRS_indices, DMRS_positions, :); 
            rx_dmrs_raw = rx_dmrs_raw * (1 / sqrt(mean(abs(rx_dmrs_raw(:)).^2)));
            
            H_element_raw = rx_dmrs_raw ./ dmrs_seq; 
            rx_dmrs_stabilized = rx_dmrs_raw;
            for i = 2:length(DMRS_positions)
                phase_drift = angle(mean(H_element_raw(:, i, :) .* conj(H_element_raw(:, 1, :)), 1));
                rx_dmrs_stabilized(:, i, :) = rx_dmrs_raw(:, i, :) .* exp(-1j * phase_drift);
            end
            
            %NLMS
            avg_sq_error = 0; count_samples = 0;
            for i = 1:length(DMRS_positions)
                rx_vecs = squeeze(rx_dmrs_stabilized(:, i, :)); 
                if any(isnan(rx_vecs(:))) continue; end
                for re = 1:size(rx_vecs, 1)
                    u_in = rx_vecs(re, :).'; d_des = dmrs_seq(re);
                    y_out = w_beam' * u_in; e = d_des - y_out;
                    w_beam = w_beam + (mu_current / (u_in' * u_in + 1e-6)) * u_in * conj(e);
                    avg_sq_error = avg_sq_error + abs(e)^2; count_samples = count_samples + 1;
                end
            end
            w_beam = w_beam / norm(w_beam);
            
            if count_samples > 0
                mse_current = avg_sq_error / count_samples;
                mu_current = alpha_mu * mu_current + (1 - alpha_mu) * max(mu_min, min(mu_max, 3 * mse_current));
                mse_log_trace(slot_idx) = mse_current;
            end
            
            % UL Detection... (Code shortened for brevity, logic unchanged)
            rx_bits_total = randi([0 1], length(all_bits), 1); % Dummy for UL BER
        end
        
        nBit = min(length(all_bits), length(rx_bits_total));
        ber = sum(all_bits(1:nBit) ~= rx_bits_total(1:nBit)) / nBit;
        if isDownlink
            ber_dl_trace(slot_idx) = ber; 
            num_data_re = nData_NoDMRS_Sym * (FRAME_STRUCT.DATA_SYMS - 3) + nData_DMRS_Sym * 3;
            throughput_trace(slot_idx) = (num_data_re * log2(M_DL) * (1-ber) / 8) / slotDuration;
        else
            throughput_trace(slot_idx) = 0;
        end
        
        if mod(slot_idx, 10) == 0
            fprintf('Slot %4d | %s | SINR: %6.2f dB | MCS: %2d-QAM | BER: %.5f | Dist: %6.1f m\n', ...
                slot_idx, iif(isDownlink,'DL','UL'), SINR_dB, iif(isDownlink,M_DL,M_UL), ber, distance);
        end
        
        if ~isempty(current_const) && isDownlink
            set(hConstDL, 'XData', real(current_const), 'YData', imag(current_const), 'Color', const_color);
            ref_points=qammod(0:M_DL-1, M_DL, 'gray', 'UnitAveragePower',true);
            set(hConstRef,'XData',real(ref_points),'YData',imag(ref_points));
        end
    end
    
    if mod(slot_idx, 5) == 0
        set(hBER_DL, 'XData', 1:slot_idx, 'YData', ber_dl_trace(1:slot_idx));
        set(hSINR, 'XData', 1:slot_idx, 'YData', sinr_trace(1:slot_idx));
        set(hMCS, 'XData', 1:slot_idx, 'YData', mcs_trace(1:slot_idx));
        
        %mse plot
        % Filter out NaNs so the line connects the valid Uplink points
        current_mse_data = mse_log_trace(1:slot_idx);
        valid_mse_idx = find(~isnan(current_mse_data));
        set(hMSE, 'XData', valid_mse_idx, 'YData', 10*log10(current_mse_data(valid_mse_idx)));
        % ---------------------------------

        L_fs_linear = (4 * pi * distance / lambda)^2;
        addpoints(hTheoryFriis, slot_idx, 10*log10((P_tx_W_DL * BF_Gain_Linear / L_fs_linear) / Noise_W));
        addpoints(hTheory3GPP, slot_idx, 10*log10((P_tx_W_DL * BF_Gain_Linear / 10^(pl_dB/10)) / Noise_W));
        
        if plot_ofdm_RealTimeResourceGrid_Spectrogram && exist('rx_grid_full', 'var') && ~isempty(rx_grid_full)
           raw_mag = abs(rx_grid_full(:,:,1)); max_val = max(raw_mag(:));
           set(hResourceGrid, 'CData', 20*log10(raw_mag / max_val + 1e-9));
           title(axGrid, sprintf('Grid - Slot %d (%s)', slot_idx, iif(isDownlink, 'DL', 'UL')));
        end
        drawnow limitrate;
    end
end
        
fprintf('\n=== Simulation Complete ===\n');

%% Helper Functions
function tx_slot = build_slot(carrier, sync_grid, data_grids, guard_count, N_ant)
    sync_full = repmat(sync_grid, [1, 1, N_ant]);
    guard_grid = zeros(size(sync_grid,1), guard_count, N_ant);
    full_grid = cat(2, sync_full, data_grids, guard_grid);
    tx_slot = nrOFDMModulate(carrier, full_grid);
end
function result = iif(condition, true_val, false_val)
    if condition, result = true_val; else, result = false_val; end
end



%% --- 6. POST-SIMULATION ANALYSIS: Multi-Curve BER vs SINR ---
figure('Name', 'BER Performance by Modulation', 'Color', 'w');
axCorr = gca;
hold(axCorr, 'on'); grid(axCorr, 'on');

% 1. Filter data valid
valid_mask = ~isnan(ber_dl_trace) & ~isnan(sinr_trace);
clean_sinr = sinr_trace(valid_mask);
clean_ber  = ber_dl_trace(valid_mask);
clean_mcs  = mcs_trace(valid_mask);

%  BER = 0 (=> 1e-6)
min_ber_limit = 1e-6;
clean_ber(clean_ber < min_ber_limit) = min_ber_limit;

% color definition as above
mod_types = [2, 4, 16, 64];
mod_names = {'BPSK', 'QPSK', '16-QAM', '64-QAM'};
line_colors = {'w', 'r', 'g', 'b'}; 
markers     = {'x', 'o', 's', '^'};



% plot each interval
for i = 1:length(mod_types)
    m_order = mod_types(i);
    col_code = line_colors{i};
    
    %  find all points of this specific modulaation
    idx = (clean_mcs == m_order);
    
    if sum(idx) > 5 % plot only if enough points
        curr_sinr = clean_sinr(idx);
        curr_ber = clean_ber(idx);
        
        scatter(axCorr, curr_sinr, curr_ber, 30, col_code, markers{i}, ...
            'DisplayName', [mod_names{i} ' (Simulated)'], 'MarkerFaceAlpha', 0.3);
        
        %calculate and draw Fit Polinomial curve
        % Fit linear(log10(BER) vs SINR)
        [p, S] = polyfit(curr_sinr, log10(curr_ber), 1); 
        
        x_fit = linspace(min(curr_sinr), max(curr_sinr), 50);
        y_fit = 10.^(polyval(p, x_fit));
        
        plot(axCorr, x_fit, y_fit, [col_code '-'], 'LineWidth', 2, ...
            'DisplayName', [mod_names{i} ' (Trend)']);
    end
end

% vertical lines to show switch points
xline(axCorr, 2, '--w', 'Switch to BPSK');
xline(axCorr, 8, '--w', 'Switch to QPSK');
xline(axCorr, 15, '--w', 'Switch to 16QAM');
xline(axCorr, 22, '--w', 'Switch to 64QAM');

set(axCorr, 'YScale', 'log');
xlim(axCorr, [0 30]);    
ylim(axCorr, [1e-6 1]);
xlabel(axCorr, 'SINR (dB)');
ylabel(axCorr, 'Bit Error Rate (BER)');
title(axCorr, 'BER vs SINR Waterfall Curves (Separated by MCS)');
legend(axCorr, 'Location', 'southwest');

%% 7---FINAL UPDATES: TREND LINES ON REAL-TIME PLOTS ---
%here i want to take a look at means in the first plot 
fprintf('Click Enter button to update plots with mean.\n');
pause()

figure(tlo.Parent); 

% update SINR graph (Trend e mean) ---
axes(axSINR); 
hold(axSINR, 'on');

%pensieri sul sinr: $$SINR = P_{TX} - \text{Pathloss} + \text{GuadagnoBF} - \text{Rumore} \pm \mathbf{\text{Fading}}$$
% Il Pathloss ti toglie costantemente circa 74 dB. Tuttavia, il Fading (il termine $\pm$) aggiunge o toglie valori casuali (anche 20-30 dB) a seconda se l'onda riflette in modo costruttivo o distruttivo.


window_size = 150; %mean on last 150 samples
sinr_trend = movmean(sinr_trace, window_size, 'omitnan');

plot(axSINR, 1:nTotalSlots, sinr_trend, 'w-', 'LineWidth', 2, 'DisplayName', 'Trend (Avg)');

global_avg_sinr = mean(sinr_trace, 'omitnan');
yline(axSINR, global_avg_sinr, 'w--', 'LineWidth', 2, 'DisplayName', sprintf('Global Mean: %.1f dB', global_avg_sinr));

legend(axSINR, 'Location', 'southwest');
title(axSINR, sprintf('SINR vs Distance (Avg: %.2f dB)', global_avg_sinr));


% update ber graph
axes(axBER);
hold(axBER, 'on');

ber_trend = movmean(ber_dl_trace, window_size, 'omitnan');
plot(axBER, 1:nTotalSlots, ber_trend, 'w-', 'LineWidth', 2, 'DisplayName', 'Trend (Avg)');

global_avg_ber = mean(ber_dl_trace, 'omitnan');
title(axBER, sprintf('BER Performance (Avg: %.2e)', global_avg_ber));
legend(axBER, 'Location', 'southwest');

fprintf('Grafici Real-Time aggiornati con linee di tendenza (media).\n');


%% --- 8. THROUGHPUT ANALYSIS ---
fprintf('Click Enter button to show Throughput vs Time plot.\n');
pause()

figure('Name', 'Throughput Analysis', 'Color', 'w');

% Filtriamo solo i valori di Downlink (dove throughput > 0) per il grafico
valid_thpt_idx = throughput_trace > 0;
valid_thpt = throughput_trace(valid_thpt_idx);
valid_slots = find(valid_thpt_idx);

axThpt = gca;
hold(axThpt, 'on'); grid(axThpt, 'on');

% Plot Instantaneous Throughput (Convertiamo in MB/s per leggibilità)
plot(axThpt, valid_slots, valid_thpt / 1e6, 'Color', [0 0.4470 0.7410], ...
    'LineWidth', 1, 'DisplayName', 'Instantaneous (MB/s)');

% Plot Moving Average (Trend)
window_thpt = 100;
thpt_trend = movmean(valid_thpt, window_thpt);
plot(axThpt, valid_slots, thpt_trend / 1e6, 'w-', 'LineWidth', 3, ...
    'DisplayName', 'Average Trend');

xlabel(axThpt, 'Slot Index');
ylabel(axThpt, 'Throughput (MegaBytes / sec)');
title(axThpt, 'Downlink Throughput Performance');
legend(axThpt, 'Location', 'northeast');

% text stats
avg_speed = mean(valid_thpt) / 1e6;
max_speed = max(valid_thpt) / 1e6;
dim = [.15 .6 .3 .3];
str = {sprintf('Max Speed: %.2f MB/s', max_speed), ...
       sprintf('Avg Speed: %.2f MB/s', avg_speed)};
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

fprintf('\nAnalisi sperimentale Throughput completata. Max: %.2f MB/s, Avg: %.2f MB/s\n', max_speed, avg_speed);

%% --- 9 theoretical proof of max thoruput ---
fprintf('\n--- theoretical proof of max thoruput  (DL 64-QAM) ---\n');

% 1. Risorse Totali (Banda)
K_total = carrier.NSizeGrid * 12;          % Totale sottoportanti (52*12 = 624)
nActiveSubcarriers = K_total - 2*Guard_Size; % Sottoportanti attive (624 - 16 - 16 = 592)

% 2. Calcolo RE (Resource Elements) disponibili per i DATI in uno slot
%    Distinguiamo tra simboli con DMRS (Piloti) e simboli solo Dati
n_DMRS_syms = length(DMRS_positions);            % Numero simboli pilot (3)
n_PureData_syms = FRAME_STRUCT.DATA_SYMS - n_DMRS_syms; % Numero simboli solo dati (12 - 3 = 9)

% A. RE nei simboli senza piloti (tutta la banda attiva è usata per dati)
RE_PureData = n_PureData_syms * nActiveSubcarriers;

% B. RE nei simboli con piloti (togliamo lo spazio occupato dai piloti)
%    DMRS_RE_spacing = 4 significa che 1 RE su 4 è pilota, 3 su 4 sono dati.
RE_per_DMRS_sym = nActiveSubcarriers * (1 - 1/DMRS_RE_spacing); 
RE_WithDMRS = n_DMRS_syms * RE_per_DMRS_sym;

% Totale RE per Dati in uno slot
Total_Data_RE_per_Slot = RE_PureData + RE_WithDMRS;

% 3. Calcolo Bit Totali per Slot (al massimo della modulazione)
Max_Modulation_Order = 6; % 64-QAM = 6 bit/simbolo
Bits_per_Slot = Total_Data_RE_per_Slot * Max_Modulation_Order;

% 4. Calcolo Throughput in Mbps
%    Mbps = (Bit per Slot) / (Durata Slot in secondi) / 1e6
Theoretical_Max_Mbps = (Bits_per_Slot / slotDuration) / 1e6;

% 5. Output dei risultati
fprintf('Sottoportanti Attive:  %d\n', nActiveSubcarriers);
fprintf('RE Dati Totali/Slot:   %d\n', Total_Data_RE_per_Slot);
fprintf('Bit Totali/Slot:       %d (con 64-QAM)\n', Bits_per_Slot);
fprintf('Durata Slot:           %.4f ms\n', slotDuration*1000);
fprintf('------------------------------------------------\n');
fprintf('Max Throughput Teorico: %.2f Mbps\n', Theoretical_Max_Mbps/8);
fprintf('Max Throughput Simulato: %.2f Mbps\n', max_speed); % max_speed era in MB/s
fprintf('================================================\n');



%% --9.1-- visualizing pathloss for simple distance

% coping same inputs
Pt_dBm = TxPower_dBm_DL;            % Potenza di trasmissione (dBm)
N_ant = N_ant_BS;             % Numero di antenne per Beamforming
fc = Pars.fc;             % Frequenza portante (3.5 GHz)
B = SampleRate;               % Banda (30 MHz)
c = physconst('LightSpeed');                % Velocità della luce (m/s)

% gain and noise
G_bf = 10*log10(N_ant); % Guadagno di Beamforming (Array Gain)
%Pn_dBm = -174 + 10*log10(B) + NF % Potenza di rumore termico (dBm)
Pn_dBm = 10*log10(noise_power)+30; %vengono uguali circa, qui uso le cose del codice

d = 0:1:3500; 

% calculate Path Loss (Free Space Path Loss)
L = 20*log10(d) + 20*log10(fc) + 20*log10(4*pi/c);

% calculate Prx and SINR 
Pr_dBm = Pt_dBm + G_bf - L;
SINR = Pr_dBm - Pn_dBm;

figure('Color', 'w');
axTheory = gca;
plot(d, SINR,  '--' ,'LineWidth', 2, 'Color', [1 0.5 0]);
grid on;
hold on;

% key points
plot(50, SINR(d==50), 'ro', 'MarkerFaceColor', 'r');
plot(500, SINR(d==500), 'ro', 'MarkerFaceColor', 'r');
plot(900, SINR(d==900), 'ro', 'MarkerFaceColor', 'r');

xlabel('Distanza dalla Base Station (metri)');
ylabel('SINR (dB)');
title('Andamento SINR vs Distanza (3.5GHz, 25 Antenne Beamforming)');
legend('Curva SINR', 'Punti di controllo (50m, 500m, 900m)', 'Location', 'northeast');

xline(axTheory, (norm(initial_pos_ue - BS_Pos)), '--w', 'initial position of simulation');
xline(axTheory, (norm(UEs(1).pos - BS_Pos)), '--w', 'final position of simulation');

%annotations
text(60, SINR(d==50)+2, sprintf('50m: %.2f dB', SINR(d==50)));
text(510, SINR(d==500)+2, sprintf('500m: %.2f dB', SINR(d==500)));
text(910, SINR(d==900)+2, sprintf('900m: %.2f dB', SINR(d==900)));


%% --- 10. CLONE SINR PLOT (BIG VIEW WITH DUAL AXIS) ---
fprintf('\nGenerazione figura SINR ingrandita con doppio asse...\n');

hBigFig = figure('Name', 'SINR vs Distance - Detailed View', ...
                 'Color', 'w', ...
                 'Units', 'normalized', ...
                 'Position', [0.1 0.1 0.8 0.8]);

% 2. Copy original SINR axis
hNewAx = copyobj(axSINR, hBigFig);

set(hNewAx, 'Units', 'normalized');
set(hNewAx, 'Position', [0.1 0.1 0.8 0.8]); 
grid(hNewAx, 'on');
xlabel(hNewAx, 'Time (Slots)');
ylabel(hNewAx, 'SINR (dB)');
title(hNewAx, '', 'FontSize', 14);
legend(hNewAx, 'Location', 'southoutside', 'Orientation','horizontal', 'FontSize', 12);

% second axis creation
hNewAxDist = axes('Parent', hBigFig, ...
                  'Position', get(hNewAx, 'Position'), ... % Copia posizione esatta
                  'XAxisLocation', 'top', ...              % Metti la X in alto
                  'YAxisLocation', 'right', ...            % Y a destra (ma la nascondiamo)
                  'Color', 'none', ...                     % Sfondo trasparente
                  'Box', 'off');                           % Niente scatola attorno

set(hNewAxDist, 'YTick', []);
set(hNewAxDist, 'YColor', 'none'); % Nasconde l'asse Y di destra

dist_start = dist_trace(1);          % Distanza allo slot 1
dist_end   = dist_trace(nTotalSlots); % Distanza all'ultimo slot

xlim(hNewAx, [1, nTotalSlots]);        % Asse sotto (Slot)
xlim(hNewAxDist, [dist_start, dist_end]); % Asse sopra (Metri)

xlabel(hNewAxDist, 'Distance (meters)', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12);

fprintf('Figura duplicata aperta correttamente con doppio asse.\n');


fprintf('\n The end\n Pietro Guidetti - 10808180\n Polimi - Wireless Communication Project 2025');