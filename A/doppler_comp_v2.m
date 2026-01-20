%% ==================================================================
%  NLMS BEAMFORMING DOPPLER DEMO WITH ADJUSTABLE COMPENSATION (1 UE, HIGH SPEED)
% This simulation demonstrates adaptive beamforming (NLMS) under high Doppler
% effects with a configurable Doppler compensation mechanism. The goal is to
% show how pre-compensation of Doppler shift improves beamforming convergence
% and tracking performance. By adjusting doppler_comp_percent from 0% (no 
% compensation) to 100% (full compensation), users can evaluate the trade-off 
% between system complexity and performance gain.
% It's preferable to run this code with only 1 UE, otherwise the simulation
% will render very slowly. To keep the bandwidth constant the parameter
% carrier.NSizeGrid should follow an inverse trend wrt
% carrier.SubcarrierSpacing.
%% ==================================================================
close all; clear; clc;

%% --- DOPPLER COMPENSATION CONTROL ---
% Adjust this value from 0 (no compensation) to 100 (full compensation) %
% 0%   = No Doppler compensation (baseline: full Doppler effect on receiver)
% 50%  = Partial compensation (residual Doppler for NLMS to track)
% 100% = Full compensation (optimal case: minimal residual Doppler)
doppler_comp_percent = 50; % 0-100 %
doppler_comp_factor = doppler_comp_percent / 100; % Convert to 0-1 scale
fprintf('\n=== Doppler Compensation: %.1f%% ===\n', doppler_comp_percent);

%% --- 1. CONFIGURATION ---
% Basic 5G NR Carrier Setup
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 30; % 30 kHz spacing
carrier.NSizeGrid = 52;         % 52 Resource Blocks (about 20MHz of Bandwidth)
carrier.CyclicPrefix = 'Normal';
carrier.NSlot = 0;
ofdmInfo = nrOFDMInfo(carrier);
K = carrier.NSizeGrid * 12;     % Total subcarriers (624)
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

M_DL = 16; % Downlink Modulation (16-QAM)
M_UL = 4;  % Uplink Modulation (QPSK)

% NLMS Adaptive Filter Parameters
mu_min = 0.05;
mu_max = 0.9;
alpha_mu = 0.7; % Weight for updating mu
mu_current = 0.5;

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
nTotalSlots = 3000; % Extended duration to see Doppler sign flip (high-speed scenario)
Pars.fc = 3.5e9; % 3.5 GHz Carrier Frequency
slotDuration = 1e-3 / (carrier.SubcarrierSpacing/15);
BS_Pos = [0;0;25]; % in m
AntSize = [6,6]; % 36 Antenna Elements
N_ant_BS = prod(AntSize);

% Initialize Beamforming Weights
w_beam = ones(N_ant_BS, 1) ./ sqrt(N_ant_BS);

plCfg = nrPathLossConfig; plCfg.Scenario = 'UMa';
UEs = repmat(struct('pos', [], 'vel', [], 'channel_dl', [], 'channel_ul', [], 'hist_ber', []), N_UE, 1);
colors = lines(N_UE);

% Speed profile: ramp 350 -> 400 km/h (high-speed vehicle scenario)
v0_kmh = 350; v_target_kmh = 400;
v0 = v0_kmh / 3.6; v_target = v_target_kmh / 3.6;
vel_axis = [1; 0; 0]; vel_axis = vel_axis / norm(vel_axis); % Straight line toward +x to flip Doppler after passing BS
ramp_time = 30 * slotDuration;
acc_mag = 10; % m/s^2

for u = 1:N_UE
    UEs(u).pos = [-70; -60; 1.5]; % Starts left of BS and crosses it to flip Doppler sign
    UEs(u).vel = vel_axis * v0;
    UEs(u).color = colors(u,:);

    % Configure Downlink Channel (CDL-C Model with time-selective fading)
    cdl_dl = nrCDLChannel; cdl_dl.DelayProfile = 'CDL-C'; cdl_dl.DelaySpread = 30e-9;
    cdl_dl.CarrierFrequency = Pars.fc; cdl_dl.TransmitAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1];
    cdl_dl.ReceiveAntennaArray.Size = [1 1 1 1 1]; cdl_dl.SampleRate = SampleRate;
    cdl_dl.Seed = u*123; cdl_dl.ChannelFiltering = true;
    [~,~] = cdl_dl(complex(zeros(10,N_ant_BS)));

    % Configure Uplink Channel (CDL-C Model with time-selective fading)
    cdl_ul = nrCDLChannel; cdl_ul.DelayProfile = 'CDL-C'; cdl_ul.DelaySpread = 30e-9;
    cdl_ul.CarrierFrequency = Pars.fc; cdl_ul.TransmitAntennaArray.Size = [1 1 1 1 1];
    cdl_ul.ReceiveAntennaArray.Size = [AntSize(1) AntSize(2) 1 1 1]; cdl_ul.SampleRate = SampleRate;
    cdl_ul.Seed = u*123; cdl_ul.ChannelFiltering = true;

    UEs(u).channel_dl = cdl_dl; UEs(u).channel_ul = cdl_ul; UEs(u).hist_ber = nan(1, nTotalSlots);
end

% Transmission Power Config
TxPower_dBm_DL = 50; TxPower_dBm_UL = 30;
Tx_AmpDL = sqrt(10^((TxPower_dBm_DL-30)/10));
Tx_AmpUL = sqrt(10^((TxPower_dBm_UL-30)/10));

% Noise Config
noise_std = sqrt((physconst('Boltzmann') * 290 * SampleRate * 10^(5/10)) / 2);

%% --- 4. GRAPHICS ---
figure('WindowState', 'maximized', 'Name', "Doppler Compensation " + doppler_comp_percent + " %");
tlo = tiledlayout(2, 3, 'TileSpacing', 'compact');

axEnv = nexttile; hold(axEnv, 'on'); grid(axEnv, 'on'); axis(axEnv, 'equal');
view(axEnv, [40 25]); xlim(axEnv, [-100 100]); ylim(axEnv, [-100 40]); zlim(axEnv, [0 50]);
title(axEnv, 'Scenario'); plot3(axEnv, BS_Pos(1), BS_Pos(2), BS_Pos(3), '^', 'Color', [0.95 0.9 0.1], 'MarkerFaceColor', [0.95 0.9 0.1], 'MarkerSize', 12, 'LineWidth', 2);
hUE_scatters = gobjects(N_UE,1);
for u=1:N_UE, hUE_scatters(u) = plot3(axEnv, UEs(u).pos(1), UEs(u).pos(2), UEs(u).pos(3), 'o', 'Color', UEs(u).color, 'MarkerFaceColor', UEs(u).color); end

axBER = nexttile; hold(axBER, 'on'); grid(axBER, 'on'); set(axBER, 'YScale', 'log');
title(axBER, 'BER (DL/UL)'); ylim([1e-6 1]); xlim([0 nTotalSlots]);
hBER_DL = plot(axBER, NaN, NaN, 'o-', 'Color', 'b', 'LineWidth', 1.2, 'DisplayName', 'DL');
hBER_UL = plot(axBER, NaN, NaN, 's-', 'Color', 'r', 'LineWidth', 1.2, 'DisplayName', 'UL');
legend(axBER, 'Location', 'northeast');

axConstUL = nexttile; hConstUL = plot(axConstUL, NaN, NaN, '.r'); grid(axConstUL,'on'); axis(axConstUL,'square'); xlim([-2 2]); ylim([-2 2]); title(axConstUL, 'UL RX (Clean)');

axConstDL = nexttile; hConstDL = plot(axConstDL, NaN, NaN, '.b'); grid(axConstDL,'on'); axis(axConstDL,'square'); xlim([-2 2]); ylim([-2 2]); title(axConstDL, 'DL RX (Clean)');

axMSE = nexttile; hold(axMSE, 'on'); grid(axMSE, 'on');
title(axMSE, 'MSE (DL/UL)');
ylabel(axMSE, 'MSE (dB)'); xlabel(axMSE, 'Slot Index');
xlim(axMSE, [0 nTotalSlots]);
hMSE_DL = plot(axMSE, NaN, NaN, '-','Color','b','LineWidth', 1.2, 'DisplayName', 'DL MSE');
legend(axMSE, 'Location', 'best');

axDopp = nexttile; hold(axDopp,'on'); grid(axDopp,'on'); title(axDopp,'Doppler (Frames)');
set(axDopp, 'XLim', [0 nTotalSlots], 'XLimMode', 'manual');
hDopp = plot(axDopp, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'Doppler (Hz)');
% Horizontal lines for max/min possible Doppler
max_dopp = (v_target / physconst('LightSpeed')) * Pars.fc;
min_dopp = -max_dopp;
plot(axDopp, [0 nTotalSlots], [max_dopp max_dopp], 'r--', 'LineWidth', 1.2);
plot(axDopp, [0 nTotalSlots], [min_dopp min_dopp], 'r--', 'LineWidth', 1.2);
% Inline labels on the left side of the lines
x_label = max(1, round(0.02*nTotalSlots));
text(axDopp, x_label, max_dopp, 'Max Doppler', 'Color','r', 'FontSize', 9, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(axDopp, x_label, min_dopp, 'Min Doppler', 'Color','r', 'FontSize', 9, 'HorizontalAlignment','left', 'VerticalAlignment','top');
legend(axDopp, hDopp, 'Location', 'northeast');
hAgingText = text(0.98, 0.58, '', 'Units', 'normalized', 'Parent', axDopp, 'FontSize', 10, 'Color', 'k', 'BackgroundColor', [0.9 0.9 0.9], 'EdgeColor', 'k', 'HorizontalAlignment', 'right', 'Interpreter','tex');
hSpeedText = text(0.98, 0.82, '', 'Units', 'normalized', 'Parent', axDopp, 'FontSize', 10, 'Color', 'k', 'BackgroundColor', 'y', 'EdgeColor', 'k', 'HorizontalAlignment', 'right');
hDoppText = text(0.98, 0.70, '', 'Units', 'normalized', 'Parent', axDopp, 'FontSize', 10, 'Color', 'k', 'BackgroundColor', 'c', 'EdgeColor', 'k', 'HorizontalAlignment', 'right');

ber_dl_trace = nan(1, nTotalSlots); ber_ul_trace = nan(1, nTotalSlots);
mse_dl_trace = nan(1, nTotalSlots); mse_ul_trace = nan(1, nTotalSlots);
update_slots = [];
dopp_hist = []; speed_hist = []; time_hist = [];
last_const_ul = [];
last_const_dl = [];
last_ul_update_time = 0;

%% --- 5. SIMULATION LOOP ---
fprintf('\n=== Starting Doppler Demo with Compensation (1 UE) ===\n');
simTime = 0;
for slot_idx = 1:nTotalSlots
    slotInPattern = mod(slot_idx-1, SlotsPerPattern);
    isDownlink = (slotInPattern < Pattern_DL);
    simTime = simTime + slotDuration;

    for u = 1:N_UE
        % Velocity ramp
        if norm(UEs(u).vel) < v_target
            speed_now = min(norm(UEs(u).vel) + acc_mag * slotDuration, v_target);
            UEs(u).vel = vel_axis * speed_now;
        end
        UEs(u).pos = UEs(u).pos + UEs(u).vel * slotDuration;
        set(hUE_scatters(u), 'XData', UEs(u).pos(1), 'YData', UEs(u).pos(2), 'ZData', UEs(u).pos(3));

        UEs(u).channel_dl.InitialTime = simTime;
        UEs(u).channel_ul.InitialTime = simTime;

        % Speed and Doppler logging
        radial_vec = BS_Pos - UEs(u).pos; radial_unit = radial_vec / (norm(radial_vec) + 1e-12);
        v_rad = dot(UEs(u).vel, radial_unit);
        doppler_curr = (v_rad / physconst('LightSpeed')) * Pars.fc;
        time_hist(end+1) = simTime; dopp_hist(end+1) = doppler_curr; speed_hist(end+1) = norm(UEs(u).vel)*3.6;

        % Apply dynamic Doppler to channels (time-selective fading)
        UEs(u).channel_dl.MaximumDopplerShift = abs(doppler_curr);
        UEs(u).channel_ul.MaximumDopplerShift = abs(doppler_curr);

        if isDownlink
            %% --- DOWNLINK ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, N_ant_BS);
            all_bits = []; release(UEs(u).channel_dl);
            w_tx = conj(w_beam); w_tx = w_tx / max(abs(w_tx));
            
            % Build data grid with DMRS and data symbols
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
            
            % OFDM Modulation and Transmission
            tx_slot = build_slot(carrier, sync_grid, data_grids, FRAME_STRUCT.GUARD_SYMS, N_ant_BS);
            tx_slot = tx_slot * Tx_AmpDL;
            
            % Channel + Fading
            [rx_faded, ~] = UEs(u).channel_dl(tx_slot);
            pl_dB = nrPathLoss(plCfg, Pars.fc, 1, BS_Pos, UEs(u).pos);
            rx_total = rx_faded * 10^(-pl_dB/20) + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            % --- DOPPLER COMPENSATION in time domain (DL) ---
            % Apply phase correction based on current Doppler and compensation level
            if doppler_comp_factor > 0
                t_samples = (0:length(rx_total)-1)' / SampleRate;
                doppler_to_comp = doppler_comp_factor * doppler_curr;
                phase_correction = exp(1j * 2 * pi * doppler_to_comp * t_samples);
                rx_total = rx_total .* phase_correction;
            end
            
            % Synchronization & Demodulation
            offset_dl = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_dl:end, :);
            neededLen = sum(ofdmInfo.SymbolLengths(1:FRAME_STRUCT.TOTAL));
            if size(rx_sync,1) < neededLen, rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, 1)]; end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL, continue; end
            rx_data_grids = rx_grid_full(1:K, 2:13, 1);

            % Channel estimation from DMRS with polar interpolation
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

            % Equalization and demodulation with MSE calculation
            rx_bits_total = []; current_const_dl = []; mse_dl_error = 0; mse_dl_count = 0;
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                rx_eq_full = rx_data_grids(:, sym) ./ (H_est_total(:, sym) + 1e-8);
                if ismember(sym, DMRS_positions), rx_useful = rx_eq_full(Data_indices); else, rx_useful = rx_eq_full(Active_Indices); end
                rx_bits_total = [rx_bits_total; qamdemod(rx_useful, M_DL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true)];
                current_const_dl = [current_const_dl; rx_useful];
                % Calculate MSE for DL (error from received to nearest constellation point)
                modulated = qammod(qamdemod(rx_useful, M_DL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true), M_DL, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
                mse_dl_error = mse_dl_error + mean(abs(rx_useful - modulated).^2);
                mse_dl_count = mse_dl_count + 1;
            end
            if mse_dl_count > 0
                mse_dl_trace(slot_idx) = 10*log10(mse_dl_error / mse_dl_count + 1e-12);
            end
            last_const_dl = current_const_dl;
        else
            %% --- UPLINK ---
            data_grids = zeros(K, FRAME_STRUCT.DATA_SYMS, 1);
            all_bits = []; release(UEs(u).channel_ul);
            
            % Build data grid with DMRS and data symbols
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
            
            % OFDM Modulation and Transmission
            tx_slot = build_slot(carrier, sync_grid, data_grids, FRAME_STRUCT.GUARD_SYMS, 1);
            tx_slot = tx_slot * Tx_AmpUL;
            
            % Channel + Fading
            [rx_faded, ~] = UEs(u).channel_ul(tx_slot);
            pl_dB = nrPathLoss(plCfg, Pars.fc, 1, BS_Pos, UEs(u).pos);
            rx_total = rx_faded * 10^(-pl_dB/20) + noise_std * complex(randn(size(rx_faded)), randn(size(rx_faded)));
            
            % --- DOPPLER COMPENSATION in time domain (UL) ---
            % Apply phase correction based on current Doppler and compensation level
            if doppler_comp_factor > 0
                t_samples = (0:length(rx_total)-1)' / SampleRate;
                doppler_to_comp = doppler_comp_factor * doppler_curr;
                phase_correction = exp(1j * 2 * pi * doppler_to_comp * t_samples);
                rx_total = rx_total .* phase_correction;
            end
            
            % Synchronization & Demodulation
            offset_ul = nrTimingEstimate(carrier, rx_total, sync_grid);
            rx_sync = rx_total(1+offset_ul:end, :);
            neededLen = sum(ofdmInfo.SymbolLengths(1:FRAME_STRUCT.TOTAL));
            if size(rx_sync,1) < neededLen, rx_sync = [rx_sync; zeros(neededLen-size(rx_sync,1)+100, size(rx_sync,2))]; end
            rx_grid_full = nrOFDMDemodulate(carrier, rx_sync);
            if size(rx_grid_full, 2) < FRAME_STRUCT.TOTAL, continue; end
            rx_data_grids = rx_grid_full(1:K, 2:13, :);

            % DMRS extraction and AGC normalization
            rx_dmrs_raw = rx_data_grids(DMRS_indices, DMRS_positions, :);
            rx_power_avg = mean(abs(rx_dmrs_raw(:)).^2);
            agc_factor = 1 / sqrt(rx_power_avg);
            rx_dmrs_raw = rx_dmrs_raw * agc_factor;

            % Phase stabilization across DMRS symbols to reduce aging effect
            H_element_raw = rx_dmrs_raw ./ dmrs_seq;
            H_ref = H_element_raw(:, 1, :);
            rx_dmrs_stabilized = rx_dmrs_raw;
            for i = 2:length(DMRS_positions)
                H_curr = H_element_raw(:, i, :);
                phase_drift = angle(mean(H_curr .* conj(H_ref), 1));
                correction_factor = exp(-1j * phase_drift);
                rx_dmrs_stabilized(:, i, :) = rx_dmrs_raw(:, i, :) .* correction_factor;
            end

            % --- NLMS ADAPTIVE BEAMFORMING ---
            % Update weights to minimize error between desired and observed DMRS
            avg_sq_error = 0; count_samples = 0;
            for i = 1:length(DMRS_positions)
                rx_vecs = squeeze(rx_dmrs_stabilized(:, i, :));
                for re = 1:size(rx_vecs, 1)
                    u_in = rx_vecs(re, :).';
                    d_des = dmrs_seq(re);
                    y_out = w_beam' * u_in;
                    e = d_des - y_out;
                    w_beam = w_beam + (mu_current / (u_in' * u_in + 1e-6)) * u_in * conj(e);
                    avg_sq_error = avg_sq_error + abs(e)^2; count_samples = count_samples + 1;
                end
            end
            w_beam = w_beam / norm(w_beam);
            
            % Update Adaptive Mu based on convergence
            if count_samples > 0
                mse_current = avg_sq_error / count_samples;
                mse_ul_trace(slot_idx) = 10*log10(mse_current + 1e-12);
                target_mu = 3 * mse_current; target_mu = max(mu_min, min(mu_max, target_mu));
                mu_current = alpha_mu * mu_current + (1 - alpha_mu) * target_mu;
            end
            update_slots(end+1) = slot_idx;
            last_ul_update_time = simTime;

            % Data reception with beamformed channel estimation
            rx_bits_total = []; current_const_ul = [];
            H_eff_snapshots = zeros(K, length(DMRS_positions));
            for i = 1:length(DMRS_positions)
                sym = DMRS_positions(i);
                rx_combined_dmrs = squeeze(rx_data_grids(DMRS_indices, sym, :)) * conj(w_beam);
                tmp = rx_combined_dmrs ./ dmrs_seq;
                H_eff_snapshots(:, i) = interp1(double(DMRS_indices), double(tmp), (1:K).', 'linear', 'extrap');
            end
            mag_snaps = abs(H_eff_snapshots).'; phase_snaps = unwrap(angle(H_eff_snapshots).');
            mag_interp = interp1(DMRS_positions, mag_snaps, (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            phase_interp = interp1(DMRS_positions, phase_snaps, (1:FRAME_STRUCT.DATA_SYMS).', 'linear', 'extrap');
            H_est_eff_time = (mag_interp .* exp(1j * phase_interp)).';
            for sym = 1:FRAME_STRUCT.DATA_SYMS
                rx_combined = squeeze(rx_data_grids(:, sym, :)) * conj(w_beam);
                rx_eq_full = rx_combined ./ (H_est_eff_time(:, sym) + 1e-8);
                if ismember(sym, DMRS_positions), rx_useful = rx_eq_full(Data_indices); else, rx_useful = rx_eq_full(Active_Indices); end
                rx_bits_total = [rx_bits_total; qamdemod(rx_useful, M_UL, 'gray', 'OutputType', 'bit', 'UnitAveragePower', true)];
                current_const_ul = [current_const_ul; rx_useful];
            end
            last_const_ul = current_const_ul;
        end

        % BER Calculation
        nBit = min(length(all_bits), length(rx_bits_total));
        if nBit > 0
            ber = sum(all_bits(1:nBit) ~= rx_bits_total(1:nBit)) / nBit;
            if isDownlink, ber_dl_trace(slot_idx) = ber; else, ber_ul_trace(slot_idx) = ber; end
        end

        % Update graphics every 5 slots
        if mod(slot_idx, 5) == 0
            set(hBER_DL, 'XData', 1:slot_idx, 'YData', ber_dl_trace(1:slot_idx));
            set(hBER_UL, 'XData', 1:slot_idx, 'YData', ber_ul_trace(1:slot_idx));
            yTopBer = axBER.YLim(2);
            set(hConstUL, 'XData', real(last_const_ul), 'YData', imag(last_const_ul));
            set(hConstDL, 'XData', real(last_const_dl), 'YData', imag(last_const_dl));
            
            % Plot MSE only for valid (non-NaN) values with interpolation
            mse_dl_valid = ~isnan(mse_dl_trace(1:slot_idx));
            
            % Interpolate DL MSE between valid points for continuous line
            dl_valid_idx = find(mse_dl_valid);
            if length(dl_valid_idx) > 1
                dl_interp_x = linspace(1, slot_idx, slot_idx);
                dl_interp_y = interp1(dl_valid_idx, mse_dl_trace(mse_dl_valid), dl_interp_x, 'linear', 'extrap');
                set(hMSE_DL, 'XData', dl_interp_x, 'YData', dl_interp_y);
            else
                set(hMSE_DL, 'XData', find(mse_dl_valid), 'YData', mse_dl_trace(mse_dl_valid));
            end
            
            set(hDopp, 'XData', 1:length(dopp_hist), 'YData', dopp_hist);
            set(hSpeedText, 'String', sprintf('Speed: %.1f km/h', speed_hist(end)));
            set(hDoppText, 'String', sprintf('Doppler: %.0f Hz', dopp_hist(end)));
            if last_ul_update_time > 0
                set(hAgingText, 'String', sprintf('Aging \\Delta t: %.1f ms', (simTime - last_ul_update_time)*1e3));
            else
                set(hAgingText, 'String', 'Aging \\Delta t: N/A');
            end
            drawnow limitrate;
        end
    end
end
fprintf('Simulation done.\n');

%% --- HELPER FUNCTIONS ---
function tx_slot = build_slot(carrier, sync_grid, data_grids, guard_count, N_ant)
    sync_full = repmat(sync_grid, [1, 1, N_ant]);
    guard_grid = zeros(size(sync_grid,1), guard_count, N_ant);
    full_grid = cat(2, sync_full, data_grids, guard_grid);
    tx_slot = nrOFDMModulate(carrier, full_grid);
end

function result = iif(condition, true_val, false_val)
    if condition, result = true_val; else, result = false_val; end
end
