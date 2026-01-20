close all; clear; clc;

%% ========================================================================
%  5G NR MULTI-BS SIMULATION WITH REALISTIC HANDOVER
%  - 2 Base Stations with beamforming
%  - 3 UEs (1 target + 2 interferers)
%  - 3GPP NR CDL channel modeling
%  - RSRP-based handover with hysteresis
% ========================================================================

%% 1. NR PHYSICAL LAYER CONFIGURATION
% =========================================================================
scs = 30e3;                      % Subcarrier spacing (30 kHz for FR1)
Nfft = 64;                       % FFT size
CPLength = 16;                   
SR = 30.72e6;                    % Sample rate (30.72 MSPS for 20 MHz)

% Modulation
M = 16;                          % 16-QAM

% Pilot configuration (CSI-RS positions)
pilotIndices = [12; 26; 40; 54];
pilotVal = 1 + 1j;

% Guard bands
GuardBands = [6; 6];
all_indices = (1:Nfft).';
guard_lower = (1:GuardBands(1)).';
guard_upper = (Nfft - GuardBands(2) + 1 : Nfft).';
guards = [guard_lower; guard_upper];
occupied_indices = setdiff(all_indices, guards);
dataIndices = setdiff(occupied_indices, pilotIndices);
NumDataCarriers = length(dataIndices);

%% 2. PHYSICAL PARAMETERS
% =========================================================================
Pars.fc = 3.5e9;                 % 3.5 GHz (n78 band)
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;

% Antenna arrays (UPA configuration)
antSize = [4, 4];                % 4×4 UPA at each BS
Nant = prod(antSize);

%% 3. BASE STATION DEPLOYMENT
% =========================================================================
BS1.pos = [-30; 0; 25];          % BS1 position [x; y; z] meters
BS1.id = 1;
BS1.color = 'r';
BS1.W = zeros(Nant, Nfft);       % Beamforming weights
BS1.W(1, :) = 1;                 % Initialize
BS1.P_rls = repmat(eye(Nant)*100, 1, 1, length(pilotIndices));

BS2.pos = [30; 0; 25];           % BS2 position
BS2.id = 2;
BS2.color = 'b';
BS2.W = zeros(Nant, Nfft);
BS2.W(1, :) = 1;
BS2.P_rls = repmat(eye(Nant)*100, 1, 1, length(pilotIndices));

BSs = {BS1, BS2};

%% 4. USER EQUIPMENT DEPLOYMENT
% =========================================================================
% UE1: TARGET (mobile)
UE1.id = 1;
UE1.pos = [-40; -15; 1.5];
UE1.vel_kmh = 60;
UE1.vel_ms = UE1.vel_kmh / 3.6;
UE1.dir = [1; 0.3; 0];
UE1.dir = UE1.dir / norm(UE1.dir);
UE1.serving_BS = 1;
UE1.color = 'g';

% UE2: INTERFERER 1 (stationary)
UE2.id = 2;
UE2.pos = [-20; 10; 1.5];
UE2.vel_kmh = 0;
UE2.vel_ms = 0;
UE2.dir = [0; 0; 0];
UE2.serving_BS = 1;
UE2.color = 'm';

% UE3: INTERFERER 2 (slow mobile)
UE3.id = 3;
UE3.pos = [25; -20; 1.5];
UE3.vel_kmh = 20;
UE3.vel_ms = UE3.vel_kmh / 3.6;
UE3.dir = [-0.5; 1; 0];
UE3.dir = UE3.dir / norm(UE3.dir);
UE3.serving_BS = 2;
UE3.color = 'c';

UEs = {UE1, UE2, UE3};

%% 5. 3GPP CDL CHANNEL MODELING
% =========================================================================
% Create CDL channel objects for each UE-BS pair
for ue_idx = 1:length(UEs)
    for bs_idx = 1:length(BSs)
        ue = UEs{ue_idx};
        bs = BSs{bs_idx};
        rel_vec = ue.pos - bs.pos;
        dist = norm(rel_vec);
        
        % Calculate Doppler
        if ue.vel_ms > 0
            k_wave = rel_vec / dist;
            fd = abs(dot(ue.dir * ue.vel_ms, k_wave)) / Pars.lambda;
        else
            fd = 0.1;
        end
        
        % Create 3GPP CDL-D channel (Urban NLOS)
        cdl = nrCDLChannel;
        cdl.DelayProfile = 'CDL-D';
        cdl.DelaySpread = 30e-9;
        cdl.CarrierFrequency = Pars.fc;
        cdl.MaximumDopplerShift = fd;
        cdl.SampleRate = SR;
        cdl.TransmitAntennaArray.Size = [antSize(1), antSize(2), 1, 1, 1];
        cdl.ReceiveAntennaArray.Size = [1, 1, 1, 1, 1];
        
        UEs{ue_idx}.channels{bs_idx} = cdl;
        UEs{ue_idx}.pathLoss_dB(bs_idx) = fspl(dist, Pars.lambda);
    end
end

%% 6. POWER AND NOISE CONFIGURATION
% =========================================================================
kB = physconst('Boltzmann');
T_amb = 290;
NF_dB = 7;
noise_bw = scs;
noise_power_per_sc = kB * T_amb * noise_bw * 10^(NF_dB/10);

TxPower_Total_dBm = 3;           % 3 dBm per UE
TxPower_Total_Lin = 10^((TxPower_Total_dBm-30)/10);
num_active_subcarriers = NumDataCarriers + length(pilotIndices);
TxPower_per_SC = TxPower_Total_Lin / num_active_subcarriers;
Tx_Amplitude = sqrt(TxPower_per_SC);

%% 7. HANDOVER PARAMETERS
% =========================================================================
HO.RSRP_offset_dB = 3;
HO.timeToTrigger = 40e-3;
HO.measurementPeriod = 5e-3;
HO.L3_filterCoeff = 0.5;

UEs{1}.RSRP_filtered = [-Inf, -Inf];
UEs{1}.HO_timer = 0;
UEs{1}.HO_candidate = 0;

%% 8. SIMULATION PARAMETERS
% =========================================================================
dt = 0.001;                      % 1 ms
nSymbols = 10000;                % 10 seconds

%% 9. DATA GENERATION
% =========================================================================
for ue_idx = 1:length(UEs)
    bitsTx = randi([0 1], NumDataCarriers * log2(M) * nSymbols, 1);
    symTx = qammod(bitsTx, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
    UEs{ue_idx}.dataMat = reshape(symTx, NumDataCarriers, nSymbols);
end

%% 10. GRAPHICS SETUP
% =========================================================================
fig = figure('WindowState', 'maximized', 'Color', 'w');
tlo = tiledlayout(2, 3, 'TileSpacing', 'compact');

% Map view
ax1 = nexttile([1 1]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
plot3(BS1.pos(1), BS1.pos(2), BS1.pos(3), '^', 'MarkerSize', 15, ...
    'MarkerFaceColor', BS1.color, 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'BS1');
plot3(BS2.pos(1), BS2.pos(2), BS2.pos(3), '^', 'MarkerSize', 15, ...
    'MarkerFaceColor', BS2.color, 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'BS2');

hUE1 = plot3(UEs{1}.pos(1), UEs{1}.pos(2), UEs{1}.pos(3), 'o', 'MarkerSize', 10, ...
    'MarkerFaceColor', UEs{1}.color, 'MarkerEdgeColor', 'k', 'LineWidth', 2, 'DisplayName', 'Target');
hTraj1 = plot3(UEs{1}.pos(1), UEs{1}.pos(2), UEs{1}.pos(3), '-', 'Color', UEs{1}.color, 'LineWidth', 1.5);
hLink1 = plot3([BS1.pos(1) UEs{1}.pos(1)], [BS1.pos(2) UEs{1}.pos(2)], [BS1.pos(3) UEs{1}.pos(3)], ...
    '--', 'Color', BS1.color, 'LineWidth', 2);

hUE2 = plot3(UEs{2}.pos(1), UEs{2}.pos(2), UEs{2}.pos(3), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', UEs{2}.color, 'MarkerEdgeColor', 'k', 'DisplayName', 'Int1');
hUE3 = plot3(UEs{3}.pos(1), UEs{3}.pos(2), UEs{3}.pos(3), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', UEs{3}.color, 'MarkerEdgeColor', 'k', 'DisplayName', 'Int2');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Network Topology & Handover');
view(2); xlim([-50 50]); ylim([-30 30]);
legend('Location', 'best');

% Constellation
ax2 = nexttile([1 1]);
hConst = plot(NaN, NaN, '.', 'Color', [0 0.4 0.8], 'MarkerSize', 4);
grid(ax2, 'on'); axis(ax2, 'square'); xlim([-2 2]); ylim([-2 2]);
title('Target UE Constellation (16-QAM)'); xline(0); yline(0);

% Info
axInfo = nexttile([1 1]); axis(axInfo, 'off');
hText = text(0.1, 0.5, '', 'FontSize', 10, 'Interpreter', 'none', 'VerticalAlignment', 'middle');

% MSE
ax3 = nexttile([1 1]);
hMSE = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r');
grid(ax3, 'on'); title('RLS Cost (MSE)');
xlabel('Symbol'); ylabel('MSE'); ylim([1e-5 10]);

% SINR
ax4 = nexttile([1 1]);
hSINR = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g');
grid(ax4, 'on'); title('Target UE SINR');
xlabel('Symbol'); ylabel('SINR (dB)'); ylim([0 60]);

% RSRP
ax5 = nexttile([1 1]);
hRSRP1 = plot(NaN, NaN, 'LineWidth', 2, 'Color', BS1.color, 'DisplayName', 'BS1');
hold(ax5, 'on');
hRSRP2 = plot(NaN, NaN, 'LineWidth', 2, 'Color', BS2.color, 'DisplayName', 'BS2');
hHO = stem(NaN, NaN, 'k', 'Marker', 'v', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'HO');
grid(ax5, 'on'); title('RSRP & Handover');
xlabel('Time (ms)'); ylabel('RSRP (dBm)'); legend('Location', 'best');

%% 11. SIMULATION LOOP
% =========================================================================
disp('Starting 5G NR simulation with 3GPP CDL channels...');

mse_hist = []; sinr_hist = []; const_buf = [];
time_vec = []; ho_times = []; ho_rsrp = [];
rsrp1_hist = []; rsrp2_hist = [];
lambda_rls = 0.94;

for t = 1:nSymbols
    current_time = (t-1) * dt;
    
    %% A. MOBILITY UPDATE
    % =====================================================================
    for ue_idx = 1:length(UEs)
        if UEs{ue_idx}.vel_ms > 0
            % Update position
            UEs{ue_idx}.pos = UEs{ue_idx}.pos + UEs{ue_idx}.dir * UEs{ue_idx}.vel_ms * dt;
            
            % Update path loss and Doppler for all channels
            for bs_idx = 1:length(BSs)
                rel_vec = UEs{ue_idx}.pos - BSs{bs_idx}.pos;
                dist = norm(rel_vec);
                UEs{ue_idx}.pathLoss_dB(bs_idx) = fspl(dist, Pars.lambda);
                
                % Update Doppler
                k_wave = rel_vec / dist;
                fd = abs(dot(UEs{ue_idx}.dir * UEs{ue_idx}.vel_ms, k_wave)) / Pars.lambda;
                UEs{ue_idx}.channels{bs_idx}.MaximumDopplerShift = max(fd, 0.1);
            end
        end
    end
    
    %% B. SIGNAL GENERATION & TRANSMISSION (3GPP CDL)
    % =====================================================================
    RX_signals = cell(1, length(BSs));
    for bs_idx = 1:length(BSs)
        RX_signals{bs_idx} = zeros(Nant, Nfft);
    end
    
    for ue_idx = 1:length(UEs)
        ue = UEs{ue_idx};
        
        % Build OFDM symbol
        TX_Grid = zeros(Nfft, 1);
        TX_Grid(dataIndices) = ue.dataMat(:, t) * Tx_Amplitude;
        TX_Grid(pilotIndices) = pilotVal * Tx_Amplitude;
        
        % IFFT + CP
        tx_td = ifft(TX_Grid, Nfft) * sqrt(Nfft);
        tx_td_cp = [tx_td(end-CPLength+1:end); tx_td];
        
        % Replicate across Tx antennas (single-antenna UE transmitting to multi-antenna BS)
        tx_waveform = repmat(tx_td_cp, 1, Nant);
        
        % Pass through 3GPP CDL channel to ALL BSs
        for bs_idx = 1:length(BSs)
            cdl_channel = ue.channels{bs_idx};
            
            % Apply CDL small-scale fading
            rx_faded = cdl_channel(tx_waveform);
            
            % Apply path loss
            pl_linear = 10^(-ue.pathLoss_dB(bs_idx) / 20);
            rx_faded = rx_faded * pl_linear;
            
            % Remove CP and FFT
            rx_no_cp = rx_faded(CPLength+1:end, :);
            RX_fd = fft(rx_no_cp, Nfft, 1) / sqrt(Nfft);
            
            % Transpose to [Nant x Nfft] and accumulate
            RX_signals{bs_idx} = RX_signals{bs_idx} + RX_fd.';
        end
    end
    
    % Add noise
    noise_std = sqrt(noise_power_per_sc / 2);
    for bs_idx = 1:length(BSs)
        noise = noise_std * (randn(Nant, Nfft) + 1j*randn(Nant, Nfft));
        RX_signals{bs_idx} = RX_signals{bs_idx} + noise;
    end
    
    %% C. RLS BEAMFORMING (Target UE at Serving BS)
    % =====================================================================
    target_bs_idx = UEs{1}.serving_BS;
    bs = BSs{target_bs_idx};
    
    % 1. Pilot extraction & CPE correction
    y_p = zeros(length(pilotIndices), 1);
    for i = 1:length(pilotIndices)
        k = pilotIndices(i);
        y_p(i) = bs.W(:, k)' * RX_signals{target_bs_idx}(:, k);
    end
    
    cpe = mean(angle(y_p .* conj(pilotVal)));
    if isnan(cpe), cpe = 0; end
    bs.W = bs.W * exp(1j * cpe);
    
    % 2. RLS weight update
    mse_s = 0;
    for i = 1:length(pilotIndices)
        k = pilotIndices(i);
        x = RX_signals{target_bs_idx}(:, k);
        P_curr = bs.P_rls(:, :, i);
        
        % Kalman gain
        Px = P_curr * x;
        g = Px / (lambda_rls + x' * Px);
        
        % Error
        y = bs.W(:, k)' * x;
        e = pilotVal - y;
        mse_s = mse_s + abs(e)^2;
        
        % Update
        bs.W(:, k) = bs.W(:, k) + g * conj(e);
        
        % Update P
        P_curr = (P_curr - g * x' * P_curr) / lambda_rls;
        bs.P_rls(:, :, i) = P_curr;
    end
    mse_hist(end+1) = mse_s;
    
    % 3. Weight interpolation
    w_p = bs.W(:, pilotIndices).';
    mag_i = interp1(pilotIndices, abs(w_p), 1:Nfft, 'linear', 'extrap');
    phs_i = interp1(pilotIndices, unwrap(angle(w_p)), 1:Nfft, 'linear', 'extrap');
    W_int = (mag_i .* exp(1j * phs_i)).';
    
    BSs{target_bs_idx} = bs;
    
    %% D. SINR CALCULATION (Simplified for CDL)
    % =====================================================================
    % For CDL channels, we estimate SINR from beamformer output
    sinr_avg = 0;
    num_valid = 0;
    
    for idx = 1:length(dataIndices)
        k = dataIndices(idx);
        w_k = W_int(:, k);
        
        % Received signal power (after beamforming)
        y_k = w_k' * RX_signals{target_bs_idx}(:, k);
        P_total = abs(y_k)^2;
        
        % Noise power
        P_N = (norm(w_k)^2) * noise_power_per_sc;
        
        % Estimate signal power (simplified)
        P_S = P_total - P_N;
        
        if P_S > 0
            sinr_avg = sinr_avg + P_S / P_N;
            num_valid = num_valid + 1;
        end
    end
    
    if num_valid > 0
        sinr_db = 10*log10(sinr_avg / num_valid);
    else
        sinr_db = -10;
    end
    sinr_hist(end+1) = sinr_db;
    
    %% E. OUTPUT SIGNAL EXTRACTION
    % =====================================================================
    y_out = zeros(length(dataIndices), 1);
    for i = 1:length(dataIndices)
        k = dataIndices(i);
        y_out(i) = W_int(:, k)' * RX_signals{target_bs_idx}(:, k);
    end
    
    const_buf = [const_buf; y_out];
    if length(const_buf) > 600, const_buf = const_buf(end-600:end); end
    
    %% F. RSRP MEASUREMENT & HANDOVER
    % =====================================================================
    if mod(t, round(HO.measurementPeriod/dt)) == 0
        for bs_idx = 1:length(BSs)
            rsrp_linear = 0;
            for i = 1:length(pilotIndices)
                k = pilotIndices(i);
                rsrp_linear = rsrp_linear + abs(RX_signals{bs_idx}(1, k))^2;
            end
            rsrp_dBm = 10*log10(rsrp_linear * 1000);
            
            % L3 filtering
            if UEs{1}.RSRP_filtered(bs_idx) == -Inf
                UEs{1}.RSRP_filtered(bs_idx) = rsrp_dBm;
            else
                UEs{1}.RSRP_filtered(bs_idx) = HO.L3_filterCoeff * UEs{1}.RSRP_filtered(bs_idx) + ...
                    (1 - HO.L3_filterCoeff) * rsrp_dBm;
            end
        end
        
        rsrp1_hist(end+1) = UEs{1}.RSRP_filtered(1);
        rsrp2_hist(end+1) = UEs{1}.RSRP_filtered(2);
        time_vec(end+1) = current_time * 1000;
        
        % Handover decision
        serving = UEs{1}.serving_BS;
        neighbor = 3 - serving;
        
        

        if UEs{1}.RSRP_filtered(neighbor) > UEs{1}.RSRP_filtered(serving) + HO.RSRP_offset_dB
            if UEs{1}.HO_candidate == neighbor
                UEs{1}.HO_timer = UEs{1}.HO_timer + HO.measurementPeriod;
                if UEs{1}.HO_timer >= HO.timeToTrigger
                    fprintf('Handover! UE1: BS%d -> BS%d at t=%.1f ms (ΔRSRP=%.1f dB)\n', ...
                        serving, neighbor, current_time*1000, ...
                        UEs{1}.RSRP_filtered(neighbor) - UEs{1}.RSRP_filtered(serving));
                    
                    UEs{1}.serving_BS = neighbor;
                    UEs{1}.HO_timer = 0;
                    UEs{1}.HO_candidate = 0;
                    
                    ho_times(end+1) = current_time * 1000;
                    ho_rsrp(end+1) = UEs{1}.RSRP_filtered(neighbor);
                end
            else
                UEs{1}.HO_candidate = neighbor;
                UEs{1}.HO_timer = 0;
            end
        else
            UEs{1}.HO_timer = 0;
            UEs{1}.HO_candidate = 0;
        end
    end
    
    %% G. VISUALIZATION
    % =====================================================================
    if mod(t, 20) == 0
        % Update positions
        set(hUE1, 'XData', UEs{1}.pos(1), 'YData', UEs{1}.pos(2), 'ZData', UEs{1}.pos(3));
        set(hUE2, 'XData', UEs{2}.pos(1), 'YData', UEs{2}.pos(2), 'ZData', UEs{2}.pos(3));
        set(hUE3, 'XData', UEs{3}.pos(1), 'YData', UEs{3}.pos(2), 'ZData', UEs{3}.pos(3));
        
        % Update trajectory
        traj_x = [get(hTraj1, 'XData'), UEs{1}.pos(1)];
        traj_y = [get(hTraj1, 'YData'), UEs{1}.pos(2)];
        traj_z = [get(hTraj1, 'ZData'), UEs{1}.pos(3)];
        set(hTraj1, 'XData', traj_x, 'YData', traj_y, 'ZData', traj_z);
        
        % Update link
        serving_bs = BSs{UEs{1}.serving_BS};
        set(hLink1, 'XData', [serving_bs.pos(1) UEs{1}.pos(1)], ...
            'YData', [serving_bs.pos(2) UEs{1}.pos(2)], ...
            'ZData', [serving_bs.pos(3) UEs{1}.pos(3)], ...
            'Color', serving_bs.color);
        
        % Update constellation
        if ~isempty(const_buf)
            pt = const_buf(~isnan(const_buf));
            if ~isempty(pt)
                n = mean(abs(pt));
                if n < 1e-9, n = 1; end
                set(hConst, 'XData', real(pt)/n, 'YData', imag(pt)/n);
            end
        end
        
        % Update plots
        set(hMSE, 'XData', 1:length(mse_hist), 'YData', mse_hist);
        set(hSINR, 'XData', 1:length(sinr_hist), 'YData', sinr_hist);
        
        if ~isempty(time_vec)
            set(hRSRP1, 'XData', time_vec, 'YData', rsrp1_hist);
            set(hRSRP2, 'XData', time_vec, 'YData', rsrp2_hist);
        end
        if ~isempty(ho_times)
            set(hHO, 'XData', ho_times, 'YData', ho_rsrp);
        end
        
        % Update text
        dist1 = norm(UEs{1}.pos - BSs{1}.pos);
        dist2 = norm(UEs{1}.pos - BSs{2}.pos);
        info_str = sprintf(['Sym: %d\n' ...
            'Serving BS: %d\n' ...
            'Dist BS1: %.1f m\n' ...
            'Dist BS2: %.1f m\n' ...
            'SINR: %.1f dB\n' ...
            'Handovers: %d'], ...
            t, UEs{1}.serving_BS, dist1, dist2, sinr_db, length(ho_times));
        set(hText, 'String', info_str);
        
        drawnow limitrate;
    end
end

disp('Simulation complete!');
fprintf('Total handovers: %d\n', length(ho_times));
fprintf('Final serving BS: %d\n', UEs{1}.serving_BS);