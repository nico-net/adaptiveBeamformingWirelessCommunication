close all; clear; clc;

%% ========================================================================
%  5G NR MULTI-BS BEAMFORMING SIMULATION WITH HANDOVER
%  - 2 Base Stations (BS1, BS2)
%  - 3 User Equipments (UE1=Target, UE2/UE3=Interferers)
%  - Realistic NR waveform generation with overhead
%  - CDL channel models with Doppler
%  - RSRP-based handover mechanism
%  - Active beamforming on serving BS for target UE
%% ========================================================================

%% 1. NR WAVEFORM CONFIGURATION
cfg = nrDLCarrierConfig;
cfg.FrequencyRange = 'FR1';
cfg.ChannelBandwidth = 20; % 20 MHz

% Numerology: 30 kHz SCS
cfg.SCSCarriers{1}.SubcarrierSpacing = 30;
cfg.SCSCarriers{1}.NSizeGrid = 51; % 51 RBs for 20 MHz

% Bandwidth Part
cfg.BandwidthParts{1}.SubcarrierSpacing = 30;
cfg.BandwidthParts{1}.NSizeBWP = 51;
cfg.BandwidthParts{1}.NStartBWP = 0;

% SS Burst (Synchronization overhead)
cfg.SSBurst.BlockPattern = 'Case C';
cfg.SSBurst.Enable = true;
cfg.SSBurst.TransmittedBlocks = [1 0 0 0];

% PDSCH Configuration
cfg.PDSCH{1}.Enable = true;
cfg.PDSCH{1}.PRBSet = 5:45; % Leave guard bands
cfg.PDSCH{1}.SymbolAllocation = [2 12]; % Leave space for control
cfg.PDSCH{1}.TargetCodeRate = 0.5;
cfg.PDSCH{1}.Modulation = '16QAM';

% Generate reference waveform (single subframe)
[waveform_ref, waveInfo] = nrWaveformGenerator(cfg);
SR = waveInfo.ResourceGrids(1).Info.SampleRate; % ~30.72 MHz
samplesPerSubframe = length(waveform_ref);

disp(['Sample Rate: ' num2str(SR/1e6) ' MHz']);
disp(['Samples per subframe: ' num2str(samplesPerSubframe)]);

%% 2. PHYSICAL PARAMETERS
fc = 4e9;                           % Carrier frequency: 4 GHz
c = physconst('lightspeed');
lambda = c/fc;
kB = physconst('Boltzmann');
T_amb = 290;                        % Ambient temperature (K)
NF_dB = 7;                          % Noise figure
BW_Hz = 20e6;                       % Bandwidth
noise_power = kB * T_amb * BW_Hz * 10^(NF_dB/10);
noise_std = sqrt(noise_power / 2);

% Transmit power
TxPower_dBm = 30;                   % 30 dBm = 1 W
TxPower_W = 10^((TxPower_dBm-30)/10);

%% 3. BASE STATION CONFIGURATION
% BS1: Origin
BS1.pos = [0; 0; 25];               % Position [x; y; z] in meters
BS1.antSize = [4, 4];               % 4x4 UPA
BS1.Nant = prod(BS1.antSize);
BS1.color = [0.8 0.2 0.2];

% BS2: Offset along X-axis
BS2.pos = [300; 50; 25];
BS2.antSize = [4, 4];
BS2.Nant = prod(BS2.antSize);
BS2.color = [0.2 0.2 0.8];

% Pre-calculate antenna element positions for both BSs
[y_idx, z_idx] = meshgrid(0:3, 0:3);
ant_y = y_idx(:) * lambda/2;
ant_z = z_idx(:) * lambda/2;

%% 4. USER EQUIPMENT CONFIGURATION
% UE1: TARGET (Mobile, linear trajectory)
UE1.pos = [50; -30; 1.5];
UE1.vel_kmh = 60;                   % 60 km/h
UE1.vel_ms = UE1.vel_kmh / 3.6;
UE1.dir = [1; 0.3; 0];
UE1.dir = UE1.dir / norm(UE1.dir);
UE1.color = [0.2 0.8 0.2];
UE1.serving_BS = 1;                 % Initially connected to BS1

% UE2: INTERFERER 1 (Stationary)
UE2.pos = [100; 40; 1.5];
UE2.vel_kmh = 0;
UE2.vel_ms = 0;
UE2.dir = [0; 0; 0];
UE2.color = [0.8 0.8 0.2];

% UE3: INTERFERER 2 (Circular motion)
UE3.radius = 80;
UE3.angle = pi/4;
UE3.pos = [150 + UE3.radius*cos(UE3.angle); 
           UE3.radius*sin(UE3.angle); 
           1.5];
UE3.vel_kmh = -40;
UE3.vel_ms = UE3.vel_kmh / 3.6;
UE3.omega = UE3.vel_ms / UE3.radius;
UE3.color = [0.8 0.2 0.8];

%% 5. CDL CHANNEL MODELS
% Create separate channel objects for each BS-UE link
channelLinks = struct();

% Function to create CDL channel with proper antenna configuration
create_cdl = @(vel_ms) create_cdl_channel(vel_ms, fc, c, SR);

% Initialize all 6 links (2 BS × 3 UE)
channelLinks.BS1_UE1 = create_cdl(UE1.vel_ms);
channelLinks.BS1_UE2 = create_cdl(UE2.vel_ms);
channelLinks.BS1_UE3 = create_cdl(abs(UE3.vel_ms));

channelLinks.BS2_UE1 = create_cdl(UE1.vel_ms);
channelLinks.BS2_UE2 = create_cdl(UE2.vel_ms);
channelLinks.BS2_UE3 = create_cdl(abs(UE3.vel_ms));

% Reset all channels
fields = fieldnames(channelLinks);
for i = 1:length(fields)
    reset(channelLinks.(fields{i}).cdl);
end

%% 6. BEAMFORMING INITIALIZATION
% Get number of receive antennas (will be determined after channel creation)
% For now, we'll initialize after the channels are created
% This will be updated in the loop section

% RLS parameters
lambda_rls = 0.95;
delta_rls = 100;

%% 7. HANDOVER PARAMETERS
handover_margin_dB = 3;             % Hysteresis margin
handover_timer = 0;
handover_TTT = 40;                  % Time-to-trigger (subframes)
measurement_history = [];
RSRP_window = 20;                   % Averaging window

%% 8. SIMULATION PARAMETERS
dt = 1e-3;                          % Time step: 1 ms (1 subframe)
numSubframes = 500;                 % Total simulation time: 500 ms
update_plot_every = 5;

%% 9. GRAPHICS SETUP
fig = figure('WindowState', 'maximized', 'Color', 'w');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% Tile 1: 3D Scenario
ax1 = nexttile([1 1]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
% Base Stations
plot3(BS1.pos(1), BS1.pos(2), BS1.pos(3), '^', 'Color', BS1.color, ...
      'MarkerSize', 12, 'MarkerFaceColor', BS1.color, 'DisplayName', 'BS1');
plot3(BS2.pos(1), BS2.pos(2), BS2.pos(3), '^', 'Color', BS2.color, ...
      'MarkerSize', 12, 'MarkerFaceColor', BS2.color, 'DisplayName', 'BS2');
% UEs
hUE1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), 'o', ...
             'Color', UE1.color, 'MarkerFaceColor', UE1.color, ...
             'MarkerSize', 8, 'DisplayName', 'UE1 (Target)');
hTraj1 = plot3(UE1.pos(1), UE1.pos(2), UE1.pos(3), '-', ...
               'Color', UE1.color, 'LineWidth', 1.5);
hUE2 = plot3(UE2.pos(1), UE2.pos(2), UE2.pos(3), 'o', ...
             'Color', UE2.color, 'MarkerFaceColor', UE2.color, ...
             'MarkerSize', 6, 'DisplayName', 'UE2 (Int)');
hUE3 = plot3(UE3.pos(1), UE3.pos(2), UE3.pos(3), 'o', ...
             'Color', UE3.color, 'MarkerFaceColor', UE3.color, ...
             'MarkerSize', 6, 'DisplayName', 'UE3 (Int)');
hTraj3 = plot3(UE3.pos(1), UE3.pos(2), UE3.pos(3), '-', ...
               'Color', UE3.color, 'LineWidth', 1);
% Connection line
hConn = plot3([BS1.pos(1) UE1.pos(1)], [BS1.pos(2) UE1.pos(2)], ...
              [BS1.pos(3) UE1.pos(3)], '--k', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('5G NR Scenario: 2 BS, 3 UE');
view(3); legend('Location', 'best');
xlim([-50 400]); ylim([-80 120]); zlim([0 30]);

% Tile 2: RSRP Measurements
ax2 = nexttile([1 1]);
hRSRP1 = plot(NaN, NaN, 'LineWidth', 2, 'Color', BS1.color, 'DisplayName', 'RSRP BS1');
hold on;
hRSRP2 = plot(NaN, NaN, 'LineWidth', 2, 'Color', BS2.color, 'DisplayName', 'RSRP BS2');
hRSRP_thr = plot(NaN, NaN, '--k', 'LineWidth', 1.5, 'DisplayName', 'Handover Margin');
grid on; legend; title('RSRP Measurements (UE1)');
xlabel('Subframe'); ylabel('RSRP (dBm)'); ylim([-120 -60]);

% Tile 3: Status Info
ax3 = nexttile([1 1]);
axis(ax3, 'off');
hText = text(0.1, 0.5, '', 'FontSize', 11, 'Interpreter', 'none', ...
             'VerticalAlignment', 'middle');

% Tile 4: SINR
ax4 = nexttile([1 1]);
hSINR = plot(NaN, NaN, 'LineWidth', 2, 'Color', [0.2 0.6 0.2]);
grid on; title('Target UE1 SINR');
xlabel('Subframe'); ylabel('SINR (dB)'); ylim([-10 40]);

% Tile 5: Throughput Estimate
ax5 = nexttile([1 1]);
hTput = plot(NaN, NaN, 'LineWidth', 2, 'Color', [0.8 0.4 0]);
grid on; title('Estimated Throughput (Shannon)');
xlabel('Subframe'); ylabel('Mbps'); ylim([0 100]);

% Tile 6: Handover Events
ax6 = nexttile([1 1]);
hHO_BS1 = stem(NaN, NaN, 'filled', 'Color', BS1.color, 'MarkerSize', 8, ...
               'DisplayName', 'Connected to BS1');
hold on;
hHO_BS2 = stem(NaN, NaN, 'filled', 'Color', BS2.color, 'MarkerSize', 8, ...
               'DisplayName', 'Connected to BS2');
grid on; legend; title('Handover Events');
xlabel('Subframe'); ylabel('Serving BS'); ylim([0.5 2.5]);
yticks([1 2]); yticklabels({'BS1', 'BS2'});

%% 10. HISTORY BUFFERS
rsrp_bs1_hist = [];
rsrp_bs2_hist = [];
sinr_hist = [];
tput_hist = [];
handover_hist = [];
ho_times = [];
ho_bs = [];

%% 11. MAIN SIMULATION LOOP
disp('Starting simulation...');

% Initialize beamforming weights based on actual receive antenna count
Nrx_size = channelLinks.BS1_UE1.cdl.ReceiveAntennaArray.Size;
Nrx_total = prod(Nrx_size);

W_BS1 = zeros(Nrx_total, 1);
W_BS1(1) = 1; % Initial omnidirectional

W_BS2 = zeros(Nrx_total, 1);
W_BS2(1) = 1;

P_rls_BS1 = eye(Nrx_total) * delta_rls;
P_rls_BS2 = eye(Nrx_total) * delta_rls;

for t = 1:numSubframes

    %% A. MOBILITY UPDATE
    % Update UE1 position (linear)
    UE1.pos = UE1.pos + UE1.dir * UE1.vel_ms * dt;

    % Update UE3 position (circular)
    UE3.angle = UE3.angle + UE3.omega * dt;
    UE3.pos(1) = 150 + UE3.radius * cos(UE3.angle);
    UE3.pos(2) = UE3.radius * sin(UE3.angle);

    %% B. GENERATE TX WAVEFORMS
    % Each UE transmits independently (uplink scenario)
    % For simplicity, replicate reference waveform
    tx_wf = waveform_ref * sqrt(TxPower_W);

    % Get number of transmit antennas from channel
    Ntx = channelLinks.BS1_UE1.cdl.TransmitAntennaArray.Size;
    Ntx_total = prod(Ntx); % Total number of TX antenna elements

    % Replicate across transmit antennas
    tx1 = repmat(tx_wf, 1, Ntx_total);
    tx2 = repmat(tx_wf .* exp(1j*rand*2*pi), 1, Ntx_total); % Phase offset
    tx3 = repmat(tx_wf .* exp(1j*rand*2*pi), 1, Ntx_total);

    %% C. CHANNEL PROPAGATION (Small Scale + Path Loss)

    % --- BS1 Channels ---
    % Path loss calculation
    dist_BS1_UE1 = norm(UE1.pos - BS1.pos);
    dist_BS1_UE2 = norm(UE2.pos - BS1.pos);
    dist_BS1_UE3 = norm(UE3.pos - BS1.pos);

    PL_BS1_UE1 = fspl(dist_BS1_UE1, lambda);
    PL_BS1_UE2 = fspl(dist_BS1_UE2, lambda);
    PL_BS1_UE3 = fspl(dist_BS1_UE3, lambda);

    atten_BS1_UE1 = 10^(-PL_BS1_UE1/20);
    atten_BS1_UE2 = 10^(-PL_BS1_UE2/20);
    atten_BS1_UE3 = 10^(-PL_BS1_UE3/20);

    % Small scale fading
    rx1_BS1_faded = channelLinks.BS1_UE1.cdl(tx1);
    rx2_BS1_faded = channelLinks.BS1_UE2.cdl(tx2);
    rx3_BS1_faded = channelLinks.BS1_UE3.cdl(tx3);

    % Apply path loss (average across antennas for scalar attenuation)
    rx1_BS1 = rx1_BS1_faded * atten_BS1_UE1;
    rx2_BS1 = rx2_BS1_faded * atten_BS1_UE2;
    rx3_BS1 = rx3_BS1_faded * atten_BS1_UE3;

    % --- BS2 Channels ---
    dist_BS2_UE1 = norm(UE1.pos - BS2.pos);
    dist_BS2_UE2 = norm(UE2.pos - BS2.pos);
    dist_BS2_UE3 = norm(UE3.pos - BS2.pos);

    PL_BS2_UE1 = fspl(dist_BS2_UE1, lambda);
    PL_BS2_UE2 = fspl(dist_BS2_UE2, lambda);
    PL_BS2_UE3 = fspl(dist_BS2_UE3, lambda);

    atten_BS2_UE1 = 10^(-PL_BS2_UE1/20);
    atten_BS2_UE2 = 10^(-PL_BS2_UE2/20);
    atten_BS2_UE3 = 10^(-PL_BS2_UE3/20);

    rx1_BS2_faded = channelLinks.BS2_UE1.cdl(tx1);
    rx2_BS2_faded = channelLinks.BS2_UE2.cdl(tx2);
    rx3_BS2_faded = channelLinks.BS2_UE3.cdl(tx3);

    rx1_BS2 = rx1_BS2_faded * atten_BS2_UE1;
    rx2_BS2 = rx2_BS2_faded * atten_BS2_UE2;
    rx3_BS2 = rx3_BS2_faded * atten_BS2_UE3;

    %% D. RSRP CALCULATION (Reference Signal Received Power)
    % RSRP = Average power of reference signals (approximated here)
    % Take mean power across time samples
    rsrp_BS1_UE1_linear = mean(mean(abs(rx1_BS1).^2, 1));
    rsrp_BS2_UE1_linear = mean(mean(abs(rx1_BS2).^2, 1));

    rsrp_BS1_UE1_dBm = 10*log10(rsrp_BS1_UE1_linear) + 30;
    rsrp_BS2_UE1_dBm = 10*log10(rsrp_BS2_UE1_linear) + 30;

    % Store measurements
    rsrp_bs1_hist(end+1) = rsrp_BS1_UE1_dBm;
    rsrp_bs2_hist(end+1) = rsrp_BS2_UE1_dBm;

    %% E. HANDOVER DECISION (Based on RSRP with Hysteresis)
    % Moving average of RSRP
    if length(rsrp_bs1_hist) >= RSRP_window
        rsrp_bs1_avg = mean(rsrp_bs1_hist(end-RSRP_window+1:end));
        rsrp_bs2_avg = mean(rsrp_bs2_hist(end-RSRP_window+1:end));
    else
        rsrp_bs1_avg = mean(rsrp_bs1_hist);
        rsrp_bs2_avg = mean(rsrp_bs2_hist);
    end

    % Current serving BS
    serving_BS = UE1.serving_BS;
    target_BS = serving_BS;

    % Check handover condition
    if serving_BS == 1
        % Check if BS2 is significantly better
        if rsrp_bs2_avg > rsrp_bs1_avg + handover_margin_dB
            handover_timer = handover_timer + 1;
            if handover_timer >= handover_TTT
                target_BS = 2;
                handover_timer = 0;
                disp(['[' num2str(t) '] HANDOVER: BS1 → BS2']);
                ho_times(end+1) = t;
                ho_bs(end+1) = 2;
            end
        else
            handover_timer = 0;
        end
    else % serving_BS == 2
        % Check if BS1 is significantly better
        if rsrp_bs1_avg > rsrp_bs2_avg + handover_margin_dB
            handover_timer = handover_timer + 1;
            if handover_timer >= handover_TTT
                target_BS = 1;
                handover_timer = 0;
                disp(['[' num2str(t) '] HANDOVER: BS2 → BS1']);
                ho_times(end+1) = t;
                ho_bs(end+1) = 1;
            end
        else
            handover_timer = 0;
        end
    end

    UE1.serving_BS = target_BS;
    handover_hist(end+1) = target_BS;

    %% F. BEAMFORMING (Only on Serving BS)
    % Get number of receive antennas from channel
    Nrx = channelLinks.BS1_UE1.cdl.ReceiveAntennaArray.Size;
    Nrx_total = prod(Nrx); % Total number of RX antenna elements

    % Combine signals at serving BS
    noise_BS = noise_std * (randn(samplesPerSubframe, Nrx_total) + ...
                            1j*randn(samplesPerSubframe, Nrx_total));

    if UE1.serving_BS == 1
        % BS1 is serving
        rx_total_BS1 = rx1_BS1 + rx2_BS1 + rx3_BS1 + noise_BS;

        % Extract center sample for beamforming update (simplified)
        mid_sample = round(samplesPerSubframe/2);
        x_obs = rx_total_BS1(mid_sample, :).'; % Observation vector

        % RLS update (simplified single-snapshot)
        Px = P_rls_BS1 * x_obs;
        g = Px / (lambda_rls + x_obs' * Px);
        y = W_BS1' * x_obs;

        % Reference: desired signal (pilot-like)
        d_ref = tx_wf(mid_sample) * atten_BS1_UE1;
        e = d_ref - y;

        % Update weights
        W_BS1 = W_BS1 + g * conj(e);
        P_rls_BS1 = (P_rls_BS1 - g * x_obs' * P_rls_BS1) / lambda_rls;

        % Apply beamforming
        rx_bf = rx_total_BS1 * W_BS1;

        % Calculate SINR components
        sig_power = mean(abs(rx1_BS1 * W_BS1).^2);
        int_power = mean(abs((rx2_BS1 + rx3_BS1) * W_BS1).^2);
        noise_power_bf = mean(abs(noise_BS * W_BS1).^2);

    else
        % BS2 is serving
        rx_total_BS2 = rx1_BS2 + rx2_BS2 + rx3_BS2 + noise_BS;

        mid_sample = round(samplesPerSubframe/2);
        x_obs = rx_total_BS2(mid_sample, :).';

        Px = P_rls_BS2 * x_obs;
        g = Px / (lambda_rls + x_obs' * Px);
        y = W_BS2' * x_obs;

        d_ref = tx_wf(mid_sample) * atten_BS2_UE1;
        e = d_ref - y;

        W_BS2 = W_BS2 + g * conj(e);
        P_rls_BS2 = (P_rls_BS2 - g * x_obs' * P_rls_BS2) / lambda_rls;

        rx_bf = rx_total_BS2 * W_BS2;

        sig_power = mean(abs(rx1_BS2 * W_BS2).^2);
        int_power = mean(abs((rx2_BS2 + rx3_BS2) * W_BS2).^2);
        noise_power_bf = mean(abs(noise_BS * W_BS2).^2);
    end

    %% G. PERFORMANCE METRICS
    % SINR (dB)
    sinr_linear = sig_power / (int_power + noise_power_bf + 1e-20);
    sinr_dB = 10*log10(sinr_linear);
    sinr_hist(end+1) = sinr_dB;

    % Throughput (Shannon capacity approximation)
    tput_Mbps = (BW_Hz / 1e6) * log2(1 + sinr_linear);
    tput_hist(end+1) = tput_Mbps;

    %% H. VISUALIZATION
    if mod(t, update_plot_every) == 0
        % Update positions
        set(hUE1, 'XData', UE1.pos(1), 'YData', UE1.pos(2), 'ZData', UE1.pos(3));
        set(hUE3, 'XData', UE3.pos(1), 'YData', UE3.pos(2), 'ZData', UE3.pos(3));

        % Update trajectories
        traj1_x = [get(hTraj1, 'XData'), UE1.pos(1)];
        traj1_y = [get(hTraj1, 'YData'), UE1.pos(2)];
        traj1_z = [get(hTraj1, 'ZData'), UE1.pos(3)];
        set(hTraj1, 'XData', traj1_x, 'YData', traj1_y, 'ZData', traj1_z);

        traj3_x = [get(hTraj3, 'XData'), UE3.pos(1)];
        traj3_y = [get(hTraj3, 'YData'), UE3.pos(2)];
        traj3_z = [get(hTraj3, 'ZData'), UE3.pos(3)];
        set(hTraj3, 'XData', traj3_x, 'YData', traj3_y, 'ZData', traj3_z);

        % Update connection line
        if UE1.serving_BS == 1
            set(hConn, 'XData', [BS1.pos(1) UE1.pos(1)], ...
                       'YData', [BS1.pos(2) UE1.pos(2)], ...
                       'ZData', [BS1.pos(3) UE1.pos(3)], ...
                       'Color', BS1.color);
        else
            set(hConn, 'XData', [BS2.pos(1) UE1.pos(1)], ...
                       'YData', [BS2.pos(2) UE1.pos(2)], ...
                       'ZData', [BS2.pos(3) UE1.pos(3)], ...
                       'Color', BS2.color);
        end

        % Update RSRP plot
        set(hRSRP1, 'XData', 1:length(rsrp_bs1_hist), 'YData', rsrp_bs1_hist);
        set(hRSRP2, 'XData', 1:length(rsrp_bs2_hist), 'YData', rsrp_bs2_hist);

        % Handover margin line
        if UE1.serving_BS == 1
            margin_line = rsrp_bs1_hist + handover_margin_dB;
        else
            margin_line = rsrp_bs2_hist + handover_margin_dB;
        end
        set(hRSRP_thr, 'XData', 1:length(margin_line), 'YData', margin_line);

        % Update status text
        status_str = sprintf(['Time: %d ms\n' ...
                             'Serving BS: %d\n' ...
                             'UE1 Position: [%.1f, %.1f, %.1f]\n' ...
                             'Distance BS1: %.1f m\n' ...
                             'Distance BS2: %.1f m\n' ...
                             'RSRP BS1: %.1f dBm\n' ...
                             'RSRP BS2: %.1f dBm\n' ...
                             'SINR: %.1f dB\n' ...
                             'Throughput: %.1f Mbps\n' ...
                             'Handovers: %d'], ...
                             t, UE1.serving_BS, UE1.pos(1), UE1.pos(2), UE1.pos(3), ...
                             dist_BS1_UE1, dist_BS2_UE1, ...
                             rsrp_BS1_UE1_dBm, rsrp_BS2_UE1_dBm, ...
                             sinr_dB, tput_Mbps, length(ho_times));
        set(hText, 'String', status_str);

        % Update SINR plot
        set(hSINR, 'XData', 1:length(sinr_hist), 'YData', sinr_hist);

        % Update throughput plot
        set(hTput, 'XData', 1:length(tput_hist), 'YData', tput_hist);

        % Update handover events
        if ~isempty(ho_times)
            ho_bs1_times = ho_times(ho_bs == 1);
            ho_bs2_times = ho_times(ho_bs == 2);
            set(hHO_BS1, 'XData', ho_bs1_times, 'YData', ones(size(ho_bs1_times)));
            set(hHO_BS2, 'XData', ho_bs2_times, 'YData', 2*ones(size(ho_bs2_times)));
        end

        drawnow limitrate;
    end
end

%% 12. FINAL STATISTICS
disp('=== SIMULATION COMPLETE ===');
disp(['Total Handovers: ' num2str(length(ho_times))]);
disp(['Average SINR: ' num2str(mean(sinr_hist), '%.2f') ' dB']);
disp(['Average Throughput: ' num2str(mean(tput_hist), '%.2f') ' Mbps']);
disp(['Final Position: [' num2str(UE1.pos', '%.1f ') '] m']);

% Calculate time in each BS
time_bs1 = sum(handover_hist == 1);
time_bs2 = sum(handover_hist == 2);
disp(['Time on BS1: ' num2str(time_bs1) ' ms (' num2str(100*time_bs1/numSubframes, '%.1f') '%)']);
disp(['Time on BS2: ' num2str(time_bs2) ' ms (' num2str(100*time_bs2/numSubframes, '%.1f') '%)']);

%% ========================================================================
%  HELPER FUNCTIONS
%% ========================================================================

function cdl_struct = create_cdl_channel(vel_ms, fc, c, SR)
    % Create CDL channel with proper antenna array configuration

    cdl = nrCDLChannel;
    cdl.DelayProfile = 'CDL-D';
    cdl.DelaySpread = 30e-9;
    cdl.CarrierFrequency = fc;
    cdl.MaximumDopplerShift = (vel_ms/c)*fc;
    cdl.SampleRate = SR;

    % Transmit Antenna Array (BS: 4x4 UPA with 2 polarizations)
    txArray = phased.NRRectangularPanelArray;
    txArray.Size = [4 4 1 2]; % [Mg Ng M N] - 4x4 grid, 1 panel, 2 polarizations
    txArray.Spacing = [0.5 0.5 1 1]*c/fc; % [dg dn dgv dnv] in meters
    txArray.ElementSet = {phased.NRAntennaElement};
    cdl.TransmitAntennaArray = txArray;

    % Receive Antenna Array (UE: 2x2 configuration with 2 polarizations)
    rxArray = phased.NRRectangularPanelArray;
    rxArray.Size = [2 2 1 2]; % [Mg Ng M N] - 2x2 grid, 1 panel, 2 polarizations
    rxArray.Spacing = [0.5 0.5 1 1]*c/fc; % [dg dn dgv dnv] in meters
    rxArray.ElementSet = {phased.NRAntennaElement};
    cdl.ReceiveAntennaArray = rxArray;

    % Return as structure
    cdl_struct = struct('cdl', cdl);
end
