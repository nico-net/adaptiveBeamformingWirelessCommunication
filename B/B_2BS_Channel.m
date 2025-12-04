close all; clear; clc;
% rng(123);
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
BS1.pos = [0; 0; 10];    
BS2.pos = [50; 0; 10];
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

% UE 3: INTERFERER
UE3.radius = 30;
UE3.angle  = pi/3;
UE3.vel_kmh = -40;
UE3.vel_ms  = UE3.vel_kmh / 3.6;
UE3.omega   = UE3.vel_ms / UE3.radius;
UE3.pos = [UE3.radius*cos(UE3.angle); UE3.radius*sin(UE3.angle); 1.5];

% CDL Channel Configuration - LOS Urban Mode
% Channel impulse response objects for modeling
cdl_UE1_BS1 = nrCDLChannel;
cdl_UE1_BS1.DelayProfile = 'CDL-E';  % Urban LOS
cdl_UE1_BS1.DelaySpread = 30e-9;
cdl_UE1_BS1.CarrierFrequency = Pars.fc;
cdl_UE1_BS1.MaximumDopplerShift = UE1.vel_ms / Pars.lambda;
cdl_UE1_BS1.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE1_BS1.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE1_BS1.SampleRate = scs * Nfft;

cdl_UE1_BS2 = nrCDLChannel;
cdl_UE1_BS2.DelayProfile = 'CDL-E';
cdl_UE1_BS2.DelaySpread = 30e-9;
cdl_UE1_BS2.CarrierFrequency = Pars.fc;
cdl_UE1_BS2.MaximumDopplerShift = UE1.vel_ms / Pars.lambda;
cdl_UE1_BS2.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE1_BS2.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE1_BS2.SampleRate = scs * Nfft;

cdl_UE2_BS1 = nrCDLChannel;
cdl_UE2_BS1.DelayProfile = 'CDL-E';
cdl_UE2_BS1.DelaySpread = 30e-9;
cdl_UE2_BS1.CarrierFrequency = Pars.fc;
cdl_UE2_BS1.MaximumDopplerShift = abs(UE2.vel_ms) / Pars.lambda;
cdl_UE2_BS1.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE2_BS1.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE2_BS1.SampleRate = scs * Nfft;

cdl_UE2_BS2 = nrCDLChannel;
cdl_UE2_BS2.DelayProfile = 'CDL-D';
cdl_UE2_BS2.DelaySpread = 30e-9;
cdl_UE2_BS2.CarrierFrequency = Pars.fc;
cdl_UE2_BS2.MaximumDopplerShift = abs(UE2.vel_ms) / Pars.lambda;
cdl_UE2_BS2.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE2_BS2.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE2_BS2.SampleRate = scs * Nfft;

cdl_UE3_BS1 = nrCDLChannel;
cdl_UE3_BS1.DelayProfile = 'CDL-E';
cdl_UE3_BS1.DelaySpread = 30e-9;
cdl_UE3_BS1.CarrierFrequency = Pars.fc;
cdl_UE3_BS1.MaximumDopplerShift = abs(UE3.vel_ms) / Pars.lambda;
cdl_UE3_BS1.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE3_BS1.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE3_BS1.SampleRate = scs * Nfft;

cdl_UE3_BS2 = nrCDLChannel;
cdl_UE3_BS2.DelayProfile = 'CDL-E';
cdl_UE3_BS2.DelaySpread = 30e-9;
cdl_UE3_BS2.CarrierFrequency = Pars.fc;
cdl_UE3_BS2.MaximumDopplerShift = abs(UE3.vel_ms) / Pars.lambda;
cdl_UE3_BS2.TransmitAntennaArray.Size = [1 1 1 1 1];
cdl_UE3_BS2.ReceiveAntennaArray.Size = [antSize(1) antSize(2) 1 1 1];
cdl_UE3_BS2.SampleRate = scs * Nfft;


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

% Creo le griglie frequenziali usando la logica originale
TX_Grid1 = zeros(Nfft, nSymbols);
TX_Grid1(dataIndices, :) = matTx1 * Tx_Amplitude;   
TX_Grid1(pilotIndices, :) = pilotVal * Tx_Amplitude; 

TX_Grid2 = zeros(Nfft, nSymbols);
TX_Grid2(dataIndices, :) = matTx2 * Tx_Amplitude; 
TX_Grid2(pilotIndices, :) = (rand(numel(pilotIndices), nSymbols)+1j*rand(numel(pilotIndices), nSymbols)) * Tx_Amplitude;

TX_Grid3 = zeros(Nfft, nSymbols);
TX_Grid3(dataIndices, :) = matTx3 * Tx_Amplitude;
TX_Grid3(pilotIndices, :) = (rand(numel(pilotIndices), nSymbols)+1j*rand(numel(pilotIndices), nSymbols)) * Tx_Amplitude;

% Genero i segnali OFDM nel tempo usando ofdmmod
TX_TimeDomain1 = zeros(Ns, nSymbols);
TX_TimeDomain2 = zeros(Ns, nSymbols);
TX_TimeDomain3 = zeros(Ns, nSymbols);

for symIdx = 1:nSymbols
    TX_TimeDomain1(:, symIdx) = ofdmmod(TX_Grid1(:, symIdx), Nfft, CPLength);
    TX_TimeDomain2(:, symIdx) = ofdmmod(TX_Grid2(:, symIdx), Nfft, CPLength);
    TX_TimeDomain3(:, symIdx) = ofdmmod(TX_Grid3(:, symIdx), Nfft, CPLength);
end

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
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Scenario: Target (Blue) vs Interferers (Magenta)');
view(2); xlim([-30 90]); ylim([-20 50]); zlim([0 15]); legend([hUE1, hUE2, hUE3]);

ax2 = nexttile([1 1]);
hConst = plot(NaN, NaN, '.', 'Color', [1 0 0], 'MarkerSize', 4);
grid(ax2, 'on'); axis(ax2, 'square'); xlim([-2 2]); ylim([-2 2]);
title('RX Constellation BS1');
xline(0); yline(0);

ax2b = nexttile([1 1]);
hConst2 = plot(NaN, NaN, '.', 'Color', [0 0.8 0.4], 'MarkerSize', 4);
grid(ax2b, 'on'); axis(ax2b, 'square'); xlim([-2 2]); ylim([-2 2]);
title('RX Constellation BS2');
xline(0); yline(0);

axInfo = nexttile([1 1]); axis(axInfo, 'off');
hText = text(0.1, 0.5, '', 'FontSize', 12, 'Interpreter', 'none');

ax3 = nexttile([1 1]);
hMSE1 = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hMSE2 = semilogy(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
grid(ax3, 'on'); title('RLS Cost Function (MSE)');
xlabel('Symbol'); ylabel('MSE'); ylim([1e-5 10]); legend;

ax4 = nexttile([1 1]);
hSINR1 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hSINR2 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
grid(ax4, 'on'); title('Output SINR (Post-Beamforming)');
xlabel('Symbol'); ylabel('dB'); ylim([0 60]); legend;

ax5 = nexttile([1 1]);
hEVM1 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'r', 'DisplayName','BS1'); hold on;
hEVM2 = plot(NaN, NaN, 'LineWidth', 1.5, 'Color', 'g', 'DisplayName','BS2');
grid(ax5, 'on'); title('EVM');
xlabel('Symbol'); ylim([0 1.5]); legend;

%% =======================
%  5) MAIN SIMULATION LOOP (With Element Pattern Fix)
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

% Info per il canale CDL
info_cdl_UE1_BS1 = info(cdl_UE1_BS1);
info_cdl_UE1_BS2 = info(cdl_UE1_BS2);
info_cdl_UE2_BS1 = info(cdl_UE2_BS1);
info_cdl_UE2_BS2 = info(cdl_UE2_BS2);
info_cdl_UE3_BS1 = info(cdl_UE3_BS1);
info_cdl_UE3_BS2 = info(cdl_UE3_BS2);

% Init metrics placeholders (used in plotting text)
sinr_db1 = NaN; sinr_db2 = NaN;
evm_curr1 = NaN; evm_curr2 = NaN;

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
    
    % --- B. CHANNEL GENERATION con nrCDLChannel (CDL-D: Urban LOS) ---
    
    % Generate transmit signals for this symbol
    txSig1 = TX_TimeDomain1(:, t);
    txSig2 = TX_TimeDomain2(:, t);
    txSig3 = TX_TimeDomain3(:, t);
    
    % === Channel UE1 (Target) BS1 ===
    % Pass TX signal through CDL channel filter
    rxSig1_BS1 = cdl_UE1_BS1(txSig1);
    % Extract frequency response per antenna via OFDM demodulation
    H1_BS1 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        % Reshape received signal to antenna-specific output
        rxSymbol = ofdmdemod(rxSig1_BS1(1:Ns, antIdx), Nfft, CPLength);
        H1_BS1(antIdx, :) = rxSymbol.' ./ (TX_Grid1(:, t).' + 1e-12);
    end
    
    % === Channel UE1 (Target) BS2 ===
    rxSig1_BS2 = cdl_UE1_BS2(txSig1);
    H1_BS2 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        rxSymbol = ofdmdemod(rxSig1_BS2(1:Ns, antIdx), Nfft, CPLength);
        H1_BS2(antIdx, :) = rxSymbol.' ./ (TX_Grid1(:, t).' + 1e-12);
    end
    
    % === Channel UE2 (Interferer) BS1 ===
    rxSig2_BS1 = cdl_UE2_BS1(txSig2);
    H2_BS1 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        rxSymbol = ofdmdemod(rxSig2_BS1(1:Ns, antIdx), Nfft, CPLength);
        H2_BS1(antIdx, :) = rxSymbol.' ./ (TX_Grid2(:, t).' + 1e-12);
    end
    
    % === Channel UE2 (Interferer) BS2 ===
    rxSig2_BS2 = cdl_UE2_BS2(txSig2);
    H2_BS2 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        rxSymbol = ofdmdemod(rxSig2_BS2(1:Ns, antIdx), Nfft, CPLength);
        H2_BS2(antIdx, :) = rxSymbol.' ./ (TX_Grid2(:, t).' + 1e-12);
    end
    
    % === Channel UE3 (Interferer) BS1 ===
    rxSig3_BS1 = cdl_UE3_BS1(txSig3);
    H3_BS1 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        rxSymbol = ofdmdemod(rxSig3_BS1(1:Ns, antIdx), Nfft, CPLength);
        H3_BS1(antIdx, :) = rxSymbol.' ./ (TX_Grid3(:, t).' + 1e-12);
    end
    
    % === Channel UE3 (Interferer) BS2 ===
    rxSig3_BS2 = cdl_UE3_BS2(txSig3);
    H3_BS2 = zeros(Nant, Nfft);
    for antIdx = 1:Nant
        rxSymbol = ofdmdemod(rxSig3_BS2(1:Ns, antIdx), Nfft, CPLength);
        H3_BS2(antIdx, :) = rxSymbol.' ./ (TX_Grid3(:, t).' + 1e-12);
    end


    
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
    % --- D. RLS BEAMFORMING BS1 ---
    y_p1 = zeros(length(pilotIndices),1);
    for i=1:length(pilotIndices)
        k = pilotIndices(i);
        y_p1(i) = W1(:,k)' * RX_BS1(:,k);
    end
    cpe1 = mean(angle(y_p1 .* conj(pilotVal))); 
    if isnan(cpe1), cpe1=0; end
    W1 = W1 * exp(1j*cpe1);

    mse_s1 = 0;
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
    mse_hist(end+1) = mse_s1;

    w_p1 = W1(:, pilotIndices).';
    mag_i1 = interp1(pilotIndices, abs(w_p1), 1:Nfft, 'linear', 'extrap');
    phs_i1 = interp1(pilotIndices, unwrap(angle(w_p1)), 1:Nfft, 'linear', 'extrap');
    W1_int = (mag_i1 .* exp(1j*phs_i1)).';

    % --- D. RLS BEAMFORMING BS2 ---
    y_p2 = zeros(length(pilotIndices),1);
    for i=1:length(pilotIndices)
        k = pilotIndices(i);
        y_p2(i) = W2(:,k)' * RX_BS2(:,k);
    end
    cpe2 = mean(angle(y_p2 .* conj(pilotVal))); 
    if isnan(cpe2), cpe2=0; end
    W2 = W2 * exp(1j*cpe2);

    mse_s2 = 0;
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
    mse_hist2(end+1) = mse_s2;

    w_p2 = W2(:, pilotIndices).';
    mag_i2 = interp1(pilotIndices, abs(w_p2), 1:Nfft, 'linear', 'extrap');
    phs_i2 = interp1(pilotIndices, unwrap(angle(w_p2)), 1:Nfft, 'linear', 'extrap');
    W2_int = (mag_i2 .* exp(1j*phs_i2)).';
    
    % --- E. OUTPUT & METRICS (BS1/BS2) ---
    % Beamformer outputs per subcarrier
    y_out1 = sum(conj(W1_int) .* RX_BS1, 1);
    y_out2 = sum(conj(W2_int) .* RX_BS2, 1);

    % Constellation buffers: accumulate data subcarriers (like B_1BS_3UE.m)
    const_buf = [const_buf; y_out1(dataIndices).'];
    if length(const_buf) > 600, const_buf = const_buf(end-600:end); end
    
    const_buf2 = [const_buf2; y_out2(dataIndices).'];
    if length(const_buf2) > 600, const_buf2 = const_buf2(end-600:end); end

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
    
    % --- G. PLOTTING ---
    if mod(t, 20) == 0
        % Update Map
        set(hUE1, 'XData', UE1.pos(1), 'YData', UE1.pos(2), 'ZData', UE1.pos(3));
        set(hUE2, 'XData', UE2.pos(1), 'YData', UE2.pos(2), 'ZData', UE2.pos(3));
        set(hUE3, 'XData', UE3.pos(1), 'YData', UE3.pos(2), 'ZData', UE3.pos(3));

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
        
            % Update Text (BS1/BS2)
            set(hText, 'String', sprintf('Sym: %d\nBS1 SINR: %.1fdB\nBS2 SINR: %.1fdB\nBS1 EVM: %.3f\nBS2 EVM: %.3f', ...
                t, sinr_db1, sinr_db2, evm_curr1, evm_curr2));
        drawnow limitrate;
    end
end