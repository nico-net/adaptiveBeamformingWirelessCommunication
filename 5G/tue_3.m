clear all; close all; clc;

% --- 1. CONFIGURAZIONE SISTEMA 5G ---
carrier = nrCarrierConfig;
carrier.SubcarrierSpacing = 30;       % 30 kHz
carrier.NSizeGrid = 52;               % Banda ~20 MHz
ofdmInfo = nrOFDMInfo(carrier);       

% Parametri Fisici
SR = ofdmInfo.SampleRate;             
fc = 4e9;                             
c = physconst('lightspeed');
lambda = c/fc;

% --- 2. SETUP VELOCITÀ E DISTANZA ---
% Impostiamo una velocità realistica (100 km/h) per un Doppler gestibile
v_kmh = 100; 
v_ms = v_kmh / 3.6; 
fd = (v_ms)/c*fc;

% Simulazione spaziale
numSlots = 50; 
dist_vector = linspace(50, 500, numSlots); 

% Configurazione Canale (Multipath CDL-D)
cdl = nrCDLChannel;
cdl.DelayProfile = 'CDL-D';           
cdl.DelaySpread = 30e-9;              
cdl.CarrierFrequency = fc;
cdl.MaximumDopplerShift = fd;
cdl.TransmitAntennaArray.Size = [4 1 1 1 1]; % MIMO 4x1 (+6dB array gain)
cdl.ReceiveAntennaArray.Size = [1 1 1 1 1];
cdl.SampleRate = SR;
cdl.ChannelFiltering = true;          
reset(cdl);

% Pre-allocazione
all_BER = zeros(numSlots, 1);
all_SNR_true = zeros(numSlots, 1);

% Accumulatore per la costellazione: [Reale, Immaginario, SNR_Locale]
scatter_data = []; 

disp('Avvio simulazione 5G Completa (OFDM + Fading + Visibilità)...');
rng(100); % Seed fisso per riproducibilità

% --- 3. LOOP DI ELABORAZIONE ---
for n = 1:numSlots
    
    % A. TRASMISSIONE
    txGrid = nrResourceGrid(carrier);
    txBits = randi([0 1], numel(txGrid)*4, 1); % 4 bit per simbolo (16-QAM)
    txSym = qammod(txBits, 16, 'InputType', 'bit', 'UnitAveragePower', true);
    txGrid(:) = txSym;
    
    [txWaveform, ~] = nrOFDMModulate(carrier, txGrid);
    
    % NORMALIZZAZIONE POTENZA (Cruciale per matchare la teoria)
    % Forziamo la deviazione standard a 1 (Potenza = 1 Watt)
    txWaveform = txWaveform / std(txWaveform); 
    
    % MIMO: Replica segnale su 4 antenne TX (beamforming uniforme)
    numTxAntennas = 4;
    txWaveform_mimo = repmat(txWaveform, 1, numTxAntennas) / sqrt(numTxAntennas);
    % Divisione per sqrt(4) = 2 mantiene potenza totale = 1W
    
    % Padding per filtri canale
    txWaveform_padded = [txWaveform_mimo; zeros(200, numTxAntennas)];
    
    % B. CANALE (Fading)
    [rxWaveform_faded, pathGains, sampleTimes] = cdl(txWaveform_padded);
    
    % C. PATH LOSS & RUMORE
    dist_now = dist_vector(n);
    
    % CORREZIONE 1: Path Loss 3GPP TR 38.901 Urban Macro (distanza in metri!)
    fc_GHz = fc/1e9;
    PL_dB = 28.0 + 22*log10(dist_now) + 20*log10(fc_GHz); % Formula corretta
    shadowFading_dB = randn * 8; % Log-normal shadowing (σ = 8 dB)
    PL_dB_total = PL_dB + shadowFading_dB;
    attenuation = 10^(-PL_dB_total/20);
    
    rxWaveform_received = rxWaveform_faded * attenuation;
    
    % CORREZIONE 2: Rumore Termico Realistico
    BW = carrier.NSizeGrid * 12 * carrier.SubcarrierSpacing * 1e3; % Hz (~18.7 MHz)
    
    % Rumore termico: N = kTB + NF
    % k = 1.38e-23 J/K, T = 290 K
    thermal_noise_W = 1.38e-23 * 290 * BW; % Potenza termica in Watt
    noiseFigure_linear = 10^(7/10); % NF = 7 dB → ~5x
    noisePower_W = thermal_noise_W * noiseFigure_linear;
    
    % Genera rumore AWGN complesso
    noise = sqrt(noisePower_W/2) * (randn(size(rxWaveform_received)) + 1j*randn(size(rxWaveform_received)));
    
    rxWaveform_total = rxWaveform_received + noise;
    
    % D. CALCOLO SNR REALE (considera potenza segnale effettiva dopo canale)
    P_signal = mean(abs(rxWaveform_received).^2);
    P_noise = mean(abs(noise).^2);
    current_snr = 10*log10(P_signal / P_noise);
    
    % DEBUG: Stampa valori alla prima iterazione
    if n == 1
        fprintf('DEBUG iterazione 1:\n');
        fprintf('  P_signal = %.4e W\n', P_signal);
        fprintf('  P_noise = %.4e W\n', P_noise);
        fprintf('  SNR = %.2f dB\n', current_snr);
        fprintf('  Path Loss = %.2f dB\n', PL_dB_total);
        fprintf('  Attenuation = %.4e\n', attenuation);
    end
    
    all_SNR_true(n) = current_snr;
    
    % E. RICEVITORE 5G
    pathFilters = getPathFilters(cdl); 
    offset = nrPerfectTimingEstimate(pathGains, pathFilters);
    
    rxWaveform_sync = rxWaveform_total(1+offset:end, :);
    rxGrid = nrOFDMDemodulate(carrier, rxWaveform_sync);
    
    H_est = nrPerfectChannelEstimate(carrier, pathGains, pathFilters, offset, sampleTimes);
    
    % Applica attenuazione path loss alla stima canale
    H_est = H_est * attenuation;
    
    % Safety Match Dimensioni
    [rows, cols, rxAnts, txAnts] = size(H_est);
    rxGrid = rxGrid(1:rows, 1:cols, :);
    H_est = H_est(1:rows, 1:cols, :, :);
    
    % Equalizzazione MISO con Zero-Forcing semplice
    % Poiché trasmetto lo stesso simbolo su 4 antenne, il canale effettivo è la somma
    rxSym_eq = zeros(rows, cols);
    
    for k = 1:rows
        for m = 1:cols
            % H: [4 x 1] guadagni dalle 4 antenne TX (include path loss)
            H_vec = squeeze(H_est(k, m, 1, :)); % [4 x 1]
            
            % Canale equivalente MISO: y = (H1 + H2 + H3 + H4) * x + n
            H_equivalent = sum(H_vec); % Scalare
            
            y_rx = rxGrid(k, m, 1); % Segnale ricevuto
            
            % Zero-Forcing: x_eq = y / H_eq
            rxSym_eq(k, m) = y_rx / H_equivalent;
        end
    end
    
    % DEBUG: Prima iterazione
    if n == 1
        fprintf('DEBUG Equalizzazione ZF:\n');
        fprintf('  H_equivalent = %.3e\n', H_equivalent);
        fprintf('  y_rx = %.3f + %.3fi\n', real(y_rx), imag(y_rx));
        fprintf('  rxSym_eq = %.3f + %.3fi\n', real(rxSym_eq(k,m)), imag(rxSym_eq(k,m)));
        fprintf('  |rxSym_eq| = %.3f\n', abs(rxSym_eq(k,m)));
    end
    
    % F. BER
    txSym_ref = txGrid(1:rows, 1:cols);
    txBits_ref = qamdemod(txSym_ref(:), 16, 'OutputType', 'bit', 'UnitAveragePower', true);
    rxBits = qamdemod(rxSym_eq(:), 16, 'OutputType', 'bit', 'UnitAveragePower', true);
    
    % Match dimensioni per biterr
    minLen = min(length(txBits_ref), length(rxBits));
    [~, ratio] = biterr(txBits_ref(1:minLen), rxBits(1:minLen));
    all_BER(n) = ratio;
    
    % G. RACCOLTA PUNTI COSTELLAZIONE
    % Prendiamo 150 punti casuali per ogni slot per popolare il grafico
    idx = randi(numel(rxSym_eq), 150, 1);
    pts = rxSym_eq(idx);
    % Salviamo: Reale, Immaginario, SNR (per colorare i punti)
    scatter_data = [scatter_data; real(pts), imag(pts), repmat(current_snr, 150, 1)];
    
end
disp('Simulazione completata.');

% --- VISUALIZZAZIONE "HIGH CONTRAST" ---
figure('Name', 'Analisi Link 5G Finale', 'Position', [100, 100, 1300, 500]);
% Impostiamo sfondo scuro per la finestra per simulare il tuo ambiente
set(gcf, 'Color', [0.15 0.15 0.15]); 

% 1. SNR
subplot(1, 3, 1);
plot(dist_vector, all_SNR_true, 'LineWidth', 3, 'Color', [0.2 0.6 1], 'DisplayName', 'Simulato');
hold on;
Ptx_dBm = 30; % 1 Watt = 30 dBm
BW_total = carrier.NSizeGrid * 12 * carrier.SubcarrierSpacing * 1e3;
thermal_noise_W = 1.38e-23 * 290 * BW_total;
noisePower_theory_dBm = 10*log10(thermal_noise_W * 1000) + 7; % kT*B + NF
PL_theory = 28.0 + 22*log10(dist_vector) + 20*log10(fc/1e9); % Formula corretta
arrayGain_dB = 6; % 4 antenne TX → +6 dB
snr_theory = Ptx_dBm - PL_theory - noisePower_theory_dBm + arrayGain_dB;
plot(dist_vector, snr_theory, 'r--', 'LineWidth', 2, 'DisplayName', 'Teorico');
title('SNR vs Distanza', 'Color', 'w'); 
xlabel('Distanza (m)', 'Color', 'w'); ylabel('dB', 'Color', 'w'); 
grid on; legend('Location', 'northeast', 'TextColor', 'k');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% 2. BER
subplot(1, 3, 2);
plot_BER = all_BER;
plot_BER(plot_BER == 0) = 1e-6; % Trucco grafico per mostrare gli zeri
semilogy(dist_vector, plot_BER, 'o-', 'Color', [0.2 1 0.2], 'LineWidth', 1.5, 'MarkerSize', 4);
yline(1e-6, 'w--', 'Zero Errori'); 
title('BER vs Distanza', 'Color', 'w'); 
xlabel('Distanza (m)', 'Color', 'w'); ylabel('BER', 'Color', 'w'); 
grid on; ylim([0.5e-6 1]);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% 3. Costellazione Colorata (FIX VISIBILITÀ)
subplot(1, 3, 3);
% Punti grandi (Size=20) e colorati in base all'SNR
scatter(scatter_data(:,1), scatter_data(:,2), 20, scatter_data(:,3), 'filled');
hold on;

% Riferimenti 16-QAM (Griglia 4x4)
ref_pts = qammod(0:15, 16, 'UnitAveragePower', true);
plot(real(ref_pts), imag(ref_pts), 'w+', 'LineWidth', 2, 'MarkerSize', 10); 
plot(real(ref_pts), imag(ref_pts), 'wo', 'LineWidth', 1.5, 'MarkerSize', 8); % Cerchi attorno

colormap('jet'); % Scala colori vivace (Blu=Basso SNR, Rosso=Alto SNR)
c = colorbar; 
c.Label.String = 'Qualità Segnale (SNR dB)';
c.Color = 'w'; % Testo colorbar bianco
clim([-5 20]); % Fissiamo i limiti per evitare punti neri

title('Costellazione Dinamica', 'Color', 'w'); 
axis square; grid on;
xlim([-2.5 2.5]); ylim([-2.5 2.5]);
xlabel('In-Phase', 'Color', 'w'); ylabel('Quadrature', 'Color', 'w');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

% --- STAMPA RISULTATI NEL TERMINALE ---
fprintf('\n========================================\n');
fprintf('   RISULTATI SIMULAZIONE 5G MIMO 4x1\n');
fprintf('========================================\n\n');

fprintf('Configurazione:\n');
fprintf('  - Frequenza: %.1f GHz\n', fc/1e9);
fprintf('  - Banda: %.1f MHz\n', BW_total/1e6);
fprintf('  - Modulazione: 16-QAM\n');
fprintf('  - Antenne TX: 4 (MIMO)\n');
fprintf('  - Canale: CDL-D Urban Macro\n');
fprintf('  - Velocità: %d km/h (Doppler: %.1f Hz)\n\n', v_kmh, fd);

fprintf('Risultati per Distanza:\n');
fprintf('%-12s %-12s %-12s\n', 'Distanza[m]', 'SNR[dB]', 'BER');
fprintf('----------------------------------------\n');
for i = 1:length(dist_vector)
    fprintf('%-12.0f %-12.2f %-12.4e\n', dist_vector(i), all_SNR_true(i), all_BER(i));
end

fprintf('\n========================================\n');
fprintf('Statistiche:\n');
fprintf('  - SNR medio: %.2f dB\n', mean(all_SNR_true));
fprintf('  - SNR min/max: %.2f / %.2f dB\n', min(all_SNR_true), max(all_SNR_true));
fprintf('  - BER medio: %.4e\n', mean(all_BER));
fprintf('  - Punti con BER=0: %d/%d (%.1f%%)\n', sum(all_BER==0), length(all_BER), 100*sum(all_BER==0)/length(all_BER));
fprintf('========================================\n\n');

disp('✅ Simulazione completata! Grafici visualizzati.');