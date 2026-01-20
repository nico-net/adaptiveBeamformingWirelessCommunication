clear all; close all; clc;

% --- CONFIGURAZIONE ---
v = 300.0;                   % Velocità UE in km/h (Molto alta!)
fc = 4e9;                    % Frequenza portante in Hz
c = physconst('lightspeed'); 
fd = (v*1000/3600)/c*fc;     % Frequenza Doppler massima (~1111 Hz)

cdl = nrCDLChannel;
cdl.DelayProfile = 'CDL-D';
cdl.DelaySpread = 30e-9;
cdl.CarrierFrequency = fc;
cdl.MaximumDopplerShift = fd;

% Configurazione antenne
cdl.TransmitAntennaArray.Size = [1 1 2 1 1]; 
cdl.ReceiveAntennaArray.Size = [2 2 2 1 1];

% --- GENERAZIONE WAVEFORM (BANDA BASE) ---
% CORREZIONE QUI: Usiamo un Sample Rate standard LTE/5G
SR = 30.72e6;                % 30.72 MHz (Tipico per 20MHz bandwidth)

% Aumentiamo la durata per vedere bene le fluttuazioni veloci
numSubframes = 5;            % 5 ms totali
T_duration = numSubframes * 1e-3; 

cdl.SampleRate = SR;
cdlinfo = info(cdl);
Nt = cdlinfo.NumInputSignals;

% Vettore tempo
t = (0:round(T_duration*SR)-1).' / SR;

% 1. Creiamo un Tono Puro a 500 kHz
f_tone = 500e3; 
baseSignal = exp(1j * 2 * pi * f_tone * t);

% 2. Replichiamo il segnale su tutte le antenne Tx
txWaveform = repmat(baseSignal, 1, Nt);

% --- PASSAGGIO NEL CANALE ---
reset(cdl);
rxWaveform = cdl(txWaveform);

% --- VISUALIZZAZIONE ---
figure('Name', 'Analisi Doppler 300km/h', 'Position', [100, 100, 1000, 800]);

% Grafico 1: MAGNITUDO (Inviluppo)
subplot(3,1,1);
plot(t*1e3, abs(txWaveform(:,1)), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Tx (Ampiezza costante)');
hold on;
plot(t*1e3, abs(rxWaveform(:,1)), 'r', 'DisplayName', 'Rx (Fast Fading)');
title(['1. Magnitudo - Velocità: ' num2str(v) ' km/h (Doppler: ~' num2str(round(fd)) ' Hz)']);
ylabel('Ampiezza');
xlabel('Tempo (ms)');
legend; grid on;

% Grafico 2: PARTE REALE (Zoom)
subplot(3,1,2);
zoom_samples = 200; % Zoom estremo per vedere lo sfasamento
idx_zoom = 1:zoom_samples;
t_zoom_us = t(idx_zoom)*1e6;

plot(t_zoom_us, real(txWaveform(idx_zoom,1)), 'b--', 'DisplayName', 'Tx Real');
hold on;
plot(t_zoom_us, real(rxWaveform(idx_zoom,1)), 'r', 'DisplayName', 'Rx Real');
title('2. Parte Reale (Zoom: sfasamento e distorsione)');
ylabel('Ampiezza Reale');
xlabel('Tempo (\mus)');
legend; grid on;

% Grafico 3: SPETTRO (Effetto Doppler)
subplot(3,1,3);
% Usiamo una finestra più lunga per maggiore risoluzione in frequenza
nfft = 4096; 
[pxx_tx, f] = pwelch(txWaveform(:,1), hanning(nfft), [], nfft, SR, 'centered');
[pxx_rx, ~] = pwelch(rxWaveform(:,1), hanning(nfft), [], nfft, SR, 'centered');

plot(f/1e6, 10*log10(pxx_tx), 'b--', 'DisplayName', 'Tx Tone');
hold on;
plot(f/1e6, 10*log10(pxx_rx), 'r', 'DisplayName', 'Rx Tone (Doppler Spread)');
title('3. Spettro di Potenza (Doppler Spread)');
ylabel('PSD (dB/Hz)');
xlabel('Frequenza (MHz)');
xlim([0.4 0.6]); % Zoom attorno a 500 kHz (0.5 MHz)
legend; grid on;