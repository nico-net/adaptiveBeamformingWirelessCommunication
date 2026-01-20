clear; clc; close all;

% 1. Configurazione della Portante
cfg = nrDLCarrierConfig;
cfg.FrequencyRange = 'FR1'; 
cfg.ChannelBandwidth = 100; % MHz

% 2. Configurazione della Griglia (Carrier) a 30 kHz
cfg.SCSCarriers{1}.SubcarrierSpacing = 30; 
cfg.SCSCarriers{1}.NSizeGrid = 273; % Max RBs per 100MHz @ 30kHz

% 3. Configurazione della Bandwidth Part (BWP)
cfg.BandwidthParts{1}.SubcarrierSpacing = 30;
cfg.BandwidthParts{1}.NSizeBWP = 273;
cfg.BandwidthParts{1}.NStartBWP = 0;

% 4. [FIX DELL'ERRORE] Allineamento SS Burst a 30 kHz
cfg.SSBurst.Enable = true;
cfg.SSBurst.SubcarrierSpacing = 30; % Deve combaciare con SCSCarriers
cfg.SSBurst.Case = 'Case C';        % Case C è lo standard per 30kHz in FR1
cfg.SSBurst.BlockPattern = 'Case C'; % Ridondanza necessaria in alcune versioni
cfg.SSBurst.NCRB_SSB = 0;           % Offset in frequenza (0 va bene per iniziare)

% 5. Configurazione Dati (PDSCH)
cfg.PDSCH{1}.Enable = true;
cfg.PDSCH{1}.PRBSet = 0:272;        % Occupa tutta la banda
cfg.PDSCH{1}.Modulation = '16QAM';
cfg.PDSCH{1}.TargetCodeRate = 0.5;

% 6. Generazione Waveform
try
    [waveform, info] = nrWaveformGenerator(cfg);
    disp('Waveform generata con successo!');
    disp(['Sample Rate: ' num2str(info.SampleRate/1e6) ' Msps']);
catch ME
    error(['Errore ancora presente: ' ME.message]);
end

% ---------------------------------------------------------
% 7. VISUALIZZAZIONE
% ---------------------------------------------------------
fs = info.SampleRate;

figure('Name', 'Analisi Segnale 5G NR (Corretto)', 'Position', [100 100 1000 600]);

% PLOT A: Spettrogramma
subplot(2,1,1);
spectrogram(waveform, ones(4096,1), 0, 4096, 'centered', fs, 'yaxis', 'MinThreshold', -130);
title('Spettrogramma (Tempo-Frequenza) - 30kHz SCS');
% Nota: vedrai una riga gialla persistente, quello è l'SS Burst!

% PLOT B: Spettro
subplot(2,1,2);
[pxx, f] = pwelch(waveform, [], [], [], fs, 'centered');
plot(f/1e6, 10*log10(pxx)); 
grid on;
title('Spettro di Potenza');
xlabel('Frequenza (MHz)');
ylabel('PSD (dB/Hz)');
xlim([-80 80]);