% 1. Configurazione Base
cfg = nrDLCarrierConfig; % Crealo UNA sola volta
cfg.FrequencyRange = 'FR1'; 
cfg.ChannelBandwidth = 20; % Ridotto a 20 MHz per velocità

% 2. Configurazione Numerologia (30 kHz)
cfg.SCSCarriers{1}.SubcarrierSpacing = 30;
% [IMPORTANTE] Devi dire esplicitamente alla griglia di essere grande 51 RBs (20 MHz)
cfg.SCSCarriers{1}.NSizeGrid = 51; 

% Configurazione Bandwidth Part (BWP)
cfg.BandwidthParts{1}.SubcarrierSpacing = 30;
% [IMPORTANTE] Anche la BWP deve essere allargata a 51
cfg.BandwidthParts{1}.NSizeBWP = 51;
cfg.BandwidthParts{1}.NStartBWP = 0; % Aggiungi offset iniziale

% Configurazione SS Burst (Case C per 30 kHz FR1)
cfg.SSBurst.BlockPattern = 'Case C';
cfg.SSBurst.Enable = true;
cfg.SSBurst.TransmittedBlocks = [1 0 0 0]; % Vettore a 4 bit per FR1 (solo primo blocco attivo)

% 3. Configurazione Dati (PDSCH)
cfg.PDSCH{1}.Enable = true; 
cfg.PDSCH{1}.PRBSet = 0:50; % Adattato alla griglia da 51 RB
cfg.PDSCH{1}.SymbolAllocation = [0 14]; % Aggiungi allocazione simboli
cfg.PDSCH{1}.TargetCodeRate = 0.5; % Valore tipico per avere dati robusti
cfg.PDSCH{1}.Modulation = 'QPSK';  % O '16QAM', '64QAM', '256QAM'

% 4. Generazione Waveform
try
    [waveform, info] = nrWaveformGenerator(cfg);
    % Ottieni il sample rate corretto dalla struttura info
    % In alcune versioni, info è una struct semplice
    if isfield(info, 'SamplingRate')
        fs = info.SamplingRate;
    elseif isfield(info, 'ResourceGrids')
        fs = info.ResourceGrids(1).Info.SampleRate;
    else
        % Fallback: calcola manualmente
        fs = 30.72e6; % Sample rate standard per 30 kHz SCS
    end
    disp(['Waveform generata con successo. Sample Rate: ' num2str(fs/1e6) ' MHz']);
    disp(['Dimensioni waveform: ' num2str(size(waveform))]);
catch ME
    disp(['Errore nella generazione: ' ME.message]);
    disp('Stack:');
    disp(getReport(ME, 'extended'));
    return; % Invece di error, usa return per vedere l'errore completo
end

% ---------------------------------------------------------
% 5. VISUALIZZAZIONE
% ---------------------------------------------------------
figure('Name', 'Analisi Segnale 5G NR 20MHz', 'Position', [100 100 1000 600]);

% PLOT A: Spettrogramma
subplot(2,1,1);
spectrogram(waveform, kaiser(1024,5), 512, 1024, fs, 'centered', 'yaxis');
title('Spettrogramma (Tempo-Frequenza)');
clim([-130 -80]); % Regola i colori per vedere meglio il segnale

% PLOT B: Spettro di Potenza
subplot(2,1,2);
[pxx, f] = pwelch(waveform, kaiser(1024,5), [], [], fs, 'centered');
plot(f/1e6, 10*log10(pxx), 'LineWidth', 1.5); 
grid on;
title(['Spettro di Potenza (Occupazione attesa ~20 MHz)']);
xlabel('Frequenza (MHz)');
ylabel('PSD (dB/Hz)');
xlim([-15 15]); % Zoom su +/- 15 MHz per vedere bene i 20 MHz centrali