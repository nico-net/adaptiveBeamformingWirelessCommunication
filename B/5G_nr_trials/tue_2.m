clear all; close all; clc;

% --- 1. PARAMETRI FISICI ---
fc = 4e9;                    
c = physconst('lightspeed');
lambda = c/fc;

% Simulazione di allontanamento (50m -> 500m)
distance_start = 50;         
distance_end = 500;          

SR = 30.72e6;                
numSubframes = 100;          % 100 ms totali
subframeDuration = 1e-3;     % Durata di un blocco (1 ms)
samplesPerSubframe = floor(subframeDuration * SR);

T_duration = numSubframes * subframeDuration;
totalSamples = samplesPerSubframe * numSubframes;

% Velocità fittizia per coprire la distanza nel tempo dato
dist_diff = distance_end - distance_start;
v_simulated = dist_diff / T_duration; 

% --- 2. CONFIGURAZIONE CANALE (Small Scale) ---
fd = (v_simulated)/c*fc;     

cdl = nrCDLChannel;
cdl.DelayProfile = 'CDL-D';
cdl.DelaySpread = 30e-9;
cdl.CarrierFrequency = fc;
cdl.MaximumDopplerShift = fd;
cdl.TransmitAntennaArray.Size = [1 1 1 1 1]; 
cdl.ReceiveAntennaArray.Size = [1 1 1 1 1]; 
cdl.SampleRate = SR;

% Pre-allochiamo i vettori per la velocità (evitiamo che la memoria cresca nel loop)
all_rxPower_dB = zeros(totalSamples, 1);
all_pathLoss_dB = zeros(totalSamples, 1);
all_dist = zeros(totalSamples, 1);

% Reset del canale prima di iniziare il loop
reset(cdl);

disp(['Inizio simulazione a blocchi... (' num2str(numSubframes) ' cicli)']);

% --- 3. LOOP DI ELABORAZIONE (STREAMING) ---
current_time = 0;

for n = 1:numSubframes
    
    % A. Generiamo il segnale per QUESTO subframe (1 ms)
    % Generiamo un tono unitario costante
    tx_chunk = (ones(samplesPerSubframe, 1) + 1j*ones(samplesPerSubframe, 1)) / sqrt(2);
    
    % B. Applichiamo Small Scale Fading (Il canale ricorda lo stato precedente!)
    rx_chunk_faded = cdl(tx_chunk);
    
    % C. Calcoliamo la distanza e il Path Loss per questo blocco
    % Creiamo un vettore tempo locale per questo chunk
    t_chunk = current_time + (0:samplesPerSubframe-1)'/SR;
    
    % Distanza istantanea: d = d0 + v*t
    dist_chunk = distance_start + v_simulated * t_chunk;
    
    % Calcolo Path Loss (Large Scale)
    PL_dB_chunk = fspl(dist_chunk, lambda);
    attenuation_factor = 10.^(-PL_dB_chunk / 20);
    
    % D. Applichiamo Path Loss
    rx_chunk_received = rx_chunk_faded .* attenuation_factor;
    
    % E. Aggiungiamo Rumore (Fisso)
    noiseFloor_dB = -100; 
    noisePower = 10^(noiseFloor_dB/10);
    noiseScale = sqrt(noisePower/2);
    noise = noiseScale * (randn(size(rx_chunk_received)) + 1j*randn(size(rx_chunk_received)));
    noise = 0;
    
    rx_chunk_total = rx_chunk_received + noise;
    
    % F. Salviamo i risultati nei vettori totali
    idx_start = (n-1)*samplesPerSubframe + 1;
    idx_end = n*samplesPerSubframe;
    
    % Salviamo solo la potenza (ampiezza^2) per risparmiare RAM nel plot finale
    % Non salviamo tutto il segnale complesso enorme se ci serve solo il plot di potenza
    all_rxPower_dB(idx_start:idx_end) = 10*log10(abs(rx_chunk_received).^2); 
    all_pathLoss_dB(idx_start:idx_end) = -PL_dB_chunk; % Salviamo il PL negativo per confronto
    all_dist(idx_start:idx_end) = dist_chunk;
    
    % Aggiorniamo il tempo globale
    current_time = current_time + subframeDuration;
end

disp('Simulazione completata.');

% --- VISUALIZZAZIONE ---
% Per il plot, sottocampioniamo i dati se sono troppi (es. prendiamo 1 campione ogni 100)
downsample_factor = 10;
idx_plot = 1:downsample_factor:totalSamples;

figure('Name', 'Streaming Simulation', 'Position', [100, 100, 1000, 600]);

plot(all_dist(idx_plot), all_rxPower_dB(idx_plot), 'b', 'DisplayName', 'Rx Signal (Fading + PL)');
hold on;
plot(all_dist(idx_plot), all_pathLoss_dB(idx_plot), 'r', 'LineWidth', 2, 'DisplayName', 'Teorico Path Loss');
yline(noiseFloor_dB, 'k--', 'Noise Floor');

title('Potenza Ricevuta vs Distanza (Elaborazione a Blocchi)');
xlabel('Distanza (m)');
ylabel('Potenza (dB)');
legend;
grid on;