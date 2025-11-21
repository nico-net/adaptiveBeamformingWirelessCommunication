close all;
clear;
clc;

%% 1. PARAMETRI DEL SISTEMA
Pars.fc = 1e9; % 1 GHz
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsin = 600; % Frequenza tono pilota
Pars.Fsample = 100e3; 
Pars.Ts = 1/Pars.Fsample;
Pars.SnapshotsPerFrame = 100; % Campioni usati per ogni stima
Pars.TsVect = (0:Pars.SnapshotsPerFrame-1)*Pars.Ts;
Pars.numFrame = 200; % Durata simulazione

% Cinematica Veicoli
Pars.speed1_kmh = 70; 
Pars.speed2_kmh = 50;
Pars.v1_ms = Pars.speed1_kmh * (1000/3600);
Pars.v2_ms = Pars.speed2_kmh * (1000/3600);

% Forme d'onda (Segnali pilota)
waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect).'; 
% Interferente con modulazione diversa (onda quadra approssimata)
waveform2 = sign(sin(2*pi*Pars.Fsin*Pars.TsVect + pi/3)).'; 

%% 2. GEOMETRIA E ARRAY
Geometry.BSPos = [0; 0; 0]; 

% Posizioni Iniziali
Geometry.V1Pos = [40; 20; 0]; % Utente desiderato
Geometry.V2Pos = [20; 50; 0]; % Interferente

% Direzioni di movimento (Vettori unitari)
dir1 = [1; 0.5; 0]; dir1 = dir1/norm(dir1); % Si allontana
dir2 = [0.5; -1; 0]; dir2 = dir2/norm(dir2); % Scende verso l'asse X

Geometry.V1Vel = Pars.v1_ms * dir1;
Geometry.V2Vel = Pars.v2_ms * dir2;

% Configurazione ULA (Uniform Linear Array)
N_Elements = 8; % Numero antenne
Geometry.BSarray = phased.ULA('NumElements', N_Elements, ...
    'ElementSpacing', Pars.lambda/2);

% Oggetto per calcolo Steering Vector (serve per MVDR)
steeringVec = phased.SteeringVector('SensorArray',Geometry.BSarray, ...
    'PropagationSpeed',Pars.c);

%% 3. INIZIALIZZAZIONE STRUTTURA UE
% Assumiamo 2 utenti noti all'inizio
UE_template = struct( ...
    'id', [], 'active', false, ...
    'DOA', [0;0], 'DOA_prev', [0;0], ...
    'weights', ones(N_Elements,1), ... % Pesi iniziali omni
    'output', zeros(1, Pars.SnapshotsPerFrame));

UEs = repmat(UE_template, 2, 1);
for k = 1:2
    UEs(k).id = k;
    UEs(k).active = true;
end
% Inizializziamo DOA fittizi per il tracking iniziale
[az1_init, ~] = rangeangle(Geometry.V1Pos, Geometry.BSPos);
[az2_init, ~] = rangeangle(Geometry.V2Pos, Geometry.BSPos);
UEs(1).DOA = [az1_init; 0];
UEs(2).DOA = [az2_init; 0];

%% 4. OGGETTI PHASED ARRAY
% Canale di propagazione
freeSpace = phased.FreeSpace('OperatingFrequency',Pars.fc, ...
    'SampleRate',Pars.Fsample, 'TwoWayPropagation', false);
collector = phased.Collector('Sensor',Geometry.BSarray, ...
    'OperatingFrequency',Pars.fc);

% Estimatore DOA (Root MUSIC è più stabile per ULA)
doaEstimator = phased.RootMUSICEstimator(...
    'SensorArray',Geometry.BSarray, ...
    'OperatingFrequency',Pars.fc, ...
    'NumSignalsSource','Property', ...
    'NumSignals',2); 

%% 5. SETUP VISUALIZZAZIONE
figure('Position',[100 100 1000 500]);

% Plot Sinistro: Geometria
subplot(1,2,1); 
hPlotGeo = plot(0,0,'k^','MarkerSize',10,'LineWidth',2,'DisplayName','BS'); hold on;
hPlotV1 = plot(0,0,'bo','MarkerFaceColor','b','DisplayName','UE 1 (Target)');
hPlotV2 = plot(0,0,'rs','MarkerFaceColor','r','DisplayName','UE 2 (Interferer)');
% Scie traiettorie
hTrail1 = animatedline('Color','b','LineStyle',':');
hTrail2 = animatedline('Color','r','LineStyle',':');
axis([-10 100 -20 80]); grid on; 
xlabel('X (m)'); ylabel('Y (m)');
title('Scenario V2X Dinamico'); legend('Location','northwest');

% Plot Destro: Diagramma di Radiazione
subplot(1,2,2); 
hPlotPat = polarplot(NaN, NaN, 'LineWidth', 2, 'Color', 'b'); 
hold on;
% Linee che indicano dove sono realmente i veicoli nel grafico polare
hLineV1 = polarplot([0 0], [0 -40], 'b--'); 
hLineV2 = polarplot([0 0], [0 -40], 'r--');
title('Beamforming MVDR (su UE 1)'); 
rlim([-40 0]); 

disp('Avvio Simulazione...');

%% 6. LOOP DI SIMULAZIONE
for iFrame = 1:Pars.numFrame
    
    % --- A. AGGIORNAMENTO FISICA (Moto) ---
    dt_sim = 0.1; % Passo temporale simulato
    Geometry.V1Pos = Geometry.V1Pos + Geometry.V1Vel * dt_sim;
    Geometry.V2Pos = Geometry.V2Pos + Geometry.V2Vel * dt_sim;
    
    % Calcolo angoli reali (Ground Truth per debug/grafica)
    [az1_true, ~] = rangeangle(Geometry.V1Pos, Geometry.BSPos);
    [az2_true, ~] = rangeangle(Geometry.V2Pos, Geometry.BSPos);
    
    % --- B. GENERAZIONE SEGNALI ---
    % Segnale che arriva alla BS
    sig1 = freeSpace(waveform1, Geometry.V1Pos, Geometry.BSPos, Geometry.V1Vel, [0;0;0]);
    sig2 = freeSpace(waveform2, Geometry.V2Pos, Geometry.BSPos, Geometry.V2Vel, [0;0;0]);
    
    % Raccolta sull'array
    ang_matrix = [az1_true, az2_true]; % Azimuth reali
    rx_clean = collector([sig1, sig2], ang_matrix);
    
    % Aggiunta rumore termico
    noise = 0.01 * (randn(size(rx_clean)) + 1i*randn(size(rx_clean)));
    rx_signal = rx_clean + noise;
    
    % --- C. STIMA DOA ---
    % MUSIC restituisce gli angoli ordinati per valore, non per utente
    estimated_doas = doaEstimator(rx_signal); 
    
    % --- D. TRACKING (Data Association) ---
    % Associa l'angolo stimato all'UE in base alla vicinanza col passo precedente
    est_used = [false, false]; % flag per non assegnare lo stesso angolo a due utenti
    
    for k = 1:2
        prev_az = UEs(k).DOA(1);
        
        % Cerca l'angolo non ancora usato più vicino a quello precedente
        min_dist = Inf;
        best_idx = -1;
        
        for j = 1:length(estimated_doas)
            dist = abs(estimated_doas(j) - prev_az);
            % Gestione salto -180/180
            if dist > 180, dist = 360 - dist; end
            
            if dist < min_dist && ~est_used(j)
                min_dist = dist;
                best_idx = j;
            end
        end
        
        if best_idx ~= -1
            UEs(k).DOA_prev = UEs(k).DOA;
            UEs(k).DOA = [estimated_doas(best_idx); 0];
            est_used(best_idx) = true;
        end
    end
    
    % --- E. BEAMFORMING MVDR ---
    % 1. Calcolo matrice covarianza spaziale Rxx
    Rxx = (rx_signal' * rx_signal) / Pars.SnapshotsPerFrame;
    % Diagonal loading (per stabilità numerica nell'inversione)
    Rxx = Rxx + 0.001 * eye(size(Rxx)); 
    R_inv = inv(Rxx);
    
    for k = 1:length(UEs)
        target_az = UEs(k).DOA(1);
        
        % Calcolo Steering Vector a(theta)
        sv = steeringVec(Pars.fc, [target_az; 0]);
        
        % Calcolo Pesi MVDR: w = (R^-1 * a) / (a' * R^-1 * a)
        num = R_inv * sv;
        den = sv' * R_inv * sv;
        w_mvdr = num / den;
        
        UEs(k).weights = w_mvdr;
        
        % Applicazione BF: y = w' * x
        UEs(k).output = w_mvdr' * rx_signal.'; 
    end
    
    % --- F. VISUALIZZAZIONE (Aggiornamento Dati) ---
    
    % 1. Aggiorna posizioni veicoli
    set(hPlotV1, 'XData', Geometry.V1Pos(1), 'YData', Geometry.V1Pos(2));
    set(hPlotV2, 'XData', Geometry.V2Pos(1), 'YData', Geometry.V2Pos(2));
    addpoints(hTrail1, Geometry.V1Pos(1), Geometry.V1Pos(2));
    addpoints(hTrail2, Geometry.V2Pos(1), Geometry.V2Pos(2));
    
    % 2. Calcolo manuale pattern (per velocità e stabilità)
    if mod(iFrame, 2) == 0
        % Definisci angoli di scansione
        scan_az = -90:0.5:90; % Scansioniamo da -90 a 90 gradi
        sv_scan = steeringVec(Pars.fc, [scan_az; zeros(1, length(scan_az))]);
        
        % Calcola risposta array per l'utente 1: P = |w' * a|^2
        pattern_response = abs(UEs(1).weights' * sv_scan).^2;
        pattern_dB = 10*log10(pattern_response + eps);
        
        % Normalizza a 0 dB il picco
        pattern_dB = pattern_dB - max(pattern_dB);
        
        % Aggiorna il grafico polare
        set(hPlotPat, 'ThetaData', deg2rad(scan_az), 'RData', pattern_dB);
        
        % Aggiorna le linee tratteggiate che mostrano dove sono i veicoli
        set(hLineV1, 'ThetaData', [deg2rad(az1_true) deg2rad(az1_true)], 'RData', [0 -40]);
        set(hLineV2, 'ThetaData', [deg2rad(az2_true) deg2rad(az2_true)], 'RData', [0 -40]);
        
        drawnow limitrate; 
    end
end

disp('Simulazione Completata.');