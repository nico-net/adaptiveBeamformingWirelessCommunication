close all;
clear;
clc;

%% ============================================================
%%  SCRIPT 3D - LMS con 3D RADIATION PATTERN
%% ============================================================

%% 1) System Parameters
Pars.fc = 1e9;
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsample = 100e3;
Pars.Ts = 1/Pars.Fsample;
Pars.SnapshotsPerFrame = 500;
Pars.TsVect = (0:Pars.SnapshotsPerFrame-1)*Pars.Ts;
Pars.TotalTime_s = 40;
Pars.PhysicsStep = 0.05;
Pars.numFrame = ceil(Pars.TotalTime_s / Pars.PhysicsStep);
dt = Pars.PhysicsStep;
Pars.Temp_ant = 293.15;
Pars.NoiseFactor = 5;

% Velocità
Pars.speed1_kmh = 30;
Pars.speed2_kmh = 30;
Pars.v1_ms = Pars.speed1_kmh*(1000/3600);
Pars.v2_ms = Pars.speed2_kmh*(1000/3600);

% Waveforms (Users)
waveform1 = exp(1j*(2*pi*(Pars.fc)*Pars.TsVect + pi/6)).';
waveform2 = exp(1j*(2*pi*(Pars.fc + 1e3)*Pars.TsVect + pi/3)).';

% AGGIUNTO: Array SINR
SINR_UE1 = zeros(Pars.numFrame, 1);
SINR_UE2 = zeros(Pars.numFrame, 1);
SINR_UE1_dB = zeros(Pars.numFrame, 1);
SINR_UE2_dB = zeros(Pars.numFrame, 1);

%% 2) Geometry + Array URA
N_Row = 16; 
N_Col = 16;
N_Elements = N_Row * N_Col;
xpolElement = phased.CrossedDipoleAntennaElement(...
    'FrequencyRange', [1e9 6e9]);
Geometry.BSarray = phased.URA('Size',[N_Row N_Col], ...
    'ElementSpacing', [Pars.lambda/2 Pars.lambda/2], 'Element',xpolElement);
% Geometry.BSarray = phased.URA('Size',[N_Row N_Col], ...
%     'ElementSpacing', [Pars.lambda/2 Pars.lambda/2]);

Geometry.BSPos = [0; 0; 25];    
Geometry.V1Pos = [40; 60; 1.5];   
Geometry.V2Pos = [60; -20; 1.5];  

dir1 = [0; -1; 0.3]; dir1 = dir1/norm(dir1);
dir2 = [1; 0.5; 0]; dir2 = dir2/norm(dir2);
Geometry.V1Vel = Pars.v1_ms * dir1;
Geometry.V2Vel = Pars.v2_ms * dir2;

steeringVec = phased.SteeringVector('SensorArray', Geometry.BSarray,...
    'PropagationSpeed', Pars.c);

%% 3) Pre-Calculate Steering Matrix for 3D Plotting (Optimization)
% Riducendo il passo (es. 3 o 4 gradi) si velocizza l'animazione
Pars.DegStep = 4; 
az_vec = -90:Pars.DegStep:90;
el_vec = -90:Pars.DegStep:90;
[AzGrid, ElGrid] = meshgrid(az_vec, el_vec);
ScanGrid = [AzGrid(:)'; ElGrid(:)']; % Matrix 2 x N_points

% Pre-calcoliamo i vettori sterzanti per tutta la griglia (pesante ma fatto una volta sola)
SV_Matrix = steeringVec(Pars.fc, ScanGrid);

%% 4) UE STRUCT
UE_template = struct('id',[],'active',false,'pos',[NaN;NaN;NaN],...
    'vel',[0;0;0],'DOA',[0;0],'DOA_prev',[0;0],'DOA_variation',0,...
    'SNR',NaN,'power',NaN,'lastUpdate',0,'nextUpdate',0,...
    'weights',ones(N_Elements,1),'output',[],'state','idle','lifetime',0);

maxUsers = 2;
UE = repmat(UE_template,maxUsers,1);
for k=1:maxUsers, UE(k).id=k; end

%% 5) Objects
freeSpace = phased.FreeSpace('OperatingFrequency',Pars.fc,...
    'SampleRate',Pars.Fsample,'TwoWayPropagation',false);
collector = phased.Collector('Sensor',Geometry.BSarray,...
    'OperatingFrequency',Pars.fc);

%% 6) Visualization LAYOUT 
fig = figure; set(fig,'WindowState','maximized');
sgtitle('3D Beamforming Dashboard: Scenario & 3D Pattern')

% --- Sottografico 1: Mappa Scenario ---
subplot(2,2,1);
plot3(Geometry.BSPos(1),Geometry.BSPos(2),Geometry.BSPos(3),...
    'k^','MarkerSize',12,'LineWidth',3,'MarkerFaceColor','y'); hold on;
hPlotV1 = plot3(Geometry.V1Pos(1),Geometry.V1Pos(2),Geometry.V1Pos(3),...
    'bo','MarkerFaceColor','b');
hPlotV2 = plot3(Geometry.V2Pos(1),Geometry.V2Pos(2),Geometry.V2Pos(3),...
    'rs','MarkerFaceColor','r');
% Linea terra-BS
plot3([Geometry.BSPos(1) Geometry.BSPos(1)], [Geometry.BSPos(2) Geometry.BSPos(2)], ...
      [0 Geometry.BSPos(3)], 'k--'); 
hTrail1 = animatedline('Color','b','LineStyle',':');
hTrail2 = animatedline('Color','r','LineStyle',':');
grid on; axis equal; view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scenario Reale'); axis([-50 300 -200 200 0 70]);

% --- Sottografico 2: SINR (NUOVO) ---
subplot(2,2,3);
hSINR_line = plot(NaN,NaN,'b-','LineWidth',2); hold on;
hSINR_line2 = plot(NaN,NaN,'r-','LineWidth',2);
grid on; xlabel('Time [s]'); ylabel('SINR [dB]');
title('SINR vs Time'); xlim([0 Pars.TotalTime_s]);

% --- Sottografico 3: Potenza ---
subplot(2,2,4);
hPow_line1 = plot(NaN,NaN,'b-','LineWidth',1.5); hold on;
hPow_line2 = plot(NaN,NaN,'r-','LineWidth',1.5);
grid on; xlabel('Time [s]'); ylabel('Rx Power (Linear)');
title('Potenza Ricevuta (LMS)'); xlim([0 Pars.TotalTime_s]);

% --- Sottografico 4: 3D RADIATION PATTERN (Grande a destra, 2 righe) ---
subplot(2,2,2);
% Inizializziamo una surface vuota
hSurf = surf(zeros(size(AzGrid)), zeros(size(AzGrid)), zeros(size(AzGrid)), ...
             zeros(size(AzGrid)));
shading interp; 
lightangle(-45,30); lighting gouraud; material shiny;
axis equal; grid on; view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Pattern Combinato 3D (UE1 + UE2)');
c = colorbar; c.Label.String = 'Gain [dB]'; clim([-40 5]);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]); 
hold on;
% Frecce direzione utenti nel plot 3D (proiezione unitaria)
hArr1 = quiver3(0,0,0,0,0,0,'b','LineWidth',3,'MaxHeadSize',0.5,'AutoScaleFactor',2);
hArr2 = quiver3(0,0,0,0,0,0,'r','LineWidth',3,'MaxHeadSize',0.5,'AutoScaleFactor',2);


%% 7) Storage for replay
StoredData.V1Pos = zeros(3,Pars.numFrame);
StoredData.V2Pos = zeros(3,Pars.numFrame);
StoredData.UE1_weights = zeros(N_Elements,Pars.numFrame);
StoredData.UE2_weights = zeros(N_Elements,Pars.numFrame);
StoredData.ang_matrix = zeros(2,2,Pars.numFrame);
StoredData.P_UE1 = zeros(1,Pars.numFrame);
StoredData.P_UE2 = zeros(1,Pars.numFrame);
StoredData.didUpdate = false(1,Pars.numFrame);
StoredData.validFrames = 0;

% Noise Power Computation
Pars.sigma2 = db2pow(-174 + 10*log10(Pars.Temp_ant/293.15) + ...
    10*log10(1e5)-30 + Pars.NoiseFactor);

%% 8) MAIN LOOP
for currentFrame = 1:Pars.numFrame
    
    %% Movement
    Geometry.V1Pos = Geometry.V1Pos + Geometry.V1Vel*dt;
    Geometry.V2Pos = Geometry.V2Pos + Geometry.V2Vel*dt;
    UE(1).pos = Geometry.V1Pos; UE(1).vel = Geometry.V1Vel;
    UE(2).pos = Geometry.V2Pos; UE(2).vel = Geometry.V2Vel;
    
    % AGGIUNTO: Attivazione UE
    UE(1).active = true;
    UE(2).active = true;
    
    %% Channel
    relPos1 = Geometry.V1Pos - Geometry.BSPos;
    relPos2 = Geometry.V2Pos - Geometry.BSPos;
    % Angles [Az; El]
    [az1, el1, ~] = cart2sph(relPos1(1), relPos1(2), relPos1(3));
    [az2, el2, ~] = cart2sph(relPos2(1), relPos2(2), relPos2(3));
    ang_matrix = [rad2deg(az1) rad2deg(az2); rad2deg(el1) rad2deg(el2)];
    
    sig1 = freeSpace(waveform1, Geometry.V1Pos, Geometry.BSPos, Geometry.V1Vel, [0;0;0]);
    sig2 = freeSpace(waveform2, Geometry.V2Pos, Geometry.BSPos, Geometry.V2Vel, [0;0;0]);
    rx_clean = collector([sig1, sig2], ang_matrix);
    
    %% Noise & LMS
    noise = sqrt(Pars.sigma2/2)*(randn(size(rx_clean)) + 1j*randn(size(rx_clean)));
    rx_signal = rx_clean + noise;
    
    for k=1:maxUsers
        UE(k).DOA = ang_matrix(:,k);
        UE(k).DOA_variation = norm(UE(k).DOA - UE(k).DOA_prev);
    end
    
    %% LMS beamforming update logic - UNIFORMATO A C0
    didUpdateNow = false;
    if UE(1).DOA_variation > 1  % MODIFICATO: era > 2 || mod(currentFrame, 20)
        didUpdateNow = true;
        
        [N,L] = size(rx_signal);
        for k=1:maxUsers
            UE(k).weights = zeros(L,1);  % Reset pesi
            UE(k).DOA_prev = UE(k).DOA;
        end

        for k=1:maxUsers
            d_desired = (k==1)*waveform1 + (k~=1)*waveform2;
            w = UE(k).weights;
            y_out = zeros(N,1);

            for n = 1:N
                x = rx_signal(n,:).';
                mu = trace(x*x');  % MODIFICATO: era mu_base / (real(x'*x) + 1e-6)
                y = w' * x;
                e = d_desired(n) - y;
                w = w + mu * x * conj(e);
                y_out(n) = y;
            end

            UE(k).weights = w;
            UE(k).output = y_out.';
            UE(k).power = mean(abs(y_out).^2);
        end
    end
    StoredData.didUpdate(currentFrame) = didUpdateNow;
    StoredData.P_UE1(currentFrame) = UE(1).power;
    StoredData.P_UE2(currentFrame) = UE(2).power;
    
    %% SINR Calculation - AGGIUNTO
    if UE(1).active && UE(2).active
        rx1 = collector(sig1, ang_matrix(:,1))';
        rx2 = collector(sig2, ang_matrix(:,2))';

        P_s1 = mean(abs(UE(1).weights'*rx1).^2);
        P_i1 = mean(abs(UE(1).weights'*rx2).^2);
        P_n1 = Pars.sigma2*(UE(1).weights'*UE(1).weights);
        SINR_UE1(currentFrame) = P_s1/(P_i1+P_n1);
        SINR_UE1_dB(currentFrame) = 10*log10(SINR_UE1(currentFrame));

        P_s2 = mean(abs(UE(2).weights'*rx2).^2);
        P_i2 = mean(abs(UE(2).weights'*rx1).^2);
        P_n2 = Pars.sigma2*(UE(2).weights'*UE(2).weights);
        SINR_UE2(currentFrame) = P_s2/(P_i2+P_n2);
        SINR_UE2_dB(currentFrame) = 10*log10(SINR_UE2(currentFrame));
    end
    
    %% Store Data
    StoredData.V1Pos(:,currentFrame) = Geometry.V1Pos;
    StoredData.V2Pos(:,currentFrame) = Geometry.V2Pos;
    StoredData.UE1_weights(:,currentFrame) = UE(1).weights;
    StoredData.UE2_weights(:,currentFrame) = UE(2).weights;
    StoredData.ang_matrix(:,:,currentFrame) = ang_matrix;
    StoredData.validFrames = currentFrame;
    
    %% Visualization Update (ogni 5 frame per velocità)
    if mod(currentFrame, 5) == 0
        time_axis = (1:currentFrame)*Pars.PhysicsStep;
        
        % 1. Geometry Update
        set(hPlotV1, 'XData', Geometry.V1Pos(1), 'YData', Geometry.V1Pos(2), 'ZData', Geometry.V1Pos(3));
        set(hPlotV2, 'XData', Geometry.V2Pos(1), 'YData', Geometry.V2Pos(2), 'ZData', Geometry.V2Pos(3));
        addpoints(hTrail1, Geometry.V1Pos(1), Geometry.V1Pos(2), Geometry.V1Pos(3));
        addpoints(hTrail2, Geometry.V2Pos(1), Geometry.V2Pos(2), Geometry.V2Pos(3));
        
        % 2. Power Lines
        set(hPow_line1, 'XData', time_axis, 'YData', StoredData.P_UE1(1:currentFrame));
        set(hPow_line2, 'XData', time_axis, 'YData', StoredData.P_UE2(1:currentFrame));
        
        % 2a. SINR Lines - AGGIUNTO
        set(hSINR_line, 'XData', time_axis, 'YData', SINR_UE1_dB(1:currentFrame));
        set(hSINR_line2, 'XData', time_axis, 'YData', SINR_UE2_dB(1:currentFrame));
        
        % 3. --- 3D PATTERN UPDATE ---
        % Calcolo risposta array sulla griglia sferica
        % Pattern combinato = somma delle potenze lineari (approssimazione incoerente)
        % Risposta UE1
        resp1 = abs(UE(1).weights' * SV_Matrix);
        % Risposta UE2
        resp2 = abs(UE(2).weights' * SV_Matrix);
        
        % Somma potenza lineare
        pat_lin_tot = (resp1.^2 + resp2.^2);
        % Conversione dB e normalizzazione
        pat_db = 10*log10(pat_lin_tot + eps);
        pat_db_norm = pat_db - max(pat_db); % Normalizza a 0 dB il picco
        
        % Taglio dinamico per visualizzazione (tutto sotto -40dB è "zero" raggio)
        dyn_range = 40;
        R = pat_db_norm + dyn_range;
        R(R<0) = 0;
        R = R / dyn_range; % Raggio normalizzato [0, 1]
        
        % Reshape per la mesh
        R_mesh = reshape(R, size(AzGrid));
        Pat_color = reshape(pat_db_norm, size(AzGrid));
        
        % Conversione Sferiche -> Cartesiane per SURF
        % Attenzione: AzGrid e ElGrid sono in gradi
        [Xm, Ym, Zm] = sph2cart(deg2rad(AzGrid), deg2rad(ElGrid), R_mesh);
        
        set(hSurf, 'XData', Xm, 'YData', Ym, 'ZData', Zm, 'CData', Pat_color);
        
        % Aggiorna frecce direzionali (puntano verso gli utenti sulla sfera unitaria)
        % DOA corrente
        u1_az = deg2rad(ang_matrix(1,1)); u1_el = deg2rad(ang_matrix(2,1));
        u2_az = deg2rad(ang_matrix(1,2)); u2_el = deg2rad(ang_matrix(2,2));
        [dx1, dy1, dz1] = sph2cart(u1_az, u1_el, 1.2); % 1.2 lunghezza freccia
        [dx2, dy2, dz2] = sph2cart(u2_az, u2_el, 1.2);
        
        set(hArr1, 'UData', dx1, 'VData', dy1, 'WData', dz1);
        set(hArr2, 'UData', dx2, 'VData', dy2, 'WData', dz2);
        
        drawnow limitrate;
    end
end

%% 9) Slider Replay
totalFrames = StoredData.validFrames;
% Trim Data
StoredData.V1Pos = StoredData.V1Pos(:, 1:totalFrames);
StoredData.V2Pos = StoredData.V2Pos(:, 1:totalFrames);
StoredData.UE1_weights = StoredData.UE1_weights(:, 1:totalFrames);
StoredData.UE2_weights = StoredData.UE2_weights(:, 1:totalFrames);
StoredData.ang_matrix = StoredData.ang_matrix(:, :, 1:totalFrames);
StoredData.P_UE1 = StoredData.P_UE1(1:totalFrames);
StoredData.P_UE2 = StoredData.P_UE2(1:totalFrames);
SINR_UE1_dB = SINR_UE1_dB(1:totalFrames);  % AGGIUNTO
SINR_UE2_dB = SINR_UE2_dB(1:totalFrames);  % AGGIUNTO

% Slider Setup
sliderPanel = uipanel(fig, 'Position', [0.05 0.01 0.9 0.05], 'BorderType', 'none');
timeSlider = uicontrol(sliderPanel, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.1 0.3 0.75 0.4], ...
    'Min', 1, 'Max', totalFrames, 'Value', totalFrames, ...
    'SliderStep', [1/(totalFrames-1), 10/(totalFrames-1)]);
timeLabel = uicontrol(sliderPanel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.86 0.2 0.12 0.6], ...
    'String', sprintf('t = %.2f s', totalFrames*Pars.PhysicsStep), ...
    'FontSize', 10, 'HorizontalAlignment', 'left');

% Payload for Callback
sData.totalFrames = totalFrames;
sData.StoredData = StoredData;
sData.Pars = Pars;
sData.SV_Matrix = SV_Matrix; % Passiamo la matrice pre-calcolata
sData.AzGrid = AzGrid; 
sData.ElGrid = ElGrid;
sData.hPlotV1 = hPlotV1; sData.hPlotV2 = hPlotV2;
sData.hTrail1 = hTrail1; sData.hTrail2 = hTrail2;
sData.hPow_line1 = hPow_line1; sData.hPow_line2 = hPow_line2;
sData.hSurf = hSurf;
sData.hArr1 = hArr1; sData.hArr2 = hArr2;
sData.timeLabel = timeLabel;
sData.SINR_UE1_dB = SINR_UE1_dB;  % AGGIUNTO
sData.SINR_UE2_dB = SINR_UE2_dB;  % AGGIUNTO
sData.hSINR_line = hSINR_line;    % AGGIUNTO
sData.hSINR_line2 = hSINR_line2;  % AGGIUNTO

timeSlider.UserData = sData;
timeSlider.Callback = @updatePlotsFinal3D;

fprintf('Simulazione Completa. Usa lo slider per rivedere il Beamforming 3D.\n');

%% Callback Function
function updatePlotsFinal3D(src, ~)
    d = src.UserData;
    f = round(src.Value);
    if f < 1, f = 1; end
    if f > d.totalFrames, f = d.totalFrames; end
    
    currT = f * d.Pars.PhysicsStep;
    set(d.timeLabel, 'String', sprintf('t = %.2f s', currT));
    
    % 1. Geo
    V1 = d.StoredData.V1Pos(:, f);
    V2 = d.StoredData.V2Pos(:, f);
    set(d.hPlotV1, 'XData', V1(1), 'YData', V1(2), 'ZData', V1(3));
    set(d.hPlotV2, 'XData', V2(1), 'YData', V2(2), 'ZData', V2(3));
    
    clearpoints(d.hTrail1); clearpoints(d.hTrail2);
    sK = max(1, f-100);
    for k=sK:f
        addpoints(d.hTrail1, d.StoredData.V1Pos(1,k), d.StoredData.V1Pos(2,k), d.StoredData.V1Pos(3,k));
        addpoints(d.hTrail2, d.StoredData.V2Pos(1,k), d.StoredData.V2Pos(2,k), d.StoredData.V2Pos(3,k));
    end
    
    % 2. Power & SINR Line Markers - MODIFICATO
    subplot(2,3,2); hold on;
    plot(currT, d.StoredData.P_UE1(f), 'bo', 'MarkerFaceColor', 'b');
    plot(currT, d.StoredData.P_UE2(f), 'ro', 'MarkerFaceColor', 'r');
    
    % 2a. SINR Markers - AGGIUNTO
    subplot(2,3,4); hold on;
    plot(currT, d.SINR_UE1_dB(f), 'bo', 'MarkerFaceColor', 'b');
    plot(currT, d.SINR_UE2_dB(f), 'ro', 'MarkerFaceColor', 'r');
    
    % 3. 3D Pattern
    w1 = d.StoredData.UE1_weights(:, f);
    w2 = d.StoredData.UE2_weights(:, f);
    
    % Calcolo Veloce (vettoriale)
    resp1 = abs(w1' * d.SV_Matrix);
    resp2 = abs(w2' * d.SV_Matrix);
    
    pat_lin_tot = (resp1.^2 + resp2.^2);
    pat_db = 10*log10(pat_lin_tot + eps);
    pat_db_norm = pat_db - max(pat_db); 
    
    dyn_range = 40;
    R = pat_db_norm + dyn_range;
    R(R<0) = 0;
    R = R / dyn_range; 
    
    R_mesh = reshape(R, size(d.AzGrid));
    Pat_color = reshape(pat_db_norm, size(d.AzGrid));
    
    [Xm, Ym, Zm] = sph2cart(deg2rad(d.AzGrid), deg2rad(d.ElGrid), R_mesh);
    
    set(d.hSurf, 'XData', Xm, 'YData', Ym, 'ZData', Zm, 'CData', Pat_color);
    
    % Update Frecce
    ang = d.StoredData.ang_matrix(:,:,f);
    u1_az = deg2rad(ang(1,1)); u1_el = deg2rad(ang(2,1));
    u2_az = deg2rad(ang(1,2)); u2_el = deg2rad(ang(2,2));
    [dx1, dy1, dz1] = sph2cart(u1_az, u1_el, 1.2);
    [dx2, dy2, dz2] = sph2cart(u2_az, u2_el, 1.2);
    set(d.hArr1, 'UData', dx1, 'VData', dy1, 'WData', dz1);
    set(d.hArr2, 'UData', dx2, 'VData', dy2, 'WData', dz2);
    
    drawnow;
end