close all;
clear;
clc;

%% ============================================================
%%  SCRIPT (DOA note dalla geometria)
%%  - Vehicles in movement
%%  - UE struct extended
%%  - Per-frame estimation of number of UEs
%%  - DOA known from geometry
%%  - LMS beamforming
%%  - NEW: Plot sinr drop vs time since last update
%% ============================================================

%% 1) System Parameters
Pars.fc = 1e9;
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;

Pars.Fsin = 600;
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

Pars.speed1_kmh = 10;
Pars.speed2_kmh = 10;
Pars.v1_ms = Pars.speed1_kmh*(1000/3600);
Pars.v2_ms = Pars.speed2_kmh*(1000/3600);

waveform1 = exp(1j*(2*pi*(Pars.fc)*Pars.TsVect + pi/6)).';
waveform2 = exp(1j*(2*pi*(Pars.fc + 1e3)*Pars.TsVect + pi/3)).';

SINR_UE1 = zeros(Pars.numFrame, 1);
SINR_UE1_dB = zeros(Pars.numFrame, 1);
SINR_UE2 = zeros(Pars.numFrame, 1);
SINR_UE2_dB = zeros(Pars.numFrame, 1);

%% 2) Geometry + Array
N_Elements = 8;
Geometry.BSarray = phased.ULA('NumElements', N_Elements, ...
    'ElementSpacing', Pars.lambda/2);

Geometry.BSPos = [0;0;0];
Geometry.V2Pos = [40;-20;0];
Geometry.V1Pos = [20;60;0];

dir2 = [0;0.5;0]; 
dir1 = [0;-1;0]; 
dir1 = dir1/norm(dir1);
dir2 = dir2/norm(dir2);
Geometry.V1Vel = Pars.v1_ms * dir1;
Geometry.V2Vel = Pars.v2_ms * dir2;

steeringVec = phased.SteeringVector('SensorArray', Geometry.BSarray,...
    'PropagationSpeed', Pars.c);

%% 3) UE STRUCT
UE_template = struct('id',[],'active',false,'pos',[NaN;NaN;0],...
    'vel',[0;0;0],'DOA',[0;0],'DOA_prev',[0;0],'DOA_variation',0,...
    'SNR',NaN,'power',NaN,'lastUpdate',0,'nextUpdate',0,...
    'weights',ones(N_Elements,1),'output',[],'state','idle','lifetime',0);

maxUsers = 2;
UE = repmat(UE_template,maxUsers,1);
for k=1:maxUsers, UE(k).id=k; end

%% 4) Objects
freeSpace = phased.FreeSpace('OperatingFrequency',Pars.fc,...
    'SampleRate',Pars.Fsample,'TwoWayPropagation',false);

collector = phased.Collector('Sensor',Geometry.BSarray,...
    'OperatingFrequency',Pars.fc);

%% 5) Visualization LAYOUT
fig = figure; set(fig,'WindowState','maximized');
sgtitle('D1 - LMS, friis, DoA')

subplot(2,4,[1 2]);
plot(Geometry.BSPos(1),Geometry.BSPos(2),'w^','MarkerSize',10,'LineWidth',2); hold on;
hPlotV1 = plot(Geometry.V1Pos(1),Geometry.V1Pos(2),'bo','MarkerFaceColor','b');
hPlotV2 = plot(Geometry.V2Pos(1),Geometry.V2Pos(2),'rs','MarkerFaceColor','r');
hTrail1 = animatedline('Color','b','LineStyle',':');
hTrail2 = animatedline('Color','r','LineStyle',':');
axis([-40 80 -50 50]); grid on; title('Scenario V2X');

subplot(2,4,[3 4]);
hSINR_line = plot(NaN,NaN,'b-','LineWidth',2); hold on;
hSINR_line2 = plot(NaN,NaN,'r-','LineWidth',2);
grid on; xlabel('Time [s]'); ylabel('SINR [dB]');
title('SINR UE1 / UE2'); xlim([0 Pars.TotalTime_s]); ylim([-10 30]);

subplot(2,4,5);  % Beam UE1
hPlotPat1 = polarplot(NaN,NaN,'b','LineWidth',2); hold on;
hLineV1_p1 = polarplot([0 0],[0 -40],'b--'); title('Beam UE1'); rlim([-40 0]);

subplot(2,4,6);  % Beam UE2
hPlotPat2 = polarplot(NaN,NaN,'r','LineWidth',2); hold on;
hLineV2_p2 = polarplot([0 0],[0 -40],'r--'); title('Beam UE2'); rlim([-40 0]);

subplot(2,4,7);  % Combined
hPlotPatTotal = polarplot(NaN,NaN,'m','LineWidth',2); hold on;
hLineV1_total = polarplot([0 0],[0 -40],'b--');
hLineV2_total = polarplot([0 0],[0 -40],'r--'); 
title('Combined'); rlim([-40 0]);

%% === NUOVO SUBPLOT POTENZA (in basso a dx) ===
subplot(2,4,8);
hPow_line1 = plot(NaN,NaN,'b-','LineWidth',1.5); hold on;
hPow_line2 = plot(NaN,NaN,'r-','LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Power');
title('Received Power vs Time');
xlim([0 Pars.TotalTime_s]);

%% 6) Storage
StoredData.V1Pos = zeros(3,Pars.numFrame);
StoredData.V2Pos = zeros(3,Pars.numFrame);
StoredData.UE1_weights = zeros(N_Elements,Pars.numFrame);
StoredData.UE2_weights = zeros(N_Elements,Pars.numFrame);
StoredData.ang_matrix = zeros(2,2,Pars.numFrame);
StoredData.P_UE1 = zeros(1,Pars.numFrame);
StoredData.P_UE2 = zeros(1,Pars.numFrame);

StoredData.didUpdate = false(1,Pars.numFrame);
StoredData.tSinceUpd = NaN(1,Pars.numFrame);
StoredData.SINR_UE1_frozen_dB = NaN(1,Pars.numFrame);
StoredData.validFrames = 0;

%% Noise power
Pars.sigma2 = db2pow(-174 + 10*log10(Pars.Temp_ant/293.15) + ...
    10*log10(1e5)-30 + Pars.NoiseFactor);

%% 7) MAIN LOOP
scan_az = -90:0.5:90;
sv_scan = steeringVec(Pars.fc,[scan_az; zeros(1,numel(scan_az))]);

for currentFrame = 1:Pars.numFrame

    %% (a) Physics
    Geometry.V1Pos = Geometry.V1Pos + Geometry.V1Vel*dt;
    Geometry.V2Pos = Geometry.V2Pos + Geometry.V2Vel*dt;

    UE(1).pos = Geometry.V1Pos; UE(1).vel = Geometry.V1Vel;
    UE(2).pos = Geometry.V2Pos; UE(2).vel = Geometry.V2Vel;

    if Geometry.V1Pos(1) > 120
        break;
    end

    %% (b) Channel
    sig1 = freeSpace(waveform1,Geometry.V1Pos,Geometry.BSPos,Geometry.V1Vel,[0;0;0]);
    sig2 = freeSpace(waveform2,Geometry.V2Pos,Geometry.BSPos,Geometry.V2Vel,[0;0;0]);

    ang1 = rad2deg(atan2(Geometry.V1Pos(2),Geometry.V1Pos(1)));
    ang2 = rad2deg(atan2(Geometry.V2Pos(2),Geometry.V2Pos(1)));
    ang_matrix = [ang1 ang2; 0 0];

    rx_clean = collector([sig1,sig2],ang_matrix);

    %% (c) Noise
    noise = sqrt(Pars.sigma2/2)*(randn(size(rx_clean))+1j*randn(size(rx_clean)));
    rx_signal = rx_clean + noise;

    %% (d) MDL
    R = (rx_signal * rx_signal') / size(rx_signal, 2);
    eigvals = sort(real(eig(R)), 'descend');
    eigvals(eigvals < 0) = eps;

    M = size(R,1);
    Nsnap = size(rx_signal,2);
    mdl = zeros(M,1);

    for k = 0:M-1
        gm = geomean(eigvals(k+1:end));
        am = mean(eigvals(k+1:end));
        mdl(k+1) = -Nsnap*(M-k)*log(gm/am) + 0.5*k*(2*M-k)*log(Nsnap);
    end

    [~, idx] = min(mdl);
    numUsers_est = idx-1;
    numUsers_est = min(max(numUsers_est,0), maxUsers);

    %% (e) DOA known
    for k=1:maxUsers
        UE(k).active = true;
        UE(k).DOA = [ang_matrix(1,k); 0];
        UE(k).DOA_variation = norm(UE(k).DOA - UE(k).DOA_prev);
        UE(k).lifetime = UE(k).lifetime + 1;
    end

    %% (f) LMS beamforming update logic
    didUpdateNow = false;
    if UE(k).DOA_variation > 4
        didUpdateNow = true;
        UE(k).DOA_prev = UE(k).DOA;

        [N,L] = size(rx_signal);
        for k=1:maxUsers
            UE(k).weights = zeros(L,1);
        end

        for k=1:maxUsers
            d_desired = (k==1)*waveform1 + (k~=1)*waveform2;
            w = UE(k).weights;
            y_out = zeros(N,1);

            for n = 1:N
                x = rx_signal(n,:).';
                mu = trace(x*x');
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

    %% (g) SINR Adaptive
    if UE(1).active && UE(2).active
        rx1 = collector(sig1,ang_matrix(:,1))';
        rx2 = collector(sig2,ang_matrix(:,2))';

        P_s1 = mean(abs(UE(1).weights'*rx1).^2);
        P_i1 = mean(abs(UE(1).weights'*rx2).^2);
        P_n1 = Pars.sigma2*(UE(1).weights'*UE(1).weights);
        SINR_UE1(currentFrame) = P_s1/(P_i1+P_n1);
        SINR_UE1_dB(currentFrame)=10*log10(SINR_UE1(currentFrame));
    end

    if UE(1).active && UE(2).active
        rx1 = collector(sig1,ang_matrix(:,1))';
        rx2 = collector(sig2,ang_matrix(:,2))';

        P_s2 = mean(abs(UE(2).weights'*rx2).^2);
        P_i2 = mean(abs(UE(2).weights'*rx1).^2);
        P_n2 = Pars.sigma2*(UE(2).weights'*UE(2).weights);
        SINR_UE2(currentFrame)=P_s2/(P_i2+P_n2);
        SINR_UE2_dB(currentFrame)=10*log10(SINR_UE2(currentFrame));
    end

    %% (g) Store power
    StoredData.P_UE1(currentFrame)=UE(1).power;
    StoredData.P_UE2(currentFrame)=UE(2).power;

    %% === Time since last update (punto 3) ===
    lastUpd = find(StoredData.didUpdate(1:currentFrame),1,'last');
    if isempty(lastUpd)
        StoredData.tSinceUpd(currentFrame)=NaN;
    else
        StoredData.tSinceUpd(currentFrame)=(currentFrame-lastUpd)*Pars.PhysicsStep;
    end

    %% (h) Store
    StoredData.V1Pos(:,currentFrame)=Geometry.V1Pos;
    StoredData.V2Pos(:,currentFrame)=Geometry.V2Pos;
    StoredData.UE1_weights(:,currentFrame)=UE(1).weights;
    StoredData.UE2_weights(:,currentFrame)=UE(2).weights;
    StoredData.ang_matrix(:,:,currentFrame)=ang_matrix;
    StoredData.validFrames=currentFrame;

    %% (i) Visualization (non modifico)
    time_axis=(1:currentFrame)*Pars.PhysicsStep;

    set(hSINR_line,'XData',time_axis,'YData',SINR_UE1_dB(1:currentFrame));
    set(hSINR_line2,'XData',time_axis,'YData',SINR_UE2_dB(1:currentFrame));

    set(hPow_line1,'XData',time_axis,'YData',StoredData.P_UE1(1:currentFrame));
    set(hPow_line2,'XData',time_axis,'YData',StoredData.P_UE2(1:currentFrame));

    if mod(currentFrame,2)==0
        pat1 = abs(UE(1).weights'*sv_scan).^2;
        pat1_dB = 10*log10(pat1+eps); pat1_dB = pat1_dB-max(pat1_dB);
        set(hPlotPat1,'ThetaData',deg2rad(scan_az),'RData',pat1_dB);

        pat2 = abs(UE(2).weights'*sv_scan).^2;
        pat2_dB = 10*log10(pat2+eps); pat2_dB = pat2_dB-max(pat2_dB);
        set(hPlotPat2,'ThetaData',deg2rad(scan_az),'RData',pat2_dB);

        pat_tot = pat1+pat2;
        pat_tot_dB = 10*log10(pat_tot+eps); pat_tot_dB = pat_tot_dB-max(pat_tot_dB);
        set(hPlotPatTotal,'ThetaData',deg2rad(scan_az),'RData',pat_tot_dB);

        t1=ang_matrix(1,1); t2 = ang_matrix(1,2);
        set(hLineV1_p1,'ThetaData',[deg2rad(t1) deg2rad(t1)],'RData',[0 -40]);
        set(hLineV2_p2,'ThetaData',[deg2rad(t2) deg2rad(t2)],'RData',[0 -40]);
        set(hLineV1_total,'ThetaData',[deg2rad(t1) deg2rad(t1)],'RData',[0 -40]);
        set(hLineV2_total,'ThetaData',[deg2rad(t2) deg2rad(t2)],'RData',[0 -40]);

        drawnow limitrate;
    end
end

%% 8) Post-simulation: Add time slider for replay
totalFrames = StoredData.validFrames;
StoredData.V1Pos = StoredData.V1Pos(:, 1:totalFrames);
StoredData.V2Pos = StoredData.V2Pos(:, 1:totalFrames);
StoredData.UE1_weights = StoredData.UE1_weights(:, 1:totalFrames);
StoredData.UE2_weights = StoredData.UE2_weights(:, 1:totalFrames);
StoredData.ang_matrix = StoredData.ang_matrix(:, :, 1:totalFrames);
SINR_UE1_dB = SINR_UE1_dB(1:totalFrames);
SINR_UE2_dB = SINR_UE2_dB(1:totalFrames);

% Add slider panel
sliderPanel = uipanel(fig, 'Position', [0.05 0.01 0.9 0.05], 'BorderType', 'none');
timeSlider = uicontrol(sliderPanel, 'Style', 'slider', ...
    'Units', 'normalized', 'Position', [0.1 0.3 0.75 0.4], ...
    'Min', 1, 'Max', totalFrames, 'Value', totalFrames, ...
    'SliderStep', [1/(totalFrames-1), 10/(totalFrames-1)]);

timeLabel = uicontrol(sliderPanel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.86 0.2 0.12 0.6], ...
    'String', sprintf('t = %.2f s', totalFrames*Pars.PhysicsStep), ...
    'FontSize', 10, 'HorizontalAlignment', 'left');

% Prepare all data needed for callback
sliderData.totalFrames = totalFrames;
sliderData.StoredData = StoredData;
sliderData.Pars = Pars;
sliderData.SINR_UE1_dB = SINR_UE1_dB;
sliderData.SINR_UE2_dB = SINR_UE2_dB;
sliderData.sv_scan = sv_scan;
sliderData.scan_az = scan_az;
sliderData.hPlotV1 = hPlotV1;
sliderData.hPlotV2 = hPlotV2;
sliderData.hTrail1 = hTrail1;
sliderData.hTrail2 = hTrail2;
sliderData.timeLabel = timeLabel;
sliderData.hPlotPat1 = hPlotPat1;
sliderData.hPlotPat2 = hPlotPat2;
sliderData.hPlotPatTotal = hPlotPatTotal;
sliderData.hLineV1_p1 = hLineV1_p1;
sliderData.hLineV2_p2 = hLineV2_p2;
sliderData.hLineV1_total = hLineV1_total;
sliderData.hLineV2_total = hLineV2_total;

timeSlider.UserData = sliderData;
timeSlider.Callback = @updatePlots;

fprintf('Simulazione completata. Usa lo slider per ripercorrere la simulazione.\n');
%% Nested function for slider callback
function updatePlots(src, ~)
    data = src.UserData;
    frameIdx = round(src.Value);
    if frameIdx < 1, frameIdx = 1; end
    if frameIdx > data.totalFrames, frameIdx = data.totalFrames; end
    
    % Update time label
    currentTime = frameIdx * data.Pars.PhysicsStep;
    set(data.timeLabel, 'String', sprintf('t = %.2f s', currentTime));
    
    % Update vehicle positions
    V1Pos_replay = data.StoredData.V1Pos(:, frameIdx);
    V2Pos_replay = data.StoredData.V2Pos(:, frameIdx);
    set(data.hPlotV1, 'XData', V1Pos_replay(1), 'YData', V1Pos_replay(2));
    set(data.hPlotV2, 'XData', V2Pos_replay(1), 'YData', V2Pos_replay(2));
    
    % Update trails
    clearpoints(data.hTrail1);
    clearpoints(data.hTrail2);
    for k = 1:frameIdx
        addpoints(data.hTrail1, data.StoredData.V1Pos(1,k), data.StoredData.V1Pos(2,k));
        addpoints(data.hTrail2, data.StoredData.V2Pos(1,k), data.StoredData.V2Pos(2,k));
    end
    
    % Update SINR plot
    time_axis = (1:data.totalFrames) * data.Pars.PhysicsStep;
    subplot(2,4,[3 4]); cla; hold on;
    plot(time_axis, data.SINR_UE1_dB, 'b-', 'LineWidth', 1.5);
    plot(time_axis, data.SINR_UE2_dB, 'r-', 'LineWidth', 1.5);
    plot(currentTime, data.SINR_UE1_dB(frameIdx), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    plot(currentTime, data.SINR_UE2_dB(frameIdx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on;
    xlabel('Time [s]');
    ylabel('SINR [dB]');
    title('SINR of UE1 and UE2 vs Time');
    xlim([0 data.Pars.TotalTime_s]);
    ylim([-10 30]);
    
    % Update beam patterns
    w1 = data.StoredData.UE1_weights(:, frameIdx);
    w2 = data.StoredData.UE2_weights(:, frameIdx);
    ang_matrix_replay = data.StoredData.ang_matrix(:, :, frameIdx);
    
    % Pattern UE 1
    pat1 = abs(w1' * data.sv_scan).^2;
    pat1_dB = 10*log10(pat1 + eps);
    pat1_dB = pat1_dB - max(pat1_dB);
    set(data.hPlotPat1, 'ThetaData', deg2rad(data.scan_az), 'RData', pat1_dB);
    
    % Pattern UE 2
    pat2 = abs(w2' * data.sv_scan).^2;
    pat2_dB = 10*log10(pat2 + eps);
    pat2_dB = pat2_dB - max(pat2_dB);
    set(data.hPlotPat2, 'ThetaData', deg2rad(data.scan_az), 'RData', pat2_dB);
    
    % Combined Pattern
    pat_total = pat1 + pat2;
    pat_total_dB = 10*log10(pat_total + eps);
    pat_total_dB = pat_total_dB - max(pat_total_dB);
    set(data.hPlotPatTotal, 'ThetaData', deg2rad(data.scan_az), 'RData', pat_total_dB);
    
    % Update angle lines
    t_az1 = ang_matrix_replay(1,1);
    t_az2 = ang_matrix_replay(1,2);
    set(data.hLineV1_p1, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
    set(data.hLineV2_p2, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);
    set(data.hLineV1_total, 'ThetaData', [deg2rad(t_az1) deg2rad(t_az1)], 'RData', [0 -40]);
    set(data.hLineV2_total, 'ThetaData', [deg2rad(t_az2) deg2rad(t_az2)], 'RData', [0 -40]);
    
    drawnow;
end

