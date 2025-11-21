close all;
clear;

%% UE STRUCT
%% ================================
%  UE OBJECT STRUCT TEMPLATE
% ================================

UE = struct( ...
    'id',               [], ...   % UE identifier (index)
    'active',           false, ...% whether UE is currently detected
    ...
    'pos',              [NaN NaN 0], ... % [x y z] BS reference frame
    'vel',              [0 0 0], ...     % velocity vector
    ...
    'DOA',              [NaN; NaN], ...  % [azimuth; elevation] in degrees
    'DOA_prev',         [NaN; NaN], ...
    'DOA_variation',    0, ...           % |DOA - DOA_prev|
    ...
    'SNR',              NaN, ...         % estimated SNR
    'power',            NaN, ...         % received power
    ...
    'fD',               NaN, ...         % Doppler shift
    'Tc',               NaN, ...         % coherence time estimate
    'lastUpdate',       0, ...           % last weight update (in samples or time)
    'nextUpdate',       Inf, ...         % next scheduled update
    ...
    'weights',          [], ...          % MVDR or other BF weights
    'output',           [], ...          % BF output signal for this UE
    ...
    'state',            'idle', ...      % idle, tracking, lost, newly_detected
    'lifetime',         0 ...            % frames since first detection
);

%% Parameters
Pars.fc = 1e9; %Hz
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsin = 600;
Pars.Ts = 1e-5; %sample rate (s)
Pars.Fsample = 1/Pars.Ts;
Pars.TsVect = 0:Pars.Ts:5/Pars.Fsin;
Pars.speed1 = 50; %km/h
Pars.numFrame = 500;

waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect)';
waveform2 = sin(2*pi*Pars.Fsin*Pars.TsVect + pi/3)';

%% 2D Geometry (x,y only)
Geometry.BSPos     = [0, 0, 0]';  % BS at origin, 2D

Geometry.V1PosStart = [60, 30, 0]';
Geometry.V1PosEnd   = [70,  100, 0]';

Geometry.V2PosStart = [20, 50, 0]';
Geometry.V2PosEnd   = [10,  -50, 0]';

%% ULA
Geometry.BSarray = phased.ULA('NumElements', 8, ...
    'ElementSpacing', Pars.lambda/2);

figure;
viewArray(Geometry.BSarray ,'Title','ULA 2D View')
set(gca,'CameraViewAngle',4.4);

%% Estimation of the number of UEs

freeSpace = phased.FreeSpace('OperatingFrequency',Pars.fc,'SampleRate',Pars.Fsample);
collector = phased.Collector('Sensor',Geometry.BSarray, ...
                             'OperatingFrequency',Pars.fc);

dir1 = Geometry.V1PosEnd - Geometry.V1PosStart;
dir1 = dir1 / norm(dir1);

vel1 = Pars.speed1 * dir1;         % 3D: [vx vy vz]

% free space loss
sigV1_at_BS = freeSpace(waveform1, Geometry.V1PosStart, Geometry.BSPos, vel1,zeros(3,1));
sigV2_at_BS = freeSpace(waveform2, Geometry.V2PosStart, Geometry.BSPos, zeros(3,1), zeros(3,1));

% calcola DOA dei due veicoli
az1 = rad2deg( atan2( Geometry.V1PosStart(2) - Geometry.BSPos(2) , Geometry.V1PosStart(1) - Geometry.BSPos(1) ) );
az2 = rad2deg( atan2( Geometry.V2PosStart(2) - Geometry.BSPos(2) , Geometry.V2PosStart(1) - Geometry.BSPos(1) ) );

angles = [az1 az2;   % riga 1 = azimuth
          0    0];   % riga 2 = elevation


% raccolta sull'array ULA
recivedW = collector([sigV1_at_BS, sigV2_at_BS], angles);

R = (recivedW * recivedW') / size(recivedW,2);

lambda = sort(eig(R), 'descend');
lambda(lambda<0) = eps;

M = size(R,1);     % numero di antenne
N = size(recivedW,2); % numero di snapshot

mdl = zeros(M,1);

for k = 0:M-1
    gm = geomean(lambda(k+1:end));
    am = mean(lambda(k+1:end));

    % funzione MDL
    mdl(k+1) = -N*(M-k)*log(gm/am) + 0.5*k*(2*M-k)*log(N);
end

[~, idx] = min(mdl);
numUsers = idx-1;

UE = repmat(UE, numUsers, 1); % crea vettore di struct
for k = 1:numUsers
    UE(k).id = k;
    UE(k).active = true;
    UE(k).state = 'newly_detected';
end

%% DOA ESTIMATION

musicEstimator = phased.MUSICEstimator( ...
    'SensorArray', Geometry.BSarray, ...
    'OperatingFrequency', Pars.fc, ...
    'NumSignalsSource','Property', ...
    'DOAOutputPort',true,...
    'NumSignals', numUsers);

for currentFrame = 1:Pars.numFrame
    
    [spectrum, doas] = musicEstimator(recivedW);
    
    %% 2. Update UE vector
    for k = 1:numUsers

        UE(k).DOA_prev = UE(k).DOA;
        UE(k).DOA = doas(:,k);
    
        UE(k).DOA_variation = norm( UE(k).DOA - UE(k).DOA_prev );
    
        % update channel statistics
        UE(k).power = mean(abs(UE(k).output).^2);
    
        % estimate Doppler from correlation or velocity (if known)
        %UE(k).fD = estimateDoppler(UE(k),Pars);
    
        % update coherence time
        %UE(k).Tc = 0.423 / abs(UE(k).fD + eps);
        UE(k).Tc = 0.5;
    
        % schedule next weight update
        UE(k).nextUpdate = UE(k).lastUpdate + UE(k).Tc/4;
    
        % MVDR update if necessary
        if currentFrame >= UE(k).nextUpdate
            %UE(k).weights = computeMVDRWeights(recivedW, UE(k).DOA, Geometry.BSarray, Pars);
            UE(k).lastUpdate = currentFrame;
        end
    
        % Apply beamforming
        UE(k).output = UE(k).weights' * recivedW;
    
    end

end