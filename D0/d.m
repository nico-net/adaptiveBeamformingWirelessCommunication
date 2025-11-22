close all;
clear;

%% UE STRUCT
%% ================================
%  UE OBJECT STRUCT TEMPLATE
% ================================

UE = struct( ...
    'id',               0, ...   % UE identifier (index)
    'active',           false, ...% whether UE is currently detected
    ...
    'DOA',              [0; 0], ...  % [azimuth; elevation] in degrees
    ...
    'SNR',              NaN, ...         % estimated SNR
    'power',            NaN, ...         % received power
    ...
    'weights',          [], ...          % MVDR or other BF weights
    'output',           [], ...          % BF output signal for this UE
    ...
    'state',            'idle');      % idle, tracking, lost, newly_detected

%% Parameters
Pars.fc = 1e9; %Hz
Pars.SNR = 5; %in dB
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsin = 600;
Pars.Ts = 1e-5; %sample rate (s)
Pars.Fsample = 1/Pars.Ts;
Pars.TsVect = 0:Pars.Ts:5/Pars.Fsin;
Pars.speed1 = 50; %km/h
Pars.numFrame = 5;
Pars.thresAngle = 5; %in degree

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

%% Receiving signal

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
ang1 = rad2deg( atan2( Geometry.V1PosStart(2) - Geometry.BSPos(2) , Geometry.V1PosStart(1) - Geometry.BSPos(1) ) );
ang2 = rad2deg( atan2( Geometry.V2PosStart(2) - Geometry.BSPos(2) , Geometry.V2PosStart(1) - Geometry.BSPos(1) ) );

angles = [ang1 ang2;   % riga 1 = azimuth
          0    0];   % riga 2 = elevation


% raccolta sull'array ULA
receivedW = collector([sigV1_at_BS, sigV2_at_BS], angles);

SNRlin = 10^(Pars.SNR/10);

% average rx power
Px = mean(abs(receivedW(:)).^2);

% noise power
sigma2 = Px / SNRlin;

% awgn
noise = sqrt(sigma2/2) * (randn(size(receivedW)) + 1j*randn(size(receivedW)));

% output with noise
chOut = receivedW + noise;

%% Estimating the number of UEs


R = (receivedW * receivedW') / size(receivedW,2);

lambda = sort(eig(R), 'descend');
lambda(lambda<0) = eps;

M = size(R,1);
N = size(receivedW,2);

mdl = zeros(M,1);

for k = 0:M-1
    gm = geomean(lambda(k+1:end));
    am = mean(lambda(k+1:end));

    mdl(k+1) = -N*(M-k)*log(gm/am) + 0.5*k*(2*M-k)*log(N);
end

[~, idx] = min(mdl);
numUsers = idx-1;

%% UEs vector
UE = repmat(UE, numUsers, 1); 
for k = 1:numUsers
    UE(k).id = k;
    UE(k).active = true;
    UE(k).state = 'newly_detected';
end


% DOA ESTIMATOR
musicEstimator = phased.MUSICEstimator( ...
    'SensorArray', Geometry.BSarray, ...
    'OperatingFrequency', Pars.fc, ...
    'NumSignalsSource','Property', ...
    'DOAOutputPort',true,...
    'NumSignals', numUsers);


for currentFrame = 1:Pars.numFrame
    % DOA estimation
    [~, doas] = musicEstimator(receivedW);

    % Update of the UEs
    for k=1:numUsers
            UE(k) = updateUE(UE(k), doas(:,k), currentFrame, Pars.thresAngle, ...
                Geometry, Pars, chOut);
    end

    %% New RX acquisition
    
    % to update with real position of UEs (silvio/pietro)
    sigV1_at_BS = freeSpace(waveform1, Geometry.V1PosStart, Geometry.BSPos, vel1,zeros(3,1)); 
    sigV2_at_BS = freeSpace(waveform2, Geometry.V2PosStart, Geometry.BSPos, zeros(3,1), zeros(3,1));
    
    ang1 = [rad2deg( atan2( Geometry.V1PosStart(2) - Geometry.BSPos(2) , Geometry.V1PosStart(1) - Geometry.BSPos(1) ) ), 0]';
    ang2 = [rad2deg( atan2( Geometry.V2PosStart(2) - Geometry.BSPos(2) , Geometry.V2PosStart(1) - Geometry.BSPos(1) ) ), 0]';

    receivedW = collectPlaneWave(Geometry.BSarray, ...
    [sigV1_at_BS, sigV2_at_BS], ...         % signals
    [ang1 , ang2], ... % [az; el]
    Pars.fc);

    noise = sqrt(sigma2/2) * (randn(size(receivedW)) + 1j*randn(size(receivedW)));

    % output with noise
    chOut = receivedW + noise;

end



function UE = updateUE(UE, doa, numFrame, thresAngle, Geometry, Pars, chOut)

    doaAzEl =  [doa, 0]';

    if norm(doaAzEl-UE.DOA) > thresAngle
        UE.DOA = doaAzEl;
    
        % update channel statistics
        UE.power = mean(abs(UE.output).^2);

        bfMulti = phased.MVDRBeamformer('SensorArray',Geometry.BSarray,...
            'PropagationSpeed',Pars.c,'OperatingFrequency',Pars.fc,...
            'Direction',[UE.DOA], ...
            'WeightsOutputPort',true);
        
        [yMulti, UE.weights] = bfMulti(chOut);
       
        % Apply beamforming
        UE.output = yMulti;

        UE.state = sprintf('updated %d-th UE at %d-th frame\n', UE.id, numFrame);
        disp(UE.state);
    end
end


