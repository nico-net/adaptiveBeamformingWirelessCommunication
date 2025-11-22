close all;
clear all;
clc;

%% Parameters
Pars.fc = 1e9; 
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsin = 600;
Pars.Ts = 1e-5;
Pars.Fsample = 1/Pars.Ts;
Pars.TsVect = 0:Pars.Ts:5/Pars.Fsin;

waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect);
waveform2 = waveform1;

%% 2D Geometry (x,y only)
Geometry.BSPos     = [0, 0];  % BS at origin, 2D

Geometry.V1PosStart = [0, -100];
Geometry.V1PosEnd   = [70,  100];

Geometry.V2PosStart = [200, 0];
Geometry.V2PosEnd   = [10,  -50];

%% Distances (2D)
Geometry.T1 = norm(Geometry.V1PosEnd - Geometry.V1PosStart);
Geometry.DistV1Start = norm(Geometry.V1PosStart - Geometry.BSPos);

Geometry.T2 = norm(Geometry.V2PosEnd - Geometry.V2PosStart);
Geometry.DistV2Start = norm(Geometry.V2PosStart - Geometry.BSPos);

%% **Only Azimuth Angles in 2D**
Geometry.AOAV1Start = atan2(Geometry.BSPos(2) - Geometry.V1PosStart(2), ...
                            Geometry.BSPos(1) - Geometry.V1PosStart(1));

Geometry.AOAV2Start = atan2(Geometry.BSPos(2) - Geometry.V2PosStart(2), ...
                            Geometry.BSPos(1) - Geometry.V2PosStart(1));

% DOA vectors for phased array (azimuth-only)
Geometry.DOAV1S = [Geometry.AOAV1Start; 0];  % Zero elevation
Geometry.DOAV2S = [Geometry.AOAV2Start; 0];

%% FSPL
% Calculate the Free Space Path Loss (FSPL) 
Geometry.FSPLv1 = (4*pi*Geometry.DistV1Start/Pars.lambda)^2;
Geometry.FSPLv2 = (4*pi*Geometry.DistV2Start/Pars.lambda)^2;

waveform1Att = waveform1;
waveform2Att = waveform2;

%% BS Array
Geometry.BSarray = phased.ULA('NumElements', 8, ...
    'ElementSpacing', Pars.lambda/2);

figure;
viewArray(Geometry.BSarray ,'Title','ULA 2D View')
set(gca,'CameraViewAngle',4.4);

%% Received signal
recivedW = collectPlaneWave(Geometry.BSarray, ...
    [waveform1Att', waveform2Att'], ...         % signals
    [Geometry.DOAV1S Geometry.DOAV2S], ... % [az; el]
    Pars.fc);

Pars.SNR = 10;
Eg = sum(waveform1.^2)/numel(waveform1);
awgnNoise = sqrt(Eg/(2*Pars.SNR)) * (randn(size(recivedW))+ 1j*randn(size(recivedW)));
chOut = recivedW + awgnNoise;


%% MVDR beamforming
bfMulti = phased.MVDRBeamformer('SensorArray',Geometry.BSarray,...
    'PropagationSpeed',Pars.c,'OperatingFrequency',Pars.fc,...
    'Direction',[Geometry.DOAV1S Geometry.DOAV2S], ...
    'WeightsOutputPort',true);

[yMulti, w] = bfMulti(chOut);

tx1 = (waveform1').^2;
tx2 = (waveform2').^2;

rx1 = abs(yMulti(:,1)).^2;
rx2 = abs(yMulti(:,2)).^2;


%% PLOTS

figure;
subplot(2,1,1);
plot(tx1);
title('Transmitted Waveform 1');
xlabel('Sample');
ylabel('Amplitude');

subplot(2,1,2);
plot(rx1);
title('Beamformed Received Signal (Direction 1)');
xlabel('Sample');
ylabel('Amplitude');


figure;
subplot(2,1,1);
plot(tx2);
title('Transmitted Waveform 2');
xlabel('Sample');
ylabel('Amplitude');

subplot(2,1,2);
plot(rx2);
title('Beamformed Received Signal (Direction 2)');
xlabel('Sample');
ylabel('Amplitude');

%% Pattern beamforming
figure;
pattern(Geometry.BSarray,Pars.fc,[-180:180],0,'PropagationSpeed',Pars.c,...
    'Weights',w,'CoordinateSystem','rectangular',...
    'Type','powerdb');
figure;
pattern(Geometry.BSarray,Pars.fc,[-180:180],0,'PropagationSpeed',Pars.c ...
    ,'CoordinateSystem','rectangular',...
    'Type','powerdb');