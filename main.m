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

Geometry.V1PosStart = [70, -100];
Geometry.V1PosEnd   = [70,  100];

Geometry.V2PosStart = [200, -50];
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

waveform1 = waveform1 / sqrt(Geometry.FSPLv1);
waveform2 = waveform2 / sqrt(Geometry.FSPLv1);

%% BS Array
Geometry.BSarray = phased.ULA('NumElements', 4, ...
    'ElementSpacing', Pars.lambda/2);

figure;
viewArray(Geometry.BSarray ,'Title','ULA 2D View')
set(gca,'CameraViewAngle',4.4);

%% Received signal
recivedW = collectPlaneWave(Geometry.BSarray, ...
    [waveform1', waveform2'], ...         % signals
    [Geometry.DOAV1S Geometry.DOAV2S], ... % [az; el]
    Pars.fc);

Pars.SNR = 10;
chOut = awgn(recivedW, Pars.SNR, 'measured');


