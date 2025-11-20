close all;
clear all;
clc;

Pars.fc = 1e9; % Carrier frequency
Pars.c = physconst('LightSpeed');
Pars.lambda = Pars.c / Pars.fc;
Pars.Fsin = 600;
Pars.Ts = 1e-5;
Pars.Fsample = 1/Pars.Ts;
Pars.TsVect = 0:Pars.Ts:5/Pars.Fsin;

waveform1 = sin(2*pi*Pars.Fsin*Pars.TsVect);
waveform2 = waveform1;

% Geometry
Geometry.BSPos = [0,0,25]; % 25m of height
Geometry.V1PosStart = [70, -100, 1.5];
Geometry.V1PosEnd = [70, 100, 1.5];
Geometry.V2PosStart = [200, -50, 1.5];
Geometry.V2PosEnd = [10, -50, 1.5];

% Distances
Geometry.T1 = sqrt(sum((Geometry.V1PosEnd(1,1:2) - Geometry.V1PosStart(1,1:2)).^2));
Geometry.DistV1Start =sqrt(sum((Geometry.V1PosStart(1,1:2) - Geometry.BSPos(1,1:2)).^2));
Geometry.T2 = sqrt(sum((Geometry.V2PosEnd(1,1:2) - Geometry.V2PosStart(1,1:2)).^2));
Geometry.DistV2Start =sqrt(sum((Geometry.V2PosStart(1,1:2) - Geometry.BSPos(1,1:2)).^2));



Geometry.AOAV1Start = atan2(Geometry.BSPos(1,2) - Geometry.V1PosStart(1,2), ...
    Geometry.BSPos(1,1) - Geometry.V1PosStart(1,1)*180/pi);
Geometry.ZOAV1Start = atan2(Geometry.DistV1Start, ...
    Geometry.BSPos(1,3) - Geometry.V1PosStart(1,3)*180/pi);
Geometry.DOAV1S = [Geometry.AOAV1Start Geometry.ZOAV1Start];
Geometry.AOAV2Start = atan2(Geometry.BSPos(1,2) - Geometry.V2PosStart(1,2), ...
    Geometry.BSPos(1,1) - Geometry.V2PosStart(1,1)*180/pi);
Geometry.ZOAV2Start = atan2(Geometry.DistV2Start, ...
    Geometry.BSPos(1,3) - Geometry.V2PosStart(1,3)*180/pi);
Geometry.DOAV2S = [Geometry.AOAV2Start Geometry.ZOAV2Start];


Geometry.BSarray = phased.URA('Size',[4 4], ...
    'ElementSpacing',[Pars.lambda/2 Pars.lambda/2], ...
    'ArrayNormal','x');
Geometry.BSAntennaPos = getElementPosition(Geometry.BSarray);

recivedW = collectPlaneWave(Geometry.BSarray,[waveform1 waveform2], ...
    [Geometry.DOAV1S' Geometry.DOAV2S'], Pars.fc);

Pars.SNR = 20;

chOut = awgn(recivedW, Pars.SNR,'measured');
