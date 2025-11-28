clear all; close all; clc;

% --- CONFIGURAZIONE CANALE ---
v = 300.0;                   
fc = 4e9;                    
c = physconst('lightspeed'); 
fd = (v*1000/3600)/c*fc;     

cdl = nrCDLChannel;
cdl.DelayProfile = 'CDL-D';
cdl.DelaySpread = 30e-9;
cdl.CarrierFrequency = fc;
cdl.MaximumDopplerShift = fd;
cdl.TransmitAntennaArray.Size = [1 1 2 1 1]; 
cdl.ReceiveAntennaArray.Size = [1 1 2 1 1]; 

% --- GENERAZIONE WAVEFORM ---
SR = 30.72e6;                
numSubframes = 20;           
T_duration = numSubframes * 1e-3; 
cdl.SampleRate = SR;
Nt = info(cdl).NumInputSignals; 

t = (0:round(T_duration*SR)-1).' / SR;
f_tone = 500e3; 
baseSignal = exp(1j * 2 * pi * f_tone * t);
txWaveform = repmat(baseSignal, 1, Nt);

% --- PASSAGGIO NEL CANALE (FADING) ---
reset(cdl);
rxFaded = cdl(txWaveform); 

% --- AGGIUNTA MANUALE DEL RUMORE ---

% 1. Impostiamo l'SNR desiderato in dB
SNR_dB = 10; % Prova a cambiare questo valore (es. 0, 10, 20)

% 2. Calcoliamo la potenza media del segnale ricevuto (Signal Power)
% Calcoliamo la media su tutti i campioni e tutte le antenne per avere un riferimento stabile
P_signal = mean(abs(rxFaded(:)).^2);

% 3. Convertiamo SNR da dB a Lineare
SNR_linear = 10^(SNR_dB/10);

% 4. Calcoliamo la potenza del rumore necessaria (Noise Power)
P_noise = P_signal / SNR_linear;

% 5. Creiamo il rumore Gaussiano (Bianco)
% Nota: randn genera una distribuzione con deviazione standard = 1 (varianza = 1)
% Dobbiamo scalarlo. Poiché il rumore è complesso (Reale + Immaginario),
% la potenza si divide equamente: metà su I, metà su Q.
scaleFactor = sqrt(P_noise / 2);

% Generiamo matrice di numeri casuali della stessa dimensione del segnale
noise = scaleFactor * (randn(size(rxFaded)) + 1j * randn(size(rxFaded)));

% 6. Sommiamo Segnale + Rumore
rxNoisy = rxFaded + noise;

% --- VISUALIZZAZIONE ---
figure('Name', 'Rumore Calcolato a Mano', 'Position', [100, 100, 1000, 800]);

% Grafico 1: Segnale nel tempo
subplot(2,1,1);
plot(t*1e3, abs(rxNoisy(:,1)), 'Color', [0.6 0.6 0.6], 'DisplayName', 'Segnale + Rumore');
hold on;
plot(t*1e3, abs(rxFaded(:,1)), 'r', 'LineWidth', 2, 'DisplayName', 'Segnale Fading (Pulito)');
title(['Segnale con SNR = ' num2str(SNR_dB) ' dB']);
xlabel('Tempo (ms)');
ylabel('Ampiezza');
legend; grid on;

% Grafico 2: Costellazione (Piano Complesso)
% Questo mostra bene la "nuvola" di rumore attorno al segnale
subplot(2,1,2);
% Prendiamo solo un pezzo di segnale per non appesantire il plot
idx = 1:5000; 
plot(real(rxNoisy(idx,1)), imag(rxNoisy(idx,1)), '.', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Campioni Rumorosi');
hold on;
plot(real(rxFaded(idx,1)), imag(rxFaded(idx,1)), 'r', 'LineWidth', 1.5, 'DisplayName', 'Traiettoria Ideale');
title('Piano Complesso (Parte Reale vs Immaginaria)');
xlabel('Reale (I)');
ylabel('Immaginario (Q)');
axis equal; % Importante per vedere il cerchio correttamente
legend; grid on;