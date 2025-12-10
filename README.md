# adaptiveBeamformingWirelessCommunication
note: the written files are the correct final versions of the work in each level.

## D0: 2D, 1 BS, LOS (FRIIS), Narrow, DoA (20/11/25-22/11-25)
```LEVEL_D0_final.m```
## D1: 2D, 1 BS, LOS (FRIIS), Narrow, LMS (22/11/25-24/11-25)
```LEVEL_D1_plot.m``` 
## C0: 3D, 1 BS, LOS (FRIIS), Narrow, LMS (23/11/25-24/11-25)
```C0.m``` or ```C0_1_3D_Pattern.m```
## C: 3D, 1 BS, LOS (FRIIS), OFDM, LMS (wideband) (24/11/25-30/11/25)
```C_ ``` DA SCEGLIERE
## B: 2D, 1 BS, Multipath simplified, OFDM, LMS (wideband) (01/12/25-10/12/25)
```B_noMUMIMO.m``` and ```B_noMUMIMO_channel.m```
## A: 3D, 2 BS, 3GPP, OFDM, LMS (wideband) (11/12/25-)

5 cases of study:
### 1) Simple tracking case in different 3GPP channels
```A_simple.m```

Simple UE around BS to compare different channels scenarios.

### 2) Jammer and null beamforming
```A_jammer.m```

BS is doing beamforming on a UE1 which is transmitting towards it. but another UE2 is jamming. LMS algorithm in bs must be capable of doing null beamforming towards the jammer.

### 3) Train
```A_doppler.m```

This experiment is done in order to see the doppler effect on a real channel. We have a very fast UE (like moving on a train) and the BS is trying to track it. We want to see the doppler effect in this case.

### 4) AMC
```A_amc.m```

We wanted to try to create an adaptive modulation coding algorithm based on the sinr tables. We have a UE1 that is going far away from the BS, so that as the sinr decreases the MC changes accordingly. So we should see the constellation changing.

### 5) altro?
```.m```


# PLOTS TO COMPARE:
* SINR vs TIME
* BER vs TIME
* THROUPUT vs TIME
* MODULATION ORDER vs TIME (in A.4)
* DOPPLER vs TIME (in A.3)


# Christmas BONUS:
## S: 3D, 1BS + 1 BS mobile (drone), 3GPP, OFDM/NR, LMS (wideband)
```bonus.m```






