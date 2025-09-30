
% Chapter-2
%% plots for 2.4

function breath_alg_plots()
%match esp sampling rate
%testing only with small drift, like recording
fs  = 25;                 % sampling rate
T   = 240;                % sample signal seconds
f0  = 0.25;               % fixed breathing freq 
phi = 0;                 % phase 
B1   = 0;                 % drift
T0_C        = 25.0;       % room absolute temp
driftAmp_C  = 0.10;       % slow drift 
driftHz     = 0.003;      % slow drift frequency 
AC_norm_ppC = 0.60;       % p2p for normal effort - apnea %1
AC_red_ppC  = 0.25;       % p2p for reduced effort - hypoap.
noiseC_sd   = 0.02;       % noise stdev  %0.1
%signal to contain all 3 levels
% 1.0=normal, 0.5=reduced, 0.0=no effort
%normal-reduced-apnea-normal-reduced-apnea-normal
sched = [  0   60  1.0
          60   90  0.5
          90  110  0.0
         110  160  1.0
         160  190  0.5
         190  220  0.0
         220  T    1.0 ];


%observation1: if floor bit high, zc fails
% thermistor deviating around 0.1 at room temp, considering +-0.05 as floor
%observation2: if short_t bit high, env ratio fails
%observation4: ratio is completely dependent on ac amplitude

% band pass
TP.sample_hz=fs; %25hz in esp-1,3
TP.analysis_hz=2; %mode-1
TP.use_hpf=true;
TP.temp_lp_hz=0.8; TP.temp_hp_hz=0.02;
%env tc
TP.env_tau_short_s=0.8; %1,2
TP.env_tau_long_s=25.0; %20, 30
%ratio
TP.abs_env_floor_c=0.050;
TP.ratio_on=1.25; %1.15
TP.ratio_off=1.10; %1
%zc
TP.dc_tau_s=3; 
TP.zc_amp_floor_c=0.05; %0.1
TP.rms_floor_c=0.030;

%simulate effect of different windows, and how they follow signal
%observation2: p2p,rms updates slow as window inc.
win_set_sec = [2 4 8 12];     % P2P and RMS
win_zc_sec  = 6; %for zc 6s works well for normal and slow breaths
%try diff val/adaptive in post processing

% colours for diff windows-> rms,p2p
%https://in.mathworks.com/help/matlab/creating_plots/specify-plot-colors.html
%https://in.mathworks.com/help/matlab/ref/uisetcolor.html => codes
COL.bg   = [0.65 0.65 0.65];
COL.blue = [0.00 0.45 0.74];
COL.orange=[0.85 0.33 0.10];
COL.green=[0.47 0.67 0.19];
COL.purp = [0.49 0.18 0.56];
COL.cyan = [0.30 0.75 0.93];
COL.gold = [0.93 0.69 0.13];
LS = {'-','--',':','-.'};

%% simple signal
t = (0:1/fs:T-1/fs)'; 
amp = zeros(size(t));                 % p2p 
%create normal, reduced and no effort
for k=1:size(sched,1)
    i = t>=sched(k,1) & t<sched(k,2);
    s = sched(k,3);
    if     s==1.0, a=AC_norm_ppC;
    elseif s==0.5, a=AC_red_ppC;
    else,          a=0.0;
    end
    amp(i)=a;
end

% sine wave
w0     = 2*pi*f0;
drift  = driftAmp_C * sin(2*pi*driftHz*t + 1.1);   % slow drift
breath = sin(w0*t + phi);
therm_abs = T0_C + B1*t + drift + (amp/2).*breath + noiseC_sd*randn(size(t)); %T0_C + B1*t + drift + (amp).*breath + noiseC_sd*randn(size(t));

%%  mimic arduino sketch for better comparison
%process function same/close to arduino sketch
% refer esp3 sketch

%iir 1-pole: https://tomroelandts.com/articles/low-pass-single-pole-iir-filter

thermAC = arduino_ac_band(therm_abs, TP, fs);
[envS, envL] = envelope_emas(abs(thermAC), TP.env_tau_short_s, TP.env_tau_long_s, fs);
% to get rid of small denominator
base  = max(envL, 0.8*TP.abs_env_floor_c);
ratio = envS ./ max(base,1e-6);

P2P = cell(numel(win_set_sec),1);  RMS = cell(numel(win_set_sec),1);
for k=1:numel(win_set_sec)
    N = round(win_set_sec(k)*fs);
    P2P{k} = moving_p2p(thermAC, N);
    RMS{k} = moving_rms(thermAC, N);
end

detSig    = detrend_shortEMA(thermAC, TP.dc_tau_s, TP.analysis_hz, fs);
zc_series = moving_zc(detSig, round(win_zc_sec*fs), TP.zc_amp_floor_c);

%simple bandpass, using 2 iir
%1st lpf to remove 50hz and other high f noise, as cutoff is 0.8
%good enough for breathing-> 0.8*60=>48/min
%2nd lpf output tracks constant signal =>dc, as cutoff is 0.02 =>1.2/min
%then subtract this dc, to only plot the ac
%breathing speeds supported: [0.02->1.2, 0.8-> 48] breaths per min
function y = arduino_ac_band(x, P, fs)
alpha_lp = alpha_from_tau( 1/(2*pi*P.temp_lp_hz), fs );
alpha_hp = alpha_from_tau( 1/(2*pi*P.temp_hp_hz), fs );
hp_y = 0; lp_y=0; y = zeros(size(x));
for n=1:numel(x)
    pre = x(n);
    if P.use_hpf, hp_y = hp_y + alpha_hp*(pre - hp_y); pre = pre - hp_y; end
    lp_y = lp_y + alpha_lp*(pre - lp_y); y(n) = lp_y;
end
end


% same iir filter, change the time constants for env
%https://www.luisllamas.es/en/arduino-exponential-low-pass/
function [envS, envL] = envelope_emas(ax, tauS, tauL, fs)
aS = alpha_from_tau(tauS, fs); aL = alpha_from_tau(tauL, fs);
envS = zeros(size(ax)); envL = zeros(size(ax)); sS = ax(1); sL = ax(1);
for n=1:numel(ax), sS = sS + aS*(ax(n)-sS); sL = sL + aL*(ax(n)-sL);
    envS(n)=sS; envL(n)=sL; end
end

%observation3: p2p and rms not much different
function s = moving_p2p(x, N)
s = zeros(size(x));
for n=1:numel(x), i0=max(1,n-N+1); w=x(i0:n); s(n)=max(w)-min(w); end
end

function s = moving_rms(x, N)
s = zeros(size(x));
for n=1:numel(x), i0=max(1,n-N+1); w=x(i0:n); s(n)=sqrt(mean(w.^2)); end
end

function s = moving_zc(x, N, floorVal)
s = zeros(size(x));
for n=1:numel(x)
    i0=max(1,n-N+1); w=x(i0:n); c=0;
    for k=2:numel(w)
        a=w(k); b=w(k-1);
        if (a>floorVal && b<-floorVal) || (a<-floorVal && b>floorVal), c=c+1; end
    end
    s(n)=c;
end
end
%apply single pole iir on ac
function det = detrend_shortEMA(x, dc_tau_s, anaFs, fs)
hop = round(fs/anaFs); dc_a = alpha_from_tau(dc_tau_s, anaFs);
det = zeros(size(x)); dc=0;
for n=1:hop:numel(x)
    dc = dc + dc_a*(x(n)-dc); d = x(n) - dc;
    idx = n:min(n+hop-1,numel(x)); det(idx)=d;
end
end

%alpha=1-e^(-2*pi* f_cutoff/f_sampling )
function a = alpha_from_tau(tau_s, fs), a = 1 - exp(-1/(tau_s*fs)); end


%% plots
%https://in.mathworks.com/help/matlab/creating_plots/plotting-with-two-y-axes.html
%https://in.mathworks.com/help/matlab/ref/yyaxis.html
%https://in.mathworks.com/help/matlab/creating_plots/add-title-axis-labels-and-legend-to-graph.html
%https://in.mathworks.com/help/matlab/ref/yline.html

%[0.2 0.2 0.2]

% Absolute temp
figure('Color','w'); 
plot(t, therm_abs, 'LineWidth',1.2);
grid on; xlim([0 T]); xlabel('Time (s)'); ylabel('Temp (°C)');
title('Thermistor absolute readings — simple sinusoid model');

% P2P (2/4/8/12 s) with AC background
figure('Color','w');
yyaxis left
plot(t, thermAC, 'LineWidth',0.9, 'Color',COL.bg); ylabel('AC (°C)'); hold on; grid on; xlim([0 T]); ylim([-0.5 1.5]);
yyaxis right
cols = {COL.blue, COL.orange, COL.green, COL.purp};
for k=1:numel(win_set_sec)
    plot(t, P2P{k}, LS{k}, 'LineWidth',1.8, 'Color', cols{k}); ylim([-0.5 1.5]);
end
xlabel('Time (s)'); ylabel('P2P (°C)');
legend(['AC (bg)'; compose('P2P %d s', win_set_sec)'],'Location','best');
title('Peak-to-peak vs window length (AC background)');

% AC + short/long EMA envelopes
figure('Color','w');
plot(t, thermAC,'LineWidth',1.0,'Color',COL.bg); hold on;
plot(t, envS,'LineWidth',1.8,'Color',COL.blue);
plot(t, envL,'LineWidth',1.8,'Color',COL.orange); ylim([-0.5 1.5]);
yline(TP.abs_env_floor_c,'--','Floor','Color',[0.2 0.2 0.2]);
grid on; xlim([0 T]); xlabel('Time (s)'); ylabel('AC / Env (°C)');
legend('AC (bg)','Short EMA','Long EMA','Floor','Location','best');
title('AC waveform with short and long EMA envelopes');

% AC + ratio
figure('Color','w');
yyaxis left
plot(t, thermAC, 'LineWidth',0.9, 'Color',COL.bg); ylabel('AC (°C)'); hold on; ylim([-0.5 1.5]);
yyaxis right
plot(t, ratio, 'LineWidth',1.8, 'Color',COL.orange); ylim([0 2]); % ratio axis
yline(TP.ratio_on,'--','ratio\_on','Color',[0.2 0.2 0.2]);
yline(TP.ratio_off,'--','ratio\_off','Color',[0.2 0.2 0.2]);
ylabel('Short/Long ratio'); grid on; xlim([0 T]); xlabel('Time (s)');
legend('AC (bg)','ratio','Location','best');
title('AC waveform and envelope ratio');

%RMS (2/4/8/12 s) with AC background
figure('Color','w');
yyaxis left
plot(t, thermAC,'LineWidth',0.9,'Color',COL.bg); ylabel('AC (°C)'); hold on; grid on; xlim([0 T]); ylim([-0.5 1.5]);
yyaxis right
colsR = {COL.blue, COL.gold, COL.purp, COL.cyan};
for k=1:numel(win_set_sec)
    plot(t, RMS{k}, LS{k}, 'LineWidth',1.8, 'Color', colsR{k}); ylim([-0.5 1.5]);
end
yline(TP.rms_floor_c,'--','RMS floor','Color',[0.2 0.2 0.2]);
xlabel('Time (s)'); ylabel('RMS (°C)');
legend(['AC (bg)'; compose('RMS %d s', win_set_sec)'; "Floor"],'Location','best');
title('RMS vs window length (AC background)');

% AC + ZC count 
figure('Color','w');
yyaxis left
plot(t, thermAC,'LineWidth',0.9,'Color',COL.bg); hold on; ylim([-1 1.5]);
plot(t, detSig,'LineWidth',1.6,'Color',COL.blue); ylabel('Detrended AC / AC (°C)'); ylim([-1 1.5]);
yyaxis right
stairs(t, zc_series,'LineWidth',1.8,'Color',COL.orange); ylabel('ZC count / window');
yline(2,'--','zc\_min=2','Color',[0.2 0.2 0.2]);
grid on; xlim([0 T]); xlabel('Time (s)');
legend('AC (bg)','Detrended AC','ZC count','Location','best');
title(sprintf('Detrended AC and zero-crossings (win=%.1f s, floor=%.2f °C)', ...
      win_zc_sec, TP.zc_amp_floor_c));
end
%%