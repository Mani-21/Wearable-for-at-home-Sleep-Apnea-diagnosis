%https://in.mathworks.com/help/matlab/ref/uigetfile.html

function plot_sleep_recording_data(csvFile)
% start and stop time
ZOOM_START_S = 0;    
ZOOM_STOP_S  = 500;    

%%upload file
[f,p] = uigetfile({'*.csv','CSV files'}, 'CSV');
csvFile = fullfile(p,f);
T = readtable(csvFile, 'TextType','string');

%time
dt  = datetime(string(T.Date)+" "+string(T.Time), ...
               'InputFormat','dd-MM-yyyy HH:mm:ss', ...
               'TimeZone','local');
tsec = seconds(dt - dt(1));   % seconds from file start
dt = diff(tsec);
dt = dt(dt > 0 & isfinite(dt));
if isempty(dt)
    fs = 2;         
else
    fs = 1/mean(dt); %average gap to an average rate   
end
%extract each column
ax = getNumCol(T,"ax"); ay = getNumCol(T,"ay"); az = getNumCol(T,"az");
gx = getNumCol(T,"gx"); gy = getNumCol(T,"gy"); gz = getNumCol(T,"gz");
Env  = getNumCol(T,"Env");    Ratio = getNumCol(T,"Ratio");
RMS  = getNumCol(T,"RMS");    ZC    = getNumCol(T,"ZC");
Eff  = getNumCol(T,"Effort"); Conf  = getNumCol(T,"Conf");
Post = getNumCol(T,"Posture"); Motion = getNumCol(T,"Motion");
Apnea = getNumCol(T,"Apnea");
TempC = getNumCol(T,"TempC");
EnvT  = getNumCol(T,"EnvT");   RatioT = getNumCol(T,"RatioT");
RMST  = getNumCol(T,"RMST");   ZCT    = getNumCol(T,"ZCT");
EffT  = getNumCol(T,"EffortT");
Snore = getNumCol(T,"Snore");
DFreq = getNumCol(T,"DFreq");
Mag   = getNumCol(T,"Mag");
Amp   = getNumCol(T,"Amp");

function x = getNumCol(TT, v)
    if ismember(v, TT.Properties.VariableNames)
        x = double(TT.(v));
    end
end


%% IMU Effort confidence refinement 
ConfSm    = movmean(Conf, max(2, round(1.0*fs)), 'omitnan');  % 1 s MA, >=2 samples
effLikely = (ConfSm >= 0.90) | (Eff > 0.5);  % atleast 90% score
EffRef    = debounceBinary(effLikely, max(1,ceil(1*fs)), max(1,ceil(2*fs)));% ON pause 1 s, OFF 2 s

function y = debounceBinary(x, k_on, k_off)
x = logical(x); n = numel(x); y = false(size(x));
state = false; len = 0;
    for i = 1:n
        if x(i) == state, len = len + 1; else, len = 1; end
        if ~state && x(i) && len >= k_on,  state = true;  len = 0; end
        if  state && ~x(i) && len >= k_off, state = false; len = 0; end
        y(i) = state;
    end
end

%% zoom window 
if isempty(ZOOM_START_S) || isempty(ZOOM_STOP_S)
    zoomRangeSec = [tsec(1)  min(tsec(1)+30, tsec(end))]; % default first 30 s
else
    z = sort(double([ZOOM_START_S, ZOOM_STOP_S]));
    %available range
    z(1) = max(z(1), tsec(1));
    z(2) = min(z(2), tsec(end));
    zoomRangeSec = z;
end
maskZoom = (tsec >= zoomRangeSec(1)) & (tsec <= zoomRangeSec(2));
zoomLbl  = sprintf('%.0f–%.0f s', zoomRangeSec(1), zoomRangeSec(2));

%% plots: FIGURE 1: IMU (full recording) 
figure('Name','ESP1 – IMU','Color','w','Position',[100 100 1200 1000]);

subplot(6,1,1);
plot(tsec, ax,'-','DisplayName','ax'); hold on;
plot(tsec, ay,'-','DisplayName','ay'); plot(tsec, az,'-','DisplayName','az');
hold off; grid on; ylabel('m/s^2'); title('Accelerometer (ax, ay, az)');
xlim([tsec(1) tsec(end)]); legend('Location','northeast');

subplot(6,1,2);
plot(tsec, gx,'-','DisplayName','gx'); hold on;
plot(tsec, gy,'-','DisplayName','gy'); plot(tsec, gz,'-','DisplayName','gz');
hold off; grid on; ylabel('rad/s'); title('Gyroscope (gx, gy, gz)');
xlim([tsec(1) tsec(end)]); legend('Location','northeast');

subplot(6,1,3);
plot(tsec, Env,'DisplayName','Env'); hold on; plot(tsec, Ratio,'DisplayName','Ratio');
hold off; grid on; title('IMU Effort Features — Envelope & Ratio');
xlim([tsec(1) tsec(end)]); legend('Location','northeast');

subplot(6,1,4);
yyaxis left;  p1 = plot(tsec, RMS,'-','DisplayName','RMS'); ylabel('RMS');
yyaxis right; p2 = stairs(tsec, ZC,'-','DisplayName','ZC');  ylabel('ZC');
grid on; title('IMU Oscillation — RMS (left) & ZC (right)');
xlim([tsec(1) tsec(end)]); legend([p1 p2],'Location','northeast');

subplot(6,1,5);
yyaxis left;  stairs(tsec, Post,'-','LineWidth',1.2,'DisplayName','Posture'); ylim([-0.5 5.5]);
yticks(0:5); yticklabels({'Unk','Prone','Right','Left','Supine','Stand'}); ylabel('Posture');
yyaxis right; stairs(tsec, Motion,'-','LineWidth',1.2,'DisplayName','Motion'); ylim([-0.1 1.1]);
yticks([0 1]); yticklabels({'Still','Moving'}); ylabel('Motion');
grid on; title('Posture and Motion'); xlim([tsec(1) tsec(end)]); legend('Location','northeast');
%https://in.mathworks.com/help/matlab/ref/stairs.html
subplot(6,1,6);
stairs(tsec, Eff,'-','DisplayName','Effort (orig)'); hold on;
stairs(tsec, EffRef,'-','LineWidth',2,'DisplayName','Effort (refined)');
if all(~isnan(Apnea)), stairs(tsec, Apnea,'-','DisplayName','Apnea flag'); end
hold off; grid on; ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'NO','YES'});
title('IMU Effort: Original vs Refined'); xlabel('Time (s)'); xlim([tsec(1) tsec(end)]);
legend('Location','northeast');

%% FIGURE 2: Thermistor (full recording) 
figure('Name','ESP3 – Thermistor','Color','w','Position',[150 150 1200 900]);

subplot(4,1,1); plot(tsec, TempC); grid on; ylabel('°C');
title('Thermistor: Absolute Temperature'); xlim([tsec(1) tsec(end)]);

subplot(4,1,2);
plot(tsec, EnvT,'DisplayName','EnvT'); hold on; plot(tsec, RatioT,'DisplayName','RatioT');
hold off; grid on; title('Envelope & Ratio'); xlim([tsec(1) tsec(end)]); legend('Location','northeast');

subplot(4,1,3);
yyaxis left;  p3 = plot(tsec, RMST,'-','DisplayName','RMS'); ylabel('RMS');
yyaxis right; p4 = stairs(tsec, ZCT,'-','DisplayName','ZC'); ylabel('ZC');
grid on; title('Thermistor Oscillation — RMS (left) & ZC (right)');
xlim([tsec(1) tsec(end)]); legend([p3 p4],'Location','northeast');

subplot(4,1,4);
stairs(tsec, EffT,'-','DisplayName','EffortT'); hold on;
if all(~isnan(Apnea)), stairs(tsec, Apnea,'-','DisplayName','Apnea flag'); end
hold off; grid on; ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'NO','YES'});
title('Thermistor Effort + Apnea Flag'); xlabel('Time (s)'); xlim([tsec(1) tsec(end)]);
legend('Location','northeast');

%% FIGURE 3: Microphone (full recording) 
figure('Name','ESP3 – Microphone','Color','w','Position',[200 200 1200 900]);

subplot(4,1,1); plot(tsec, Amp); grid on; title('Average Amplitude (|signal|)'); xlim([tsec(1) tsec(end)]); ylabel('Amp');
subplot(4,1,2); plot(tsec, Mag,'DisplayName','Max Mag'); grid on; title('Max Spectral Magnitude');
xlim([tsec(1) tsec(end)]); legend('Location','northeast');
subplot(4,1,3); plot(tsec, DFreq); grid on; title('Dominant Frequency (Hz)'); ylabel('Hz');
xlim([tsec(1) tsec(end)]); yline(600,'--'); yline(1400,'--');
subplot(4,1,4); stairs(tsec, Snore,'DisplayName','Snore'); grid on;
ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'NO','YES'});
title('Snore Decision'); xlabel('Time (s)'); xlim([tsec(1) tsec(end)]); legend('Location','northeast');

%% FIGURE 4, 5 Zoom 
figure('Name','ESP1 – IMU (custom zoom)','Color','w','Position',[120 120 1200 1000]);

subplot(6,1,1);
plot(tsec(maskZoom), ax(maskZoom),'-','DisplayName','ax'); hold on;
plot(tsec(maskZoom), ay(maskZoom),'-','DisplayName','ay');
plot(tsec(maskZoom), az(maskZoom),'-','DisplayName','az'); hold off; grid on;
ylabel('m/s^2'); title(['Accelerometer (ax, ay, az) — ' zoomLbl]); xlim(zoomRangeSec);
legend('Location','northeast');

subplot(6,1,2);
plot(tsec(maskZoom), gx(maskZoom),'-','DisplayName','gx'); hold on;
plot(tsec(maskZoom), gy(maskZoom),'-','DisplayName','gy');
plot(tsec(maskZoom), gz(maskZoom),'-','DisplayName','gz'); hold off; grid on;
ylabel('rad/s'); title(['Gyroscope — ' zoomLbl]); xlim(zoomRangeSec);
legend('Location','northeast');

subplot(6,1,3);
plot(tsec(maskZoom), Env(maskZoom),'DisplayName','Env'); hold on;
plot(tsec(maskZoom), Ratio(maskZoom),'DisplayName','Ratio'); hold off; grid on;
title(['IMU Envelope & Ratio — ' zoomLbl]); xlim(zoomRangeSec); legend('Location','northeast');

subplot(6,1,4);
yyaxis left;  p1 = plot(tsec(maskZoom), RMS(maskZoom),'-','DisplayName','RMS'); ylabel('RMS');
yyaxis right; p2 = stairs(tsec(maskZoom), ZC(maskZoom),'-','DisplayName','ZC'); ylabel('ZC');
grid on; title(['IMU Oscillation — ' zoomLbl]); xlim(zoomRangeSec);
legend([p1 p2],'Location','northeast');

subplot(6,1,5);
yyaxis left;  stairs(tsec(maskZoom), Post(maskZoom),'-','LineWidth',1.2,'DisplayName','Posture'); ylim([-0.5 5.5]);
yticks(0:5); yticklabels({'Unk','Prone','Right','Left','Supine','Stand'}); ylabel('Posture');
yyaxis right; stairs(tsec(maskZoom), Motion(maskZoom),'-','LineWidth',1.2,'DisplayName','Motion'); ylim([-0.1 1.1]);
yticks([0 1]); yticklabels({'Still','Moving'}); ylabel('Motion');
grid on; title(['Posture & Motion — ' zoomLbl]); xlim(zoomRangeSec); legend('Location','northeast');

subplot(6,1,6);
stairs(tsec(maskZoom), Eff(maskZoom),'-','DisplayName','Effort (orig)'); hold on;
stairs(tsec(maskZoom), EffRef(maskZoom),'-','LineWidth',2,'DisplayName','Effort (refined)');
if all(~isnan(Apnea)), stairs(tsec(maskZoom), Apnea(maskZoom),'-','DisplayName','Apnea flag'); end
hold off; grid on; ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'NO','YES'});
title(['IMU Effort — ' zoomLbl]); xlabel('Time (s)'); xlim(zoomRangeSec); legend('Location','northeast');

% ---- Thermistor (custom window)
figure('Name','ESP3 – Thermistor (custom zoom)','Color','w','Position',[170 170 1200 900]);

subplot(4,1,1); plot(tsec(maskZoom), TempC(maskZoom));
grid on; ylabel('°C'); title(['Thermistor Temperature — ' zoomLbl]); xlim(zoomRangeSec);

subplot(4,1,2);
plot(tsec(maskZoom), EnvT(maskZoom),'DisplayName','EnvT'); hold on;
plot(tsec(maskZoom), RatioT(maskZoom),'DisplayName','RatioT'); hold off; grid on;
title(['Envelope & Ratio — ' zoomLbl]); xlim(zoomRangeSec); legend('Location','northeast');

subplot(4,1,3);
yyaxis left;  p3 = plot(tsec(maskZoom), RMST(maskZoom),'-','DisplayName','RMS'); ylabel('RMS');
yyaxis right; p4 = stairs(tsec(maskZoom), ZCT(maskZoom),'-','DisplayName','ZC'); ylabel('ZC');
grid on; title(['Oscillation — ' zoomLbl]); xlim(zoomRangeSec); legend([p3 p4],'Location','northeast');

subplot(4,1,4);
stairs(tsec(maskZoom), EffT(maskZoom),'-','DisplayName','EffortT'); hold on;
if all(~isnan(Apnea)), stairs(tsec(maskZoom), Apnea(maskZoom),'-','DisplayName','Apnea flag'); end
hold off; grid on; ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'NO','YES'});
title(['Thermistor Effort — ' zoomLbl]); xlabel('Time (s)'); xlim(zoomRangeSec); legend('Location','northeast');

end
