function report_sleep_summary(csvFile)
[f,p] = uigetfile({'*.csv','CSV files'}, 'CSV');
csvFile = fullfile(p,f);
T = readtable(csvFile, 'TextType','string');

%time

dt  = datetime(string(T.Date)+" "+string(T.Time), ...
               'InputFormat','dd-MM-yyyy HH:mm:ss','TimeZone','local');
tsec = seconds(dt - dt(1));

function fs = eff_rate(t)
d = diff(t); d = d(d>0); fs = isempty(d) * 2 + ~isempty(d) * (1/median(d));
end

fs   = eff_rate(tsec);

startTime = dt(1); stopTime = dt(end);
totalSec  = seconds(stopTime - startTime);

function x = getNumCol(T, v)
if ismember(v, T.Properties.VariableNames), x = double(T.(v));
else, x = nan(height(T),1); end
end


%need imu effort to classify type of apnea

% columns
Apnea = getNumCol(T,"Apnea");
Snore = getNumCol(T,"Snore");
Eff0  = getNumCol(T,"Effort");
Conf  = getNumCol(T,"Conf");
Post  = getNumCol(T,"Posture");

%% IMU effort same as last sketch
winConf = max(1, round(1.0 * fs));
ConfSm  = movmean(Conf, winConf, 'omitnan');
effLikely = (ConfSm >= 0.9) | (Eff0 > 0.5);
k_on  = max(1, round(1 * fs));
k_off = max(1, round(2 * fs));
Eff = debounceBinary(effLikely, k_on, k_off);

function y = debounceBinary(x, k_on, k_off)
x = logical(x(:)); n = numel(x); y = false(size(x));
state=false; len=0;
for i = 1:n
    if x(i)==state, len=len+1; else, len=1; end
    if ~state && x(i) && len>=k_on,  state=true;  len=0; end
    if  state && ~x(i) && len>=k_off, state=false; len=0; end
    y(i)=state;
end
end

%% events
apneaIdx = rising_edges(Apnea > 0.5);
snoreIdx = rising_edges(Snore > 0.5);

% type apneas using 10 s before each pulse
typeWinSec = 10;
[apneaTypes, apneaN] = typeApneaEdges(apneaIdx, Eff, fs, typeWinSec);
snoreN = numel(snoreIdx);

function [types, N] = typeApneaEdges(edgeIdx, Eff, fs, winSec)
N = numel(edgeIdx); cntC=0; cntO=0; cntM=0;
win = max(1, round(winSec*fs));
for i = 1:N
    e = edgeIdx(i); s = max(1, e - win + 1);     % 10 s before
    seg = Eff(s:e);
    if ~any(seg)
        cntC = cntC + 1;
    else
        fracEff = mean(seg>0.5);
        %osa>70, csa<30 else msa
        if     fracEff > 0.70, cntO = cntO + 1;
        elseif fracEff < 0.30, cntC = cntC + 1;
        else                   cntM = cntM + 1;
        end
    end
end
types = struct('central',cntC,'obstructive',cntO,'mixed',cntM);
end

function s = dur(secTotal)
h = floor(secTotal/3600);
m = floor(mod(secTotal,3600)/60);
s2= round(mod(secTotal,60));
s = sprintf('%02d:%02d:%02d', h, m, s2);
end

function idx = rising_edges(mask)
mask = logical(mask(:)); d = diff([false; mask]); idx = find(d==1);
end

%% posture durations
labels = ["Unknown","Prone","Right","Left","Supine","Stand"];
postSecs = zeros(1,6);
for k = 0:5, postSecs(k+1) = sum(Post==k)/fs; end
postPct = 100*postSecs/sum(postSecs);

%% snoring and apnea timestamps
function printStamps(titleStr, dtAll, idx, fmtStr)
    fprintf('%s\n', titleStr);
    if isempty(idx), fprintf('    (none)\n'); return; end
    idx = idx(idx>=1 & idx<=numel(dtAll));            % bounds guard
    for ii = 1:numel(idx)
        fprintf('    - %s\n', datestr(dtAll(idx(ii)), fmtStr));
    end
end

%% print
fmt = 'yyyy-mm-dd HH:MM:SS';
fprintf('SLEEP SUMMARY\n=============\n');
fprintf('Start time    : %s\n', datestr(startTime,fmt));
fprintf('Stop  time    : %s\n', datestr(stopTime ,fmt));
fprintf('Total time    : %s (%.2f h)\n\n', dur(totalSec), totalSec/3600);

fprintf('Respiratory Events\n-------------------\n');
fprintf('Apneas        : %d  (Central=%d, Obstructive=%d, Mixed=%d)\n', ...
    apneaN, apneaTypes.central, apneaTypes.obstructive, apneaTypes.mixed);
printStamps('  Apnea timestamps:', dt, apneaIdx, fmt);

fprintf('Snoring       : %d episodes\n', snoreN);
printStamps('  Snore timestamps:', dt, snoreIdx, fmt);

fprintf('\nSleep Posture (duration and percent of recording)\n');
fprintf('-------------------------------------------------\n');
for i = 1:numel(labels)
    fprintf('  %-8s : %8s  (%5.1f%%)\n', labels(i), dur(postSecs(i)), postPct(i));
end
fprintf('\n'); drawnow;


end







