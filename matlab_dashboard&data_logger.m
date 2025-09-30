%https://in.mathworks.com/help/matlab/ref/fopen.html

function live_sleep_plot()
    hubIP    = 'sleep-hub.local';   % mdns or actual IP of ESP2
    hubPort  = 8000;                % TCP port ESP2 sends CSV on
    outFile  = 'esp2_full_p11.csv';% file to save full incoming data
    maxPoints = 500;                % max points in live plots
    % ESP2 CSV header (36 columns) 
    header = [ ...
        'Date,Time,Mode,' , ...
        'ax,ay,az,gx,gy,gz,Posture,Motion,Env,Ratio,RMS,ZC,Effort,Why,Conf,' , ...
        'TempC,EnvT,RatioT,RMST,ZCT,EffortT,WhyT,ConfT,' , ...
        'Snore,DFreq,Mag,Amp,Apnea,' , ...
        'IRdc,REDdc,SpO2,HR,ApneaCount' ...
    ];

    %column index
    COL_Date   = 1;
    COL_Time   = 2;
    COL_SpO2   = 34;
    COL_TempC  = 19;
    COL_EffIMU = 16;  % IMU effort (0/1)
    COL_EffTH  = 24;  % Thermistor effort (0/1)

    %output file, add header
    needHeader = true;
    if isfile(outFile)
        d = dir(outFile);
        if d.bytes > 0, needHeader = false; end
    end
    fid = fopen(outFile, 'a');
    if needHeader, fprintf(fid, '%s\n', header); end
    fclose(fid);

    %connect to ESP2 
    %https://in.mathworks.com/help/matlab/ref/tcpclient.html
    c = tcpclient(hubIP, hubPort, 'Timeout', 2, 'ConnectTimeout', 5);
    flush(c);
    disp("Connected to " + hubIP + ":" + hubPort);

    %live figure 
    figure('Name','Live Sleep Data','NumberTitle','off','Color','w');
    %spo2
    tiledlayout(3,1);
    ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on');
    title(ax1,'SpO₂ (%)'); ylabel(ax1,'%'); ylim(ax1,[85 100]);

    %effort signals
    ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on');
    title(ax2,'Breathing Effort (IMU vs Thermistor)'); ylabel(ax2,'0/1');
    ylim(ax2,[-0.2 1.2]);
    hEffIMU  = plot(ax2, NaT, NaN, '-', 'DisplayName','IMU Effort');
    hEffTher = plot(ax2, NaT, NaN, '-', 'DisplayName','Therm Effort');
    legend(ax2,'Location','northwest');
    
    %thermistor signal, with this easy to check if thermistor placed
    %properly
    ax3 = nexttile; hold(ax3,'on'); grid(ax3,'on');
    title(ax3,'Breathing Temperature (°C)'); ylabel(ax3,'°C');

    % buffers
    timestamps = datetime.empty(0,1);
    spo2Data   = [];
    tempData   = [];
    effIMU     = [];
    effTher    = [];
    rxBuf = '';

    %% live loop
    while ishandle(ax1)
        % read any available bytes
        nb = c.NumBytesAvailable; %https://in.mathworks.com/help/matlab/ref/tcpclient.html
        if nb > 0
            bytes = read(c, nb, 'uint8');     
            rxBuf = [rxBuf, char(bytes(:).')];% add to buffer
        else
            pause(0.05);
        end

        % 
        while true
            lfPos = find(rxBuf == newline, 1, 'first'); % '\n'
            if isempty(lfPos), break; end

            rawLine = rxBuf(1:lfPos-1);
            if ~isempty(rawLine) && rawLine(end) == char(13) % '\r'
                rawLine(end) = [];
            end
            rxBuf = rxBuf(lfPos+1:end);

            if isempty(rawLine), continue; end
            strLine = string(strtrim(rawLine));
            if numel(strLine) ~= 1, continue; end

            % skip all unwanted
            if strlength(strLine) < 10 || ~contains(strLine, ",") 
                continue;
            end

            % add line to file
            fid = fopen(outFile, 'a');
            if fid ~= -1
                fprintf(fid, "%s\n", strLine);
                fclose(fid);
            end

            % parse CSV
            parts = split(strLine, ",");
            if numel(parts) ~= 36
                continue; % skip partial
            end

            %timestamp
            try
                ts = datetime(parts(COL_Date) + " " + parts(COL_Time), ...
                              'InputFormat','d-M-yyyy HH:mm:ss');
            catch
                ts = datetime('now');
            end

            % get fields
            spo2   = str2double(parts(COL_SpO2));
            tempC  = str2double(parts(COL_TempC));
            eIMU   = str2double(parts(COL_EffIMU));
            eTH    = str2double(parts(COL_EffTH));

            if any(isnan([spo2, tempC, eIMU, eTH])), continue; end

            % append
            timestamps(end+1,1) = ts; 
            spo2Data(end+1,1)   = spo2; 
            tempData(end+1,1)   = tempC;
            effIMU(end+1,1)     = eIMU; 
            effTher(end+1,1)    = eTH;  

            % trim buffers
            if numel(timestamps) > maxPoints
                idx = (numel(timestamps)-maxPoints+1):numel(timestamps);
                timestamps = timestamps(idx);
                spo2Data   = spo2Data(idx);
                tempData   = tempData(idx);
                effIMU     = effIMU(idx);
                effTher    = effTher(idx);
            end

            % update plots
            cla(ax1); plot(ax1, timestamps, spo2Data, '-'); ylim(ax1,[85 100]);
            set(hEffIMU,  'XData',timestamps, 'YData',effIMU);
            set(hEffTher, 'XData',timestamps, 'YData',effTher);
            cla(ax3); plot(ax3, timestamps, tempData, '-');
            drawnow limitrate;
        end
    end
end