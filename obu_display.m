%% Simulated OBU screen - Traffic Police Hand Signal Translator
%  RSE4501 Mini Project 2, Part 2
%
%  Reads the signal detected by TrafficSignal_IMU.ino (Arduino Leonardo +
%  MPU-6500 held in the officer's RIGHT hand) and shows it on a simulated
%  ERP 2.0 On-Board Unit (OBU) screen.
%
%  Requires : getData.m in the same folder, MATLAB R2019b or newer
%  Close the Arduino Serial Monitor / Plotter before running.
%
%  Buttons
%   Zero heading  - hold the arm STRAIGHT FORWARD and level, then click
%   Reset display - back to "---"
%   Gyro bias     - put the board down still for ~2 s, then click
%  Close the window to stop. A log (CSV) of every packet and every signal
%  change is saved for the application tests.

clear; clc; close all;

%% ---------------- SETTINGS ----------------
port = "COM5";             % <-- change to your Leonardo port
baud = 115200;

%% ---------------- SIGNAL TEXT (edit wording here) ----------------
% Right hand only. Direction colour code in the junction diagram:
%   S = stop (red), G = go (green), R = prepare to stop (amber),
%   T = turn right allowed (green), - = not indicated (grey)
% dirs order: Front, Behind, Right, Left (relative to the officer)
SIG(1) = struct('title', '- - -', 'msg', 'Waiting for traffic police signal', ...
                'dirs', '----', 'col', [0.35 0.35 0.35]);
SIG(2) = struct('title', 'RIGHT TRAFFIC STOP', ...          % arm raised up
                'msg', {{'Vehicles from the officer''s RIGHT:', 'STOP'}}, ...
                'dirs', '--S-', 'col', [0.80 0.15 0.15]);
SIG(3) = struct('title', 'RIGHT TRAFFIC GO', ...            % arm down by side
                'msg', {{'Vehicles from the officer''s RIGHT:', 'GO'}}, ...
                'dirs', '--G-', 'col', [0.10 0.55 0.25]);
SIG(4) = struct('title', 'PREPARE TO STOP', ...             % arm horizontal to the right
                'msg', {{'Vehicles from the officer''s RIGHT:', 'PREPARE TO STOP'}}, ...
                'dirs', '--R-', 'col', [0.85 0.55 0.05]);
SIG(5) = struct('title', 'TURN RIGHT', ...                  % arm across the body
                'msg', {{'Vehicles in FRONT of the officer:', 'may TURN RIGHT'}}, ...
                'dirs', 'T---', 'col', [0.10 0.45 0.75]);
CANDNAME = {'none', 'arm UP', 'arm DOWN', 'arm SIDE', 'arm ACROSS'};

%% ---------------- SERIAL ----------------
disp("Available ports:"); disp(serialportlist("available"));
board = serialport(port, baud, "Timeout", 2);
configureTerminator(board, "CR/LF");
setDTR(board, true);
pause(0.5); flush(board);

%% ---------------- FIGURE ----------------
fig = figure('Name', 'Simulated OBU - Traffic Police Signal', 'NumberTitle', 'off', ...
             'Color', [0.93 0.93 0.93], 'Position', [80 80 1200 620], ...
             'MenuBar', 'none', 'ToolBar', 'none');
setappdata(fig, 'run', true);
fig.CloseRequestFcn = @(src, ~) setappdata(src, 'run', false);

% ---- OBU screen (left) ----
ax = axes('Parent', fig, 'Units', 'pixels', 'Position', [30 40 780 540], ...
          'XLim', [0 100], 'YLim', [0 70], 'Visible', 'off');
hold(ax, 'on'); axis(ax, 'manual');
rectangle(ax, 'Position', [0 0 100 70], 'Curvature', 0.06, ...
          'FaceColor', [0.06 0.06 0.08], 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 6);
hClock = text(ax, 5, 64, '', 'Color', 'w', 'FontSize', 26, 'FontWeight', 'bold');
hDate  = text(ax, 5, 58.5, '', 'Color', [0.75 0.75 0.75], 'FontSize', 12);
text(ax, 95, 64, 'TRAFFIC POLICE SIGNAL', 'Color', [0.75 0.75 0.75], ...
     'FontSize', 11, 'HorizontalAlignment', 'right');
hBanner = rectangle(ax, 'Position', [5 41 58 13], 'Curvature', 0.15, ...
                    'FaceColor', SIG(1).col, 'EdgeColor', 'none');
hTitle = text(ax, 34, 47.5, SIG(1).title, 'Color', 'w', 'FontSize', 30, ...
              'FontWeight', 'bold', 'HorizontalAlignment', 'center');
hMsg = text(ax, 5, 26, SIG(1).msg, 'Color', 'w', 'FontSize', 17, ...
            'VerticalAlignment', 'middle', 'Interpreter', 'none');
hAge = text(ax, 5, 6, '', 'Color', [0.6 0.6 0.6], 'FontSize', 10);
hWarn = text(ax, 34, 15, '', 'Color', [1 0.35 0.35], 'FontSize', 14, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');

% ---- junction diagram (officer in the middle) ----
cx = 81; cy = 30; s = 7; L = 9;
rectangle(ax, 'Position', [cx-s/2 cy-s/2 s s], 'Curvature', 1, ...
          'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none');
text(ax, cx, cy, 'P', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
dirPos = [cx-s/2  cy+s/2+1  s L;       % front (top)
          cx-s/2  cy-s/2-1-L s L;      % behind (bottom)
          cx+s/2+1 cy-s/2   L s;       % right
          cx-s/2-1-L cy-s/2 L s];      % left
dirLbl = {'FRONT', 'BEHIND', 'RIGHT', 'LEFT'};
lblPos = [cx cy+s/2+L+3; cx cy-s/2-L-3; cx+s/2+L/2+1 cy-s/2-2.5; cx-s/2-L/2-1 cy-s/2-2.5];
for k = 1:4
    hDir(k) = rectangle(ax, 'Position', dirPos(k,:), 'Curvature', 0.2, ...
                        'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'none'); %#ok<SAGROW>
    text(ax, lblPos(k,1), lblPos(k,2), dirLbl{k}, 'Color', [0.8 0.8 0.8], ...
         'FontSize', 9, 'HorizontalAlignment', 'center');
end
text(ax, cx, cy-s/2-L-7, '(directions relative to officer)', 'Color', [0.6 0.6 0.6], ...
     'FontSize', 8, 'HorizontalAlignment', 'center');

% ---- control / data panel (right) ----
uicontrol(fig, 'Style', 'text', 'String', 'IMU data', 'FontWeight', 'bold', ...
          'FontSize', 12, 'Position', [840 560 330 25], 'BackgroundColor', fig.Color, 'ForegroundColor', 'k');
hData = uicontrol(fig, 'Style', 'text', 'String', 'waiting for data...', ...
          'FontName', 'Consolas', 'FontSize', 11, 'HorizontalAlignment', 'left', ...
          'Position', [840 430 330 125], 'BackgroundColor', 'w', 'ForegroundColor', 'k');
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Zero heading (arm straight forward)', ...
          'FontSize', 11, 'Position', [840 385 330 35], ...
          'Callback', @(~,~) writeline(board, "z"));
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset display to - - -', ...
          'FontSize', 11, 'Position', [840 345 330 35], ...
          'Callback', @(~,~) writeline(board, "r"));
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Re-measure gyro bias (board still)', ...
          'FontSize', 11, 'Position', [840 305 330 35], ...
          'Callback', @(~,~) writeline(board, "b"));
uicontrol(fig, 'Style', 'text', 'String', 'Signal log', 'FontWeight', 'bold', ...
          'FontSize', 12, 'Position', [840 270 330 25], 'BackgroundColor', fig.Color, 'ForegroundColor', 'k');
hLog = uicontrol(fig, 'Style', 'listbox', 'String', {}, 'FontName', 'Consolas', ...
          'FontSize', 10, 'Position', [840 40 330 225]);

%% ---------------- MAIN LOOP ----------------
LOG = nan(20000, 7);            % [t heading pitch roll temp signal candidate]
k = 0; events = {}; logLines = {}; lastSig = -1; tChange = tic; t0 = tic;
tLastData = tic; serialDead = false;
while ishandle(fig) && getappdata(fig, 'run')
    newest = [];
    if ~serialDead
        try
            while board.NumBytesAvailable > 0
                d = getData(board, "all");
                if numel(d) < 6, break; end
                newest = d(1:6);
                k = k + 1;
                if k > size(LOG,1), LOG = [LOG; nan(20000,7)]; end %#ok<AGROW>
                LOG(k,:) = [toc(t0) newest];
            end
        catch err                      % USB cable pulled / port lost
            serialDead = true;
            hWarn.String = 'USB CONNECTION LOST - close window, re-plug, run again';
            fprintf(2, 'Serial error: %s\n', err.message);
        end
    end
    if ~isempty(newest)
        tLastData = tic;
        if ~serialDead, hWarn.String = ''; end
    elseif ~serialDead && toc(tLastData) > 1.5
        hWarn.String = 'NO DATA FROM IMU - check jumpers / press reset';
    end

    hClock.String = char(datetime('now', 'Format', 'HH:mm'));
    hDate.String  = char(datetime('now', 'Format', 'eee, dd/MM/yyyy'));

    if ~isempty(newest)
        head = newest(1); pitch = newest(2); roll = newest(3);
        sig = round(newest(5)); cnd = round(newest(6));
        hData.String = sprintf(['Arm pitch   : %7.1f deg\n' ...
                                'Arm heading : %7.1f deg\n' ...
                                'Arm roll    : %7.1f deg\n' ...
                                'Pose now    : %s\n' ...
                                'Shown       : %s'], ...
                                pitch, head, roll, CANDNAME{cnd+1}, SIG(sig+1).title);
        if sig ~= lastSig
            S = SIG(sig+1);
            hBanner.FaceColor = S.col;
            hTitle.String = S.title;
            hMsg.String = S.msg;
            for j = 1:4
                switch S.dirs(j)
                    case 'S', c = [0.85 0.15 0.15];
                    case 'G', c = [0.15 0.70 0.30];
                    case 'T', c = [0.15 0.70 0.30];
                    case 'R', c = [0.95 0.65 0.10];
                    otherwise, c = [0.30 0.30 0.30];
                end
                hDir(j).FaceColor = c;
            end
            if lastSig >= 0
                events(end+1,:) = {toc(t0), SIG(sig+1).title}; %#ok<SAGROW>
                logLines = [{sprintf('%7.1f s  %s', toc(t0), SIG(sig+1).title)}; logLines];
                hLog.String = logLines;
            end
            lastSig = sig; tChange = tic;
        end
    end
    if lastSig > 0
        hAge.String = sprintf('Signal shown for %.0f s', toc(tChange));
    end
    drawnow limitrate;
    pause(0.01);
end

%% ---------------- SAVE ----------------
clear board
if isvalid(fig), delete(fig); end
LOG = LOG(1:k,:);
if k > 0
    tag = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    T = array2table(LOG, 'VariableNames', ...
        {'t_s','heading_deg','pitch_deg','roll_deg','temp_C','signal','candidate'});
    writetable(T, "obu_log_" + tag + ".csv");
    if ~isempty(events)
        E = cell2table(events, 'VariableNames', {'t_s', 'signal'});
        writetable(E, "obu_events_" + tag + ".csv");
    end
    fprintf('Saved obu_log_%s.csv (%d samples) and signal events.\n', tag, k);
end
