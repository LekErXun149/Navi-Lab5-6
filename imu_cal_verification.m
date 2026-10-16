%% IMU CALIBRATION VERIFICATION  (RSE4501 Mini Project 2, Part 1 d-g)
%  Compares orientation accuracy WITHOUT and WITH the six-position
%  accelerometer calibration, using NEW data (not the calibration batches).
%
%  Hardware : Arduino Leonardo + GY-9250 (MPU-6500), MPU lying flat on the
%             breadboard, component side up, VCC end = forward
%  Arduino  : IMU_NED_Calibrated.ino with  #define AUTO_LEVEL 0
%  MATLAB   : getData.m in the same folder, R2019b or newer
%  Close the Arduino Serial Monitor / Plotter before running.
%
%  Method
%   - The Arduino runs a calibrated and an uncalibrated filter side by side,
%     so both are measured at the SAME moment in the SAME pose.
%   - Six reference poses (0, +/-90, 180 deg), each recorded twice:
%       A = normal, B = board turned 180 deg about the vertical on the same spot.
%     Averaging A and B cancels the slope of the table/box (reversal method),
%     leaving the sensor error that calibration is meant to remove.
%   - Gravity magnitude |a| (should be exactly 1 g) is a reference-free check.
%   - A static run gives noise, pitch/roll stability and yaw drift.
%
%  Set RUN_EXPERIMENT = false to re-analyse a saved .mat without hardware.

clear; clc; close all;

%% ================= SETTINGS =================
RUN_EXPERIMENT = true;
port       = "COM5";      % <-- change to your Leonardo port
baud       = 115200;
recDur     = 8;           % s per recording
settleTime = 1.5;         % s wait after Enter before recording
staticDur  = 120;         % s static run (noise + drift)

% ---- Objectives (state and justify these in the report) ----
obj.tiltRMS_deg  = 1.0;   % calibrated tilt RMS error (reversal-averaged)
obj.gRMS_g       = 0.005; % calibrated gravity-magnitude RMS error
obj.noise_deg    = 0.2;   % pitch/roll SD at rest
obj.yawDrift_dpm = 2.0;   % yaw drift at rest after start-up bias removal
obj.minRate_Hz   = 15;    % output rate

% ---- Reference poses (NED, VCC end = forward) ----
% name | instruction | ref roll | ref pitch | evaluate roll? | evaluate pitch?
P = {
 'level',      'FLAT, component side UP, VCC end pointing forward',            0,    0, true,  true
 'upside',     'UPSIDE DOWN (component side facing the table)',               180,   0, true,  true
 'right_down', 'RIGHT side down: X arrow points DOWN, header pins point UP',   90,   0, true,  true
 'left_down',  'LEFT side down: X arrow points UP, header pins point DOWN',   -90,   0, true,  true
 'nose_up',    'NOSE UP: VCC end of the header pointing straight UP',           0,   90, false, true
 'nose_down',  'NOSE DOWN: VCC end of the header pointing straight DOWN',       0,  -90, false, true
};
nP = size(P, 1);

tag    = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outDir = "calverify_" + tag;
mkdir(outDir);

%% ================= (d) EXPERIMENT =================
% Recorded columns: [t yaw pitch roll temp pitch_raw roll_raw g_cal g_raw]
if RUN_EXPERIMENT
    board = serialport(port, baud, "Timeout", 2);
    configureTerminator(board, "CR/LF");
    setDTR(board, true);
    pause(0.5);
    writeline(board, "v1");               % extended packet
    writeline(board, "c1");               % main output = calibrated
    pause(0.3); flush(board);
    d = getData(board, "all");
    if numel(d) < 8
        error(['No verify packet. Check: IMU_NED_Calibrated.ino uploaded, ' ...
               'correct port, Serial Monitor closed.']);
    end

    fprintf('\n===== CALIBRATION VERIFICATION =====\n');
    fprintf('Each pose is recorded twice: A (normal) and B (turned 180 deg on the same spot).\n');
    fprintf('Brace the breadboard against a heavy box / wall, LET GO, then click "Record" in the pop-up.\n');

    R = struct('name', {}, 'side', {}, 'D', {});
    for i = 1:nP
        for side = ['A' 'B']
            msg = sprintf('[%d/%d]  %s\n\nSIDE %s', i, nP, P{i,2}, side);
            if side == 'B'
                msg = [msg sprintf(['\n\nSpin the board HALF A TURN like a plate ' ...
                       '(same spot, same face/edge down) so it faces the opposite way.'])];
            end
            fprintf('\n%s\n', strrep(msg, sprintf('\n\n'), ' - '));
            waitUser([msg sprintf('\n\nSet the pose, LET GO, then click "Record".')]);
            R(end+1) = struct('name', P{i,1}, 'side', side, ...
                              'D', recordIMU(board, recDur, settleTime)); %#ok<SAGROW>
        end
    end

    fprintf('\n===== STATIC RUN (%d s) =====\n', staticDur);
    waitUser(sprintf('STATIC RUN (%d s)\n\nLay the board FLAT, do not touch the desk, then click "Record".', staticDur));
    writeline(board, "z");                % zero yaw at the start
    S = recordIMU(board, staticDur, settleTime);

    writeline(board, "v0");
    clear board
    dataFile = fullfile(outDir, "calverify_data_" + tag + ".mat");
    save(dataFile, 'R', 'S', 'P');
    fprintf('\nRaw data saved: %s\n', dataFile);
else
    [f, p] = uigetfile('calverify_data_*.mat', 'Select saved verification data');
    if isequal(f, 0), return; end
    L = load(fullfile(p, f)); R = L.R; S = L.S; P = L.P; nP = size(P, 1);
end

diary(fullfile(outDir, 'verification_summary.txt'));

%% ================= (e) ANALYSIS =================
% ---- per-recording means ----
for k = 1:numel(R)
    D = R(k).D;
    R(k).pitchCal = mean(D(:,3));      R(k).rollCal = circMean(D(:,4));
    R(k).pitchRaw = mean(D(:,6));      R(k).rollRaw = circMean(D(:,7));
    R(k).sdPitch  = std(D(:,3));       R(k).sdRoll  = circStd(D(:,4));
    R(k).gCal     = mean(D(:,8));      R(k).gRaw    = mean(D(:,9));
    R(k).n        = size(D,1);
end

% ---- angle error table (reversal-averaged) ----
rows = {};
for i = 1:nP
    A = R(strcmp({R.name}, P{i,1}) & [R.side] == 'A');
    B = R(strcmp({R.name}, P{i,1}) & [R.side] == 'B');
    angs = {};
    if P{i,5}, angs(end+1,:) = {'roll',  P{i,3}, A.rollRaw,  B.rollRaw,  A.rollCal,  B.rollCal};  end %#ok<SAGROW>
    if P{i,6}, angs(end+1,:) = {'pitch', P{i,4}, A.pitchRaw, B.pitchRaw, A.pitchCal, B.pitchCal}; end %#ok<SAGROW>
    for j = 1:size(angs,1)
        ref = angs{j,2};
        rawAvg = circMean([angs{j,3}; angs{j,4}]);
        calAvg = circMean([angs{j,5}; angs{j,6}]);
        rows(end+1,:) = {P{i,1}, angs{j,1}, ref, ...
            angs{j,3}, angs{j,4}, rawAvg, wrap180(rawAvg - ref), ...
            angs{j,5}, angs{j,6}, calAvg, wrap180(calAvg - ref), ...
            wrap180(angs{j,5} - angs{j,6}) / 2}; %#ok<SAGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'Pose','Angle','Ref_deg', ...
    'Raw_A','Raw_B','Raw_avg','Raw_error', ...
    'Cal_A','Cal_B','Cal_avg','Cal_error','Surface_tilt'});
numCols = 4:12;
for c = numCols, T{:,c} = round(T{:,c}, 2); end

fprintf('\n--- Tilt accuracy (A/B reversal-averaged), degrees ---\n');
disp(T);

errRaw = T.Raw_error;  errCal = T.Cal_error;
M = @(e) [mean(e), std(e), sqrt(mean(e.^2)), max(abs(e))];
tiltStats = array2table(round([M(errRaw); M(errCal)], 3), ...
    'VariableNames', {'Mean_deg','SD_deg','RMS_deg','MaxAbs_deg'}, ...
    'RowNames', {'Uncalibrated','Calibrated'});
fprintf('--- Tilt error summary ---\n'); disp(tiltStats);

% ---- gravity magnitude (reference-free) ----
gErrRaw = [R.gRaw]' - 1;   gErrCal = [R.gCal]' - 1;
gStats = array2table(round([M(gErrRaw); M(gErrCal)], 5), ...
    'VariableNames', {'Mean_g','SD_g','RMS_g','MaxAbs_g'}, ...
    'RowNames', {'Uncalibrated','Calibrated'});
fprintf('--- Gravity magnitude |a| - 1 over all %d recordings ---\n', numel(R)); disp(gStats);

% ---- static run ----
t = S(:,1) - S(1,1);
rate   = (numel(t) - 1) / t(end);
yawU   = rad2deg(unwrap(deg2rad(S(:,2))));
pY     = polyfit(t/60, yawU, 1);
pP     = polyfit(t/60, S(:,3), 1);
pR     = polyfit(t/60, S(:,4), 1);
st.rate = rate; st.noiseP = std(S(:,3)); st.noiseR = std(S(:,4));
st.yawDrift = pY(1); st.pDrift = pP(1); st.rDrift = pR(1);
st.dTemp = S(end,5) - S(1,5);
fprintf('--- Static run (%.0f s) ---\n', t(end));
fprintf('Output rate        : %.1f Hz\n', st.rate);
fprintf('Noise SD pitch/roll: %.3f / %.3f deg\n', st.noiseP, st.noiseR);
fprintf('Drift pitch/roll   : %.3f / %.3f deg/min\n', st.pDrift, st.rDrift);
fprintf('Yaw drift          : %.3f deg/min  (%.1f deg after 10 min)\n', st.yawDrift, abs(st.yawDrift)*10);
fprintf('Temperature change : %.2f degC\n', st.dTemp);

writetable(T, fullfile(outDir, 'tilt_error_table.csv'));
writetable(tiltStats, fullfile(outDir, 'tilt_error_summary.csv'), 'WriteRowNames', true);
writetable(gStats, fullfile(outDir, 'gravity_error_summary.csv'), 'WriteRowNames', true);

%% ---- Figures ----
lbl = strcat(T.Pose, " ", T.Angle);
f1 = figure('Color','w','Position',[80 80 1000 450],'Name','Tilt error');
bar(categorical(lbl, lbl), [errRaw errCal]); grid on;
ylabel('Error (deg)'); legend('Uncalibrated','Calibrated','Location','best');
title(sprintf('Tilt error after A/B reversal: RMS %.2f^o \\rightarrow %.2f^o', ...
      tiltStats.RMS_deg(1), tiltStats.RMS_deg(2)));
saveas(f1, fullfile(outDir, 'fig1_tilt_error_raw_vs_cal.png'));

f2 = figure('Color','w','Position',[100 100 1000 420],'Name','Gravity magnitude');
recLbl = strcat({R.name}', "_", string([R.side]'));
bar(categorical(recLbl, recLbl), [[R.gRaw]' [R.gCal]']); hold on;
yline(1, 'k--', '1 g', 'LineWidth', 1.2); grid on;
ylim([min([[R.gRaw] [R.gCal]]) - 0.005, max([[R.gRaw] [R.gCal]]) + 0.005]);
ylabel('|a| (g)'); legend('Uncalibrated','Calibrated','Location','best');
title(sprintf('Gravity magnitude: RMS error %.4f g \\rightarrow %.4f g', ...
      gStats.RMS_g(1), gStats.RMS_g(2)));
saveas(f2, fullfile(outDir, 'fig2_gravity_magnitude.png'));

f3 = figure('Color','w','Position',[120 50 900 800],'Name','Static run');
subplot(3,1,1); plot(t, S(:,3), 'b', t, S(:,4), 'g'); grid on;
legend('Pitch','Roll'); ylabel('deg'); title('Static run: calibrated pitch and roll');
subplot(3,1,2); plot(t, yawU, 'r', t, polyval(pY, t/60), 'k--', 'LineWidth', 1.1); grid on;
legend('Yaw', sprintf('Fit %.2f deg/min', st.yawDrift)); ylabel('deg');
title('Yaw drift (gyro only, bias removed at start-up)');
subplot(3,2,5); histogram(S(:,3) - mean(S(:,3)), 30); grid on;
xlabel('deg'); title(sprintf('Pitch noise \\sigma = %.3f^o', st.noiseP));
subplot(3,2,6); histogram(S(:,4) - circMean(S(:,4)), 30); grid on;
xlabel('deg'); title(sprintf('Roll noise \\sigma = %.3f^o', st.noiseR));
saveas(f3, fullfile(outDir, 'fig3_static_run.png'));

%% ================= (f) EVALUATION =================
E = {
 'Calibrated tilt RMS error (deg)',   sprintf('<= %g', obj.tiltRMS_deg),  tiltStats.RMS_deg(2), tiltStats.RMS_deg(2) <= obj.tiltRMS_deg
 'Calibration reduces tilt RMS',      '< uncalibrated',                    tiltStats.RMS_deg(2), tiltStats.RMS_deg(2) <  tiltStats.RMS_deg(1)
 'Calibrated |a| RMS error (g)',      sprintf('<= %g', obj.gRMS_g),       gStats.RMS_g(2),      gStats.RMS_g(2) <= obj.gRMS_g
 'Calibration reduces |a| RMS',       '< uncalibrated',                    gStats.RMS_g(2),      gStats.RMS_g(2) <  gStats.RMS_g(1)
 'Pitch noise SD (deg)',              sprintf('<= %g', obj.noise_deg),    st.noiseP,            st.noiseP <= obj.noise_deg
 'Roll noise SD (deg)',               sprintf('<= %g', obj.noise_deg),    st.noiseR,            st.noiseR <= obj.noise_deg
 'Yaw drift (deg/min)',               sprintf('<= %g', obj.yawDrift_dpm), abs(st.yawDrift),     abs(st.yawDrift) <= obj.yawDrift_dpm
 'Output rate (Hz)',                  sprintf('>= %g', obj.minRate_Hz),   st.rate,              st.rate >= obj.minRate_Hz
};
evalT = cell2table(E, 'VariableNames', {'Criterion','Target','Measured','Pass'});
evalT.Measured = round(evalT.Measured, 4);
evalT.Result = repmat("FAIL", height(evalT), 1); evalT.Result(evalT.Pass) = "PASS";
evalT.Pass = [];
fprintf('\n--- (f) Evaluation against objectives ---\n'); disp(evalT);
writetable(evalT, fullfile(outDir, 'evaluation_table.csv'));

%% ================= (g) DRAFT CONCLUSION =================
fprintf('\n--- (g) Draft conclusion (rewrite in your own words) ---\n');
fprintf(['Applying the six-position calibration changed the reversal-averaged tilt RMS error from ' ...
         '%.2f to %.2f deg and the gravity-magnitude RMS error from %.4f to %.4f g. '], ...
         tiltStats.RMS_deg(1), tiltStats.RMS_deg(2), gStats.RMS_g(1), gStats.RMS_g(2));
fprintf(['The table/box slope estimated by the A/B reversal was up to %.2f deg, i.e. comparable to ' ...
         'the sensor error, which is why the reversal method was needed. '], max(abs(T.Surface_tilt)));
fprintf(['At rest the pitch/roll noise was %.3f/%.3f deg and pitch/roll did not drift (%.3f/%.3f deg/min), ' ...
         'because gravity corrects the gyro. Yaw drifted %.2f deg/min because the MPU-6500 has no ' ...
         'magnetometer, so yaw is only suitable for short-term relative rotation.\n'], ...
         st.noiseP, st.noiseR, st.pDrift, st.rDrift, st.yawDrift);
diary off
fprintf('\nAll results saved in folder: %s\n', outDir);

%% ================= LOCAL FUNCTIONS =================
function D = recordIMU(board, dur, settle)
pause(settle);
flush(board);
D = nan(ceil(dur * 25) + 50, 9);
k = 0; t0 = tic;
fprintf('      recording %g s ... ', dur);
while toc(t0) < dur
    d = getData(board, "all");
    if numel(d) < 8, continue; end
    k = k + 1;
    if k > size(D,1), D = [D; nan(100,9)]; end %#ok<AGROW>
    D(k,:) = [toc(t0) d(1:8)];
end
D = D(1:k,:);
fprintf('%d samples\n', k);
beep;                                   % recording finished
end

function waitUser(msg)
% Click-to-continue dialog. Unlike input(), extra Enter presses made while
% recording cannot be buffered and skip the next pose.
answer = questdlg(msg, 'IMU calibration verification', 'Record', 'Cancel', 'Record');
if ~strcmp(answer, 'Record')
    error('Verification cancelled by user.');
end
drawnow;
end

function m = circMean(a)
m = rad2deg(angle(mean(exp(1i * deg2rad(a(:))))));
end

function s = circStd(a)
s = std(wrap180(a(:) - circMean(a)));
end

function x = wrap180(x)
x = mod(x + 180, 360) - 180;
end
