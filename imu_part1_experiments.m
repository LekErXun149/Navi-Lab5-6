%% RSE4501 Mini Project 2 - Part 1 (d)-(g)
%  (d) Conduct experiments   (e) Analyse data   (f) Evaluate vs objectives
%  (g) Draw conclusions
%
%  Hardware : Arduino Leonardo + MPU-9250 board (MPU-6500 chip), lying FLAT
%  Arduino  : MPU6500_Leonardo_MATLAB.ino  (NED: X fwd, Y right, Z down;
%             roll+ right side down, pitch+ nose up, yaw+ clockwise)
%  Needs    : getData.m in the same folder, MATLAB R2019b or newer
%  Close the Arduino Serial Monitor / Plotter before running.
%
%  Experiments
%   1. Static test   : board still -> noise, offset, yaw drift, sample rate
%   2. Tilt accuracy : known pitch & roll angles -> error, scale factor, cross-axis
%   3. Yaw rotation  : rotate 90 deg and back -> heading change accuracy, return error
%
%  Set RUN_EXPERIMENTS = false to re-analyse a saved .mat without hardware.

clear; clc; close all;

%% ================= SETTINGS =================
RUN_EXPERIMENTS = true;
port = "COM5";                 % <-- change to your Leonardo port
baud = 115200;

staticDur = 120;               % s   Exp 1 duration
tiltDur   = 5;                 % s   per tilt position
tiltRefs  = [-45 -30 -15 0 15 30 45];   % deg, reference angles
yawDur    = 5;                 % s   per yaw position
yawStep   = 90;                % deg rotation in Exp 3
yawReps   = 3;                 % repetitions of Exp 3

% ---- Objectives (targets). Edit and justify these in your report. ----
obj.minRate_Hz      = 15;      % output rate
obj.maxNoise_deg    = 0.5;     % std of pitch/roll at rest
obj.maxYawDrift_dpm = 5;       % yaw drift at rest, deg/min
obj.maxTiltRMS_deg  = 2;       % RMS tilt error (offset-corrected)
obj.maxCross_deg    = 2;       % off-axis change during single-axis tilt
obj.maxYawErr_deg   = 5;       % error of measured 90 deg rotation / return

tag    = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outDir = "results_" + tag;
mkdir(outDir);

%% ================= (d) EXPERIMENTS =================
if RUN_EXPERIMENTS
    board = serialport(port, baud, "Timeout", 2);
    configureTerminator(board, "CR/LF");
    setDTR(board, true);
    pause(0.5);
    if isempty(getData(board))
        error('No data from Arduino. Check port name and close the Serial Monitor.');
    end

    % ---------- Experiment 1: static ----------
    fprintf('\n=== EXPERIMENT 1: Static test (%d s) ===\n', staticDur);
    input('Lay the board FLAT on the table and do not touch it. Press Enter to start...', 's');
    S = recordIMU(board, staticDur, 'static');

    % ---------- Experiment 2: tilt accuracy ----------
    fprintf('\n=== EXPERIMENT 2: Tilt accuracy ===\n');
    fprintf('Use a protractor, a wedge or a phone inclinometer app as the reference.\n');
    tilt.pitch = runTilt(board, 'pitch', tiltRefs, tiltDur, ...
        'PITCH %+d deg  (NED: positive = RAISE the FRONT / VCC end (nose up); negative = lower it)');
    tilt.roll  = runTilt(board, 'roll', tiltRefs, tiltDur, ...
        'ROLL %+d deg  (NED: positive = LOWER the RIGHT side (right wing down); negative = raise it)');

    % ---------- Experiment 3: yaw rotation ----------
    fprintf('\n=== EXPERIMENT 3: Yaw rotation (%d deg, %d reps) ===\n', yawStep, yawReps);
    fprintf('Keep the board flat. Mark its start position/edge on the table.\n');
    for r = 1:yawReps
        input(sprintf('\nRep %d: board at START mark. Press Enter...', r), 's');
        yawT(r).start  = recordIMU(board, yawDur, 'start'); %#ok<SAGROW>
        input(sprintf('Rotate %d deg CLOCKWISE (seen from above, NED positive yaw), then press Enter...', yawStep), 's');
        yawT(r).turned = recordIMU(board, yawDur, 'turned'); %#ok<SAGROW>
        input('Rotate back to the START mark, then press Enter...', 's');
        yawT(r).back   = recordIMU(board, yawDur, 'back'); %#ok<SAGROW>
    end

    clear board
    dataFile = fullfile(outDir, "imu_part1_data_" + tag + ".mat");
    save(dataFile, 'S', 'tilt', 'yawT', 'tiltRefs', 'yawStep', 'staticDur');
    fprintf('\nRaw data saved to %s\n', dataFile);
else
    [f, p] = uigetfile('imu_part1_data_*.mat', 'Select saved experiment data'); %#ok<UNRCH>
    if isequal(f, 0), return; end
    L = load(fullfile(p, f));
    S = L.S; tilt = L.tilt; yawT = L.yawT; yawStep = L.yawStep;
end

diary(fullfile(outDir, 'results_summary.txt'));   % log all printed results

%% ================= (e) ANALYSIS =================
fprintf('\n================ (e) ANALYSIS ================\n');

% ---------- Exp 1: static ----------
t      = S(:, 1) - S(1, 1);
yawS   = unwrapDeg(S(:, 2));
pitchS = S(:, 3);
rollS  = S(:, 4);
tempS  = S(:, 5);

res.rate   = (numel(t) - 1) / t(end);
res.noise  = [std(pitchS) std(rollS)];
res.offset = [mean(pitchS) mean(rollS)];
pY = polyfit(t / 60, yawS, 1);    res.yawDrift   = pY(1);   % deg/min
pP = polyfit(t / 60, pitchS, 1);  res.pitchDrift = pP(1);
pR = polyfit(t / 60, rollS, 1);   res.rollDrift  = pR(1);
res.dTemp  = tempS(end) - tempS(1);

fprintf('\n--- Experiment 1: Static (%.0f s, %d samples) ---\n', t(end), numel(t));
fprintf('Sample rate         : %.1f Hz\n', res.rate);
fprintf('Pitch mean / std    : %7.3f / %.3f deg\n', res.offset(1), res.noise(1));
fprintf('Roll  mean / std    : %7.3f / %.3f deg\n', res.offset(2), res.noise(2));
fprintf('Drift pitch/roll/yaw: %.3f / %.3f / %.3f deg/min\n', res.pitchDrift, res.rollDrift, res.yawDrift);
fprintf('Temperature change  : %.2f degC (%.2f -> %.2f)\n', res.dTemp, tempS(1), tempS(end));

fig1 = figure('Color', 'w', 'Position', [100 50 900 850], 'Name', 'Exp 1 Static');
subplot(4, 1, 1);
plot(t, pitchS, 'b', t, rollS, 'g'); grid on;
legend('Pitch', 'Roll'); ylabel('deg'); title('Exp 1: Pitch and roll at rest');
subplot(4, 1, 2);
plot(t, yawS, 'r', t, polyval(pY, t / 60), 'k--', 'LineWidth', 1.2); grid on;
legend('Yaw', sprintf('Linear fit: %.2f deg/min', res.yawDrift)); ylabel('deg');
title('Exp 1: Yaw drift (gyro integration, no magnetometer)');
subplot(4, 1, 3);
plot(t, tempS, 'Color', [1 0.5 0]); grid on;
xlabel('Time (s)'); ylabel('^oC'); title('Exp 1: Sensor temperature');
subplot(4, 2, 7);
histogram(pitchS - mean(pitchS), 30); grid on;
xlabel('deviation (deg)'); title(sprintf('Pitch noise, \\sigma = %.3f deg', res.noise(1)));
subplot(4, 2, 8);
histogram(rollS - mean(rollS), 30); grid on;
xlabel('deviation (deg)'); title(sprintf('Roll noise, \\sigma = %.3f deg', res.noise(2)));
saveas(fig1, fullfile(outDir, 'fig1_static.png'));

% ---------- Exp 2: tilt ----------
Ap = analyseTilt(tilt.pitch, 2, 3);   % columns of mean: [yaw pitch roll]
Ar = analyseTilt(tilt.roll,  3, 2);

fprintf('\n--- Experiment 2: Tilt accuracy ---\n');
fprintf('PITCH test\n');
disp(table(Ap.ref, Ap.meas, Ap.errRaw, Ap.errCorr, ...
     'VariableNames', {'Ref_deg', 'Measured_deg', 'Error_raw', 'Error_offsetCorr'}));
fprintf('ROLL test\n');
disp(table(Ar.ref, Ar.meas, Ar.errRaw, Ar.errCorr, ...
     'VariableNames', {'Ref_deg', 'Measured_deg', 'Error_raw', 'Error_offsetCorr'}));
fprintf('%-6s %8s %9s %9s %8s %8s %9s %10s\n', 'Axis', 'Offset', 'RMS raw', 'RMS corr', ...
        'Max err', 'Scale', 'R^2', 'Cross-axis');
fprintf('%-6s %8.2f %9.2f %9.2f %8.2f %8.3f %9.5f %10.2f\n', 'Pitch', ...
        Ap.offset, Ap.rmsRaw, Ap.rmsCorr, Ap.maxCorr, Ap.scale, Ap.R2, Ap.cross);
fprintf('%-6s %8.2f %9.2f %9.2f %8.2f %8.3f %9.5f %10.2f\n', 'Roll', ...
        Ar.offset, Ar.rmsRaw, Ar.rmsCorr, Ar.maxCorr, Ar.scale, Ar.R2, Ar.cross);

fig2 = figure('Color', 'w', 'Position', [150 50 1000 750], 'Name', 'Exp 2 Tilt');
As = {Ap, Ar}; nm = {'Pitch', 'Roll'};
for i = 1:2
    A = As{i};
    subplot(2, 2, i);
    plot(A.ref, A.ref, 'k--', A.ref, A.meas, 'o-', 'LineWidth', 1.5); grid on;
    xlabel('Reference (deg)'); ylabel('Measured (deg)');
    legend('Ideal', 'Measured', 'Location', 'northwest');
    title(sprintf('%s: scale %.3f, R^2 = %.4f', nm{i}, A.scale, A.R2));
    subplot(2, 2, i + 2);
    bar(A.ref, [A.errRaw A.errCorr]); grid on;
    xlabel('Reference (deg)'); ylabel('Error (deg)');
    legend('Raw', 'Offset-corrected', 'Location', 'best');
    title(sprintf('%s error (RMS corrected %.2f deg)', nm{i}, A.rmsCorr));
end
saveas(fig2, fullfile(outDir, 'fig2_tilt.png'));

% ---------- Exp 3: yaw ----------
nR = numel(yawT);
Y.turn = nan(nR, 1); Y.ret = nan(nR, 1);
for r = 1:nR
    a0 = circMean(yawT(r).start(:, 2));
    a1 = circMean(yawT(r).turned(:, 2));
    a2 = circMean(yawT(r).back(:, 2));
    Y.turn(r) = wrap180(a1 - a0);
    Y.ret(r)  = wrap180(a2 - a0);
end
Y.err     = Y.turn - yawStep;
Y.meanErr = mean(Y.err);
Y.maxErr  = max(abs(Y.err));
Y.maxRet  = max(abs(Y.ret));

fprintf('\n--- Experiment 3: Yaw rotation (%d deg) ---\n', yawStep);
disp(table((1:nR)', Y.turn, Y.err, Y.ret, ...
     'VariableNames', {'Rep', 'Measured_rotation', 'Error_deg', 'Return_error_deg'}));
fprintf('Mean rotation %.2f +/- %.2f deg, max |error| %.2f deg, max |return error| %.2f deg\n', ...
        mean(Y.turn), std(Y.turn), Y.maxErr, Y.maxRet);

fig3 = figure('Color', 'w', 'Position', [200 100 900 400], 'Name', 'Exp 3 Yaw');
subplot(1, 2, 1);
bar(Y.turn); hold on; yline(yawStep, 'r--', 'Target', 'LineWidth', 1.5); grid on;
xlabel('Repetition'); ylabel('Measured rotation (deg)'); title('Exp 3: Measured yaw rotation');
subplot(1, 2, 2);
bar([Y.err Y.ret]); grid on;
legend('Rotation error', 'Return-to-start error', 'Location', 'best');
xlabel('Repetition'); ylabel('deg'); title('Exp 3: Yaw errors');
saveas(fig3, fullfile(outDir, 'fig3_yaw.png'));

%% ================= (f) EVALUATION vs OBJECTIVES =================
fprintf('\n================ (f) EVALUATION ================\n');
E = {
 'Output sample rate (Hz)',          sprintf('>= %g', obj.minRate_Hz),      res.rate,           res.rate >= obj.minRate_Hz;
 'Pitch noise at rest, std (deg)',   sprintf('<= %g', obj.maxNoise_deg),    res.noise(1),       res.noise(1) <= obj.maxNoise_deg;
 'Roll noise at rest, std (deg)',    sprintf('<= %g', obj.maxNoise_deg),    res.noise(2),       res.noise(2) <= obj.maxNoise_deg;
 'Yaw drift at rest (deg/min)',      sprintf('<= %g', obj.maxYawDrift_dpm), abs(res.yawDrift),  abs(res.yawDrift) <= obj.maxYawDrift_dpm;
 'Pitch tilt RMS error (deg)',       sprintf('<= %g', obj.maxTiltRMS_deg),  Ap.rmsCorr,         Ap.rmsCorr <= obj.maxTiltRMS_deg;
 'Roll tilt RMS error (deg)',        sprintf('<= %g', obj.maxTiltRMS_deg),  Ar.rmsCorr,         Ar.rmsCorr <= obj.maxTiltRMS_deg;
 'Cross-axis in pitch test (deg)',   sprintf('<= %g', obj.maxCross_deg),    Ap.cross,           Ap.cross <= obj.maxCross_deg;
 'Cross-axis in roll test (deg)',    sprintf('<= %g', obj.maxCross_deg),    Ar.cross,           Ar.cross <= obj.maxCross_deg;
 'Yaw 90 deg rotation error (deg)',  sprintf('<= %g', obj.maxYawErr_deg),   Y.maxErr,           Y.maxErr <= obj.maxYawErr_deg;
 'Yaw return-to-start error (deg)',  sprintf('<= %g', obj.maxYawErr_deg),   Y.maxRet,           Y.maxRet <= obj.maxYawErr_deg;
};
evalT = cell2table(E, 'VariableNames', {'Criterion', 'Target', 'Measured', 'Pass'});
evalT.Result = repmat("FAIL", height(evalT), 1);
evalT.Result(evalT.Pass) = "PASS";
evalT.Pass = [];
evalT.Measured = round(evalT.Measured, 3);
disp(evalT);
writetable(evalT, fullfile(outDir, 'evaluation_table.csv'));

%% ================= (g) CONCLUSION (draft - rewrite in your own words) =================
fprintf('\n================ (g) CONCLUSION (draft) ================\n');
nPass = sum(evalT.Result == "PASS");
fprintf('%d of %d objectives were met.\n\n', nPass, height(evalT));

tiltOK = Ap.rmsCorr <= obj.maxTiltRMS_deg && Ar.rmsCorr <= obj.maxTiltRMS_deg;
fprintf(['1. Tilt: after removing the mounting offset (pitch %.2f deg, roll %.2f deg), the RMS error ' ...
         'was %.2f deg (pitch) and %.2f deg (roll) over %+d to %+d deg, with scale factors of ' ...
         '%.3f and %.3f (R^2 = %.4f / %.4f). '], Ap.offset, Ar.offset, Ap.rmsCorr, Ar.rmsCorr, ...
         min(Ap.ref), max(Ap.ref), Ap.scale, Ar.scale, Ap.R2, Ar.R2);
if tiltOK
    fprintf(['The complementary filter gives accurate, drift-free pitch and roll, because the ' ...
             'accelerometer continuously corrects the gyro using gravity.\n\n']);
else
    fprintf(['Tilt accuracy did not meet the target. Likely causes: uncertainty in the reference ' ...
             'angles, the board moving during recording, or misalignment of the board on the surface.\n\n']);
end

fprintf(['2. Noise: the static standard deviation was %.3f deg (pitch) and %.3f deg (roll). ' ...
         'Pitch/roll drift was negligible (%.3f / %.3f deg/min).\n\n'], ...
         res.noise(1), res.noise(2), res.pitchDrift, res.rollDrift);

fprintf(['3. Yaw: yaw drifted at %.2f deg/min at rest, i.e. about %.0f deg after 10 min. ' ...
         'The chip is an MPU-6500 (WHO_AM_I = 0x70) with no magnetometer, so yaw comes only from ' ...
         'integrating the gyro and residual bias cannot be corrected. A %d deg rotation was measured as ' ...
         '%.1f +/- %.1f deg, with a return error of up to %.1f deg. Yaw is therefore usable only for ' ...
         'short-term relative heading changes, not absolute heading.\n\n'], ...
         res.yawDrift, abs(res.yawDrift) * 10, yawStep, mean(Y.turn), std(Y.turn), Y.maxRet);

fprintf(['4. Temperature changed by %.2f degC during the static test; gyro bias varies with ' ...
         'temperature and contributes to yaw drift.\n\n'], res.dTemp);

fprintf(['5. Inference: the system is suitable for tilt-based applications (gesture control, ' ...
         'balancing, posture monitoring). Applications needing absolute heading require a ' ...
         'magnetometer (genuine MPU-9250) or another external reference, and a quaternion ' ...
         'filter (Madgwick/Mahony) to avoid gimbal lock near +/-90 deg pitch.\n']);

diary off
fprintf('\nAll figures, tables and the summary are saved in folder: %s\n', outDir);

%% ================= LOCAL FUNCTIONS =================
function D = recordIMU(board, dur, label)
% Record [t yaw pitch roll temp] for dur seconds
flush(board);                      % drop data buffered while waiting for the user
D = nan(ceil(dur * 40) + 20, 5);
k = 0;
fprintf('  Recording %-8s (%g s)... ', label, dur);
t0 = tic;
while toc(t0) < dur
    d = getData(board);
    if isempty(d), continue; end
    k = k + 1;
    if k > size(D, 1), D = [D; nan(100, 5)]; end %#ok<AGROW>
    D(k, :) = [toc(t0) d];
end
D = D(1:k, :);
fprintf('%d samples\n', k);
if k < 5, warning('Very few samples recorded - check the connection.'); end
end

function R = runTilt(board, axisName, refs, dur, msgFmt)
% Step through reference angles and record each
R.ref  = refs(:);
R.raw  = cell(numel(refs), 1);
R.mean = nan(numel(refs), 3);      % [yaw pitch roll]
R.std  = nan(numel(refs), 3);
for i = 1:numel(refs)
    fprintf(['\n' msgFmt '\n'], refs(i));
    input('Set the angle, keep the board still, then press Enter to record...', 's');
    D = recordIMU(board, dur, sprintf('%s%+d', axisName, refs(i)));
    R.raw{i}     = D;
    R.mean(i, :) = mean(D(:, 2:4), 1);
    R.std(i, :)  = std(D(:, 2:4), 0, 1);
end
end

function A = analyseTilt(T, col, otherCol)
% Accuracy metrics for one tilt axis
ref  = T.ref;
meas = T.mean(:, col);
z = find(ref == 0, 1);
if isempty(z), off = 0; else, off = meas(z); end
A.ref     = ref;
A.meas    = meas;
A.offset  = off;
A.errRaw  = meas - ref;
A.errCorr = (meas - off) - ref;
A.rmsRaw  = sqrt(mean(A.errRaw .^ 2));
A.rmsCorr = sqrt(mean(A.errCorr .^ 2));
A.maxCorr = max(abs(A.errCorr));
p = polyfit(ref, meas, 1);
A.scale = p(1);
A.R2    = 1 - sum((meas - polyval(p, ref)) .^ 2) / sum((meas - mean(meas)) .^ 2);
other = T.mean(:, otherCol);
if isempty(z), o0 = other(1); else, o0 = other(z); end
A.cross = max(abs(other - o0));
end

function m = circMean(a)
% Mean of angles in degrees, safe across the +/-180 wrap
m = rad2deg(angle(mean(exp(1i * deg2rad(a)))));
end

function x = wrap180(x)
x = mod(x + 180, 360) - 180;
end

function y = unwrapDeg(x)
y = rad2deg(unwrap(deg2rad(x)));
end
