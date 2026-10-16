%% IMU live visualisation + data logging  (RSE4501 Mini Project 2, Part 1)
% Reads yaw/pitch/roll/temp from the Arduino Leonardo via getData.m,
% shows a 3D board that follows the sensor, plots the angles live,
% and saves everything to CSV + MAT when you close the figure or time runs out.
%
% Requires MATLAB R2019b+ (serialport).  Close the Arduino Serial Monitor /
% Plotter first - only one program can use the COM port at a time.

clear; clc; close all;

%% ---- Settings ----
disp("Available ports:"); disp(serialportlist("available"));
port    = "COM5";        % <-- CHANGE to your Leonardo's port
baud    = 115200;
maxTime = 120;           % seconds; close the figure to stop earlier
window  = 20;            % seconds shown in the time plot

%% ---- Open serial port ----
board = serialport(port, baud, "Timeout", 2);
configureTerminator(board, "CR/LF");
setDTR(board, true);     % Leonardo only transmits when the port is "open"
pause(0.5);
flush(board);            % discard stale data

%% ---- Figure: 3D board ----
fig = figure('Name', 'IMU live', 'Color', 'w', 'Position', [100 100 1200 500]);

ax3 = subplot(1, 2, 1); hold(ax3, 'on'); grid(ax3, 'on');
% World frame is NED: X = North (initial forward), Y = East, Z = Down.
axis(ax3, [-3 3 -3 3 -2 2]);
daspect(ax3, [1 1 1]);
% Oblique camera: BEHIND-LEFT and ABOVE the board, so all three axes are
% visible. Screen-up = -Z (physical up), so the view is not mirrored.
% (Use the rotate icon in the axes toolbar to change the view if you like.)
ax3.CameraTarget    = [0 0 0];
ax3.CameraPosition  = [-15 -10 -10];  % behind (-X), left (-Y), above (-Z)
ax3.CameraUpVector  = [0 0 -1];       % up on screen = -Z (NED down is +Z)
ax3.CameraViewAngle = 22;
% Fixed world reference axes (dashed): North, East, Down
line(ax3, [0 2.8], [0 0], [0 0], 'Color', [1 0.6 0.6], 'LineStyle', '--');
line(ax3, [0 0], [0 2.8], [0 0], 'Color', [0.6 0.9 0.6], 'LineStyle', '--');
line(ax3, [0 0], [0 0], [0 1.9], 'Color', [0.6 0.6 1], 'LineStyle', '--');
text(ax3, 2.9, 0, 0, 'N', 'Color', [0.8 0.3 0.3]);
text(ax3, 0, 2.9, 0, 'E', 'Color', [0.3 0.6 0.3]);
text(ax3, 0, 0, 2.0, 'D', 'Color', [0.3 0.3 0.8]);
xlabel(ax3, 'X North (fwd)'); ylabel(ax3, 'Y East (right)'); zlabel(ax3, 'Z Down');
tf = hgtransform('Parent', ax3);
[V, F] = boxMesh(4, 2.5, 0.3);
patch('Vertices', V, 'Faces', F, 'FaceColor', [0.2 0.35 0.8], ...
      'EdgeColor', 'k', 'FaceAlpha', 0.5, 'Parent', tf);   % see-through so Z-down axis is visible
line('XData', [0 3], 'YData', [0 0], 'ZData', [0 0],   'Color', 'r', 'LineWidth', 3, 'Parent', tf); % body X forward
line('XData', [0 0], 'YData', [0 2], 'ZData', [0 0],   'Color', 'g', 'LineWidth', 3, 'Parent', tf); % body Y right
line('XData', [0 0], 'YData', [0 0], 'ZData', [0 1.5], 'Color', 'b', 'LineWidth', 3, 'Parent', tf); % body Z down
text(3.2, 0, 0, 'X fwd',   'Color', 'r', 'FontWeight', 'bold', 'Parent', tf);
text(0, 2.2, 0, 'Y right', 'Color', [0 0.6 0], 'FontWeight', 'bold', 'Parent', tf);
text(0, 0, 1.7, 'Z down',  'Color', 'b', 'FontWeight', 'bold', 'Parent', tf);
ttl = title(ax3, 'Waiting for data...');
% Press any key in the figure to zero the displayed yaw (point board at screen first)
yawOff = 0;
setappdata(fig, 'zeroYaw', false);
fig.KeyPressFcn = @(src, evt) setappdata(src, 'zeroYaw', true);

%% ---- Figure: time series (colours as in handout: yaw red, pitch blue, roll green) ----
axT = subplot(1, 2, 2); hold(axT, 'on'); grid(axT, 'on');
hY = plot(axT, NaN, NaN, 'r', 'LineWidth', 1.2);
hP = plot(axT, NaN, NaN, 'b', 'LineWidth', 1.2);
hR = plot(axT, NaN, NaN, 'g', 'LineWidth', 1.2);
legend(axT, 'Yaw', 'Pitch', 'Roll', 'Location', 'northwest');
xlabel(axT, 'Time (s)'); ylabel(axT, 'Angle (deg)'); ylim(axT, [-180 180]);
title(axT, 'Euler angles (NED)');

%% ---- Acquisition loop ----
imuLog = nan(maxTime * 30, 5);   % [t yaw pitch roll temp], grows if needed
k  = 0;
t0 = tic;
while ishandle(fig) && toc(t0) < maxTime
    % read every packet waiting in the buffer (no samples lost)
    while board.NumBytesAvailable > 0
        d = getData(board);
        if isempty(d), break; end
        k = k + 1;
        if k > size(imuLog, 1), imuLog = [imuLog; nan(1000, 5)]; end %#ok<AGROW>
        imuLog(k, :) = [toc(t0) d];
    end

    if k > 0 && ishandle(fig)
        if getappdata(fig, 'zeroYaw')          % key pressed -> zero yaw display
            yawOff = imuLog(k, 2);
            setappdata(fig, 'zeroYaw', false);
        end
        yaw   = mod(imuLog(k, 2) - yawOff + 180, 360) - 180;  % display only
        pitch = imuLog(k, 3); roll = imuLog(k, 4);
        % NED body->world rotation (ZYX Euler): R = Rz(yaw) * Ry(pitch) * Rx(roll)
        tf.Matrix = makehgtform('zrotate', deg2rad(yaw), ...
                                'yrotate', deg2rad(pitch), ...
                                'xrotate', deg2rad(roll));
        ttl.String = sprintf('Yaw %6.1f   Pitch %6.1f   Roll %6.1f   (%.1f ^oC)   [any key = zero yaw]', ...
                             yaw, pitch, roll, imuLog(k, 5));
        tt = imuLog(1:k, 1);
        set(hY, 'XData', tt, 'YData', imuLog(1:k, 2));
        set(hP, 'XData', tt, 'YData', imuLog(1:k, 3));
        set(hR, 'XData', tt, 'YData', imuLog(1:k, 4));
        xlim(axT, [max(0, tt(end) - window), max(window, tt(end))]);
    end
    drawnow limitrate;
end

%% ---- Close port and save ----
clear board
imuLog = imuLog(1:k, :);
if k == 0
    warning('No data received. Check port name, baud rate, and that the Serial Monitor is closed.');
    return
end
T = array2table(imuLog, 'VariableNames', {'t_s', 'yaw_deg', 'pitch_deg', 'roll_deg', 'temp_C'});
fname = "imu_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
writetable(T, fname + ".csv");
save(fname + ".mat", 'T');
fprintf('Saved %d samples (%.1f s, ~%.1f Hz) to %s.csv\n', ...
        k, imuLog(end, 1), (k - 1) / imuLog(end, 1), fname);

%% ---- Local function ----
function [V, F] = boxMesh(L, W, H)
% Rectangular box centred at origin (L along x, W along y, H along z)
x = [-1 1] * L / 2; y = [-1 1] * W / 2; z = [-1 1] * H / 2;
V = [x(1) y(1) z(1); x(2) y(1) z(1); x(2) y(2) z(1); x(1) y(2) z(1);
     x(1) y(1) z(2); x(2) y(1) z(2); x(2) y(2) z(2); x(1) y(2) z(2)];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end
