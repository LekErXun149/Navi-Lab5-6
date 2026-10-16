function data = getData(board, mode)
% GETDATA  Read one IMU packet from the Arduino (MATLAB serialport version).
%
%   data = getData(board)          -> [yaw pitch roll temp]            (1x4)
%   data = getData(board, "all")   -> every value in the packet        (1xN)
%
%   Packets sent by the Arduino, one per line:
%       normal : 111,yaw,pitch,roll,temp,FFF
%       verify : 111,yaw,pitch,roll,temp,pitch_raw,roll_raw,g_cal,g_raw,FFF
%   Start header '111', comma-separated values, end header 'FFF'.
%   Lines that are not packets (header row, '# ...' messages) are skipped.
%
%   Returns [] if no valid packet arrived before board.Timeout.
%
%   board : serialport object, e.g.
%       board = serialport("COM5",115200,"Timeout",2);
%       configureTerminator(board,"CR/LF");

if nargin < 2, mode = "first4"; end
data = [];
for attempt = 1:5                       % first line may be partial -> retry
    line = readline(board);             % waits up to board.Timeout
    if isempty(line)
        return                          % timeout, no data
    end
    parts = strsplit(strtrim(char(line)), ',');
    if numel(parts) >= 6 && strcmp(parts{1}, '111') && strcmp(parts{end}, 'FFF')
        vals = str2double(parts(2:end-1));
        if ~any(isnan(vals))
            if mode == "all"
                data = vals;
            else
                data = vals(1:4);
            end
            return
        end
    end
    % otherwise: not a data packet -> try next line
end
end
