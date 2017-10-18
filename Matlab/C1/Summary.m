%       Nicholas Mayne & Laura Petrich, University of Alberta � 2017.

%       For THESE particular settings in C_Main.java
%       If the settings are different you'll have to change the code to
%       reflect the setpoints and powers you use in your experiment

%       From C_Main.java
%       precision = 10;
% 		int setPoints[] = {50, 200, 400};	// test these angles (Deg)
% 		int powers[] = {25, 50, 100};		// test these motor powers, [25, 100]

X = [25 50 100];    % List of motor powers used in C_PID.java
precision = 10;     % Precision from C_PID.java


% Add one list for each motor power to each Kx gain parameter
% Kx{i}{j}: i = idx of powers, j = idx of setPoints
Kp = {};
Ki = {};
Kd = {};
for i = 1:size(X')
    Kp{end+1} = [];
    Ki{end+1} = [];
    Kd{end+1} = [];
end

% x(1) matches Kx{1}
% Kx{i}{j}: i = idx of X 
% Kx{2}: Power = 50
% Kx{3}: Power = 100
% Kx{x}{1}: SetPoint = 50
% Kx{x}{2}: SetPoint = 200
% Kx{x}{3}: SetPoint = 400



%   Power = 25
P25SP50
P25SP200
P25SP400
lgd = legend();
P25 = lgd.String;
[Kp{1}, Ki{1}, Kd{1}] = getPID(P25);
pause;
close 1;
close 2;

%   Power = 50
P50SP50
P50SP200
P50SP400
lgd = legend();
P50 = lgd.String;
[Kp{2}, Ki{2}, Kd{2}] = getPID(P50);
pause;
close 1;
close 2;


%   Power = 100
P100SP50
P100SP200
P100SP400
lgd = legend();
P100 = lgd.String;
[Kp{3}, Ki{3}, Kd{3}] = getPID(P100);
pause;
close 1;
close 2;


% % Create the Xx and Yx vectors for all values in Kx
% % THEN USE THESE Xx vs. Yx VECTORS TO FIT A FUNCTION FOR Kp, Ki, AND Kd
% Xp = [];
% Xi = [];
% Xd = [];
% Yp = [];
% Yi = [];
% Yd = [];
% for i  = 1:3
%     for j = 1:3
%         a = size(Kp{i}{j});
%         b = size(Ki{i}{j});
%         c = size(Kd{i}{j});
%         ra = repmat(X(i),[1,a]);
%         rb = repmat(X(i),[1,b]);
%         rc = repmat(X(i),[1,c]);
%         ra = ra(:)';
%         rb = rb(:)';
%         rc = rc(:)';
%         Xp = horzcat(Xp, ra);
%         Xi = horzcat(Xi, rb);
%         Xd = horzcat(Xd, rc);
%         Yp = horzcat(Yp, Kp{i}{j});
%         Yi = horzcat(Yi, Ki{i}{j});
%         Yd = horzcat(Yd, Kd{i}{j});
%     end
% end




% Create the Xx and Yx vectors for all values in Kx
% THEN USE THESE Xx vs. Yx VECTORS TO FIT A FUNCTION FOR Kp, Ki, AND Kd
Xp = [];
Xi = [];
Xd = [];
Yp = [];
Yi = [];
Yd = [];
for i  = 1:3
    for j = 1:size(X)
        Xp = horzcat(Xp, X(j));
        Xi = horzcat(Xi, X(j));
        Xd = horzcat(Xd, X(j));
        Yp = horzcat(Yp, Kp{i}{j});
        Yi = horzcat(Yi, Ki{i}{j});
        Yd = horzcat(Yd, Kd{i}{j});
    end
end


function [Kp, Ki, Kd] = getPID(StringArray, Precision)
    Kp = {[],[],[]};
    Ki = {[],[],[]};
    Kd = {[],[],[]};
    idx = 0;
    for string = StringArray
        idxP = strfind(string{1},'p');
        idxI = strfind(string{1},'i');
        idxD = strfind(string{1},'d');
        KpNum = str2num(string{1}(idxP+1:idxI-1));
        KiNum = str2num(string{1}(idxI+1:idxD-1));
        KdNum = str2num(string{1}(idxD+1:end));
        if isempty(KpNum)
            idx = idx + 1;
        else
            Kp{idx}(end+1) = KpNum / Precision;
            Ki{idx}(end+1) = KiNum / Precision;
            Kd{idx}(end+1) = KdNum / Precision;
        end
    end
end