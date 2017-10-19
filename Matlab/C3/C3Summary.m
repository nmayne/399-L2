%% Nicholas Mayne & Laura Petrich, University of Alberta © 2017.
%% Declare Variable
% For THESE particular settings in C_Main.java
% If the settings are different you'll have to change the code to
% reflect the setpoints and powers you use in your experiment
%
% From C_Main.java
%   prec = 10;
% 	int setPoints[] = {50, 200, 400};   // test these angles (Deg) {50, 200, 400}
% 	int powers[] = {25, 50, 100};		// test these motor powers {25, 50, 100}

% Prec from C_PID.java
prec = 10;

% List of motor powers used in C_PID.java
Xmp =  [25 50 100];

% List of test set point used in C_PID.java
Xsp = [50 200 400];

% Data Filtering
upperBound = 10;         % Maximum number of degrees above setpoint to search for good options
settlingTime50 = 500;    % Maximum permissable settling time for minimum setpoint
settlingTime200 = 1300;
settlingTime400 = 2500;
minResults = 13;         % Return a least this many good option cases
%% Fetch desired data from .m files
%   Power = 25
P25SP50
filterData(t, caseList, 25, 50, upperBound, settlingTime50, minResults)
P25SP200
filterData(t, caseList, 25, 200, upperBound, settlingTime200, minResults)
P25SP400
filterData(t, caseList, 25, 400, upperBound, settlingTime400, minResults)

lgd = legend();
P25 = lgd.String;
[K{1,1}, K{1,2}, K{1,3}] = getPID(P25, prec);
pause;
close 1;
close 2;

%   Power = 50
P50SP50
filterData(t, caseList, 50, 50, upperBound, settlingTime50, minResults)
P50SP200
filterData(t, caseList, 50, 200, upperBound, settlingTime200, minResults)
P50SP400
filterData(t, caseList, 50, 400, upperBound, settlingTime400, minResults)

lgd = legend();
P50 = lgd.String;
[K{2,1}, K{2,2}, K{2,3}] = getPID(P50, prec);
pause;
close 1;
close 2;

%   Power = 100
P100SP50
filterData(t, caseList, 100, 50, upperBound, settlingTime50, minResults)
P100SP200
filterData(t, caseList, 100, 200, upperBound, settlingTime200, minResults)
P100SP400
filterData(t, caseList, 100, 400, upperBound, settlingTime400, minResults)

lgd = legend();
P100 = lgd.String;
[K{3,1}, K{3,2}, K{3,3}] = getPID(P100, prec);
pause;
close 1;
close 2;
%% Process the data | Use the MEAN/MODE of the Good Option values from each sample
% % Create the Xx and Yx vectors for all values in Kx
% % THEN USE THESE Xx vs. Yx VECTORS TO FIT A FUNCTION FOR Kp, Ki, AND Kd
% % 
% %       A =     ______________   S
% %              /400  400  400/|  e
% %             /200  200  200/ |  t
% %            /50   50   50 /  |  P
% %           /-------------/   /  o
% %  Mp = 25  |{P}  {I}  {D}|  /  i
% %  Mp = 50  |{P}  {I}  {D}| /  n
% %  Mp = 100 |{P}  {I}  {D}|/  t
% %           ---------------  s
% %
% % Put crunched data in a 3x3x3 array, A(Mp,PID,Sp)
% % {Mp = 25/50/100, Kx = P/I/D}{Sp = 50/200/400}
% % Using some approximating function like Mean
for Mp  = 1:3
    for Kx = 1:3
        for Sp = 1:3
            A(Mp, Kx, Sp) = mean(K{Mp,Kx}{Sp}); % DO MEAN OR MODE
        end                                
    end
end

% % Get mean Kx values by set points a function of motor power
Ymp = [mean(A(1,1,:)) mean(A(1,2,:)) mean(A(1,3,:));
     mean(A(2,1,:)) mean(A(2,2,:)) mean(A(2,3,:));
     mean(A(3,1,:)) mean(A(3,2,:)) mean(A(3,3,:))];
 
% % Get mean Kx values by motor powers as a function of set point
Ysp = [mean(A(:,1,1)) mean(A(:,2,1)) mean(A(:,3,1));
     mean(A(:,1,2)) mean(A(:,2,2)) mean(A(:,3,2));
     mean(A(:,1,3)) mean(A(:,2,3)) mean(A(:,3,3))];
 
 
% % Ysp(1:3, 1) = Mean experimental P values vs set points in Xsp
% % Ysp(1:3, 2) = Mean experimental I values vs set points in Xsp
% % Ysp(1:3, 3) = Mean experimental D values vs set points in Xsp

% % Ymp(1:3, 1) = Mean experimental P values vs motor powers in Xmp
% % Ymp(1:3, 2) = Mean experimental I values vs motor powers in Xmp
% % Ymp(1:3, 3) = Mean experimental D values vs motor powers in Xmp

Yspp = Ysp(1:3, 1);
Yspi = Ysp(1:3, 2);
Yspd = Ysp(1:3, 3);

Ympp = Ymp(1:3, 1);
Ympi = Ymp(1:3, 2);
Ympd = Ymp(1:3, 3);
%% Filter data for Good Options
function filterData(t, caseList, power, min, upperBound, timeMax, minResults)
    figure(2);
    hold on;
    xlabel('Time (ms)');
    ylabel('Theta (Deg)');
    title(strcat('PID Controller | P: ', num2str(power), ' SP: ', num2str(min), ' | Good Options'));
    plot([0 timeMax], [min min], 'r--');
    legendTwo = {strcat(num2str(min), 'Deg')};
    
    results = 0;
    idx = 0;
    while results < minResults && min <= min + upperBound
        for e = t
            idx = idx + 1;
            if max(e{1}(:,2)) == min && max(e{1}(:,1)) < timeMax
                figure(2);
                plot(e{1}(:,1),e{1}(:,2));
                legendTwo{end+1} = caseList{idx};
                results = results + 1;
            end
        end
        idx = 0;
        min = min + 1;
    end
    newLegend = legend();
    newLegend = newLegend.String;
    idx = size(legendTwo);
    newLegend(end-idx(2)+1:end) = legendTwo(1:end);
    legend(newLegend);
end
%% Parse a legend into Kx values
function [Kp, Ki, Kd] = getPID(legend, Precision)
    Kp = {[],[],[]};
    Ki = {[],[],[]};
    Kd = {[],[],[]};
    idx = 0;
    for string = legend
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