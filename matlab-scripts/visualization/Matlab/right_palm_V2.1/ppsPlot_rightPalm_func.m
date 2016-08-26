%% Import data from text file.
% Script for importing data from the following text file:
%
%    E:\Dropbox\PhD_IIT\1st Year\iCubSkin\right_forearm_V1\taxels1D_learned_r_hand.ini
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2016/08/05 17:07:29

%% About 
% Script to visualize the PPS of the right palm of iCub
% Author: NGUYEN Dong Hai Phuong
% Email: phuong.nguyen@iit.it; ph17dn@gmail.com

%%
function hAxes = ppsPlot_rightPalm_func(varargin)

printToFile = 0;

%% Initialize variables.
% filename = 'E:\Dropbox\PhD_IIT\1st Year\iCubSkin\right_palm_V2.1\taxels1D_learned_r_hand.ini';
filename = 'right_palm_V2.1\taxels1D_learned_r_hand.ini';
delimiter = ' ';
startRow = 8;

%% Read columns of data as strings:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric strings to numbers.
% Replace non-numeric strings with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = dataArray{col};
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41]
    % Converts strings in the input cell array to numbers. Replaced non-numeric
    % strings with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1);
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData{row}, regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if any(numbers==',');
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'));
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric strings to numbers.
            if ~invalidThousandsSeparator;
                numbers = textscan(strrep(numbers, ',', ''), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch me
        end
    end
end


%% Allocate imported array to column variable names
r_hand = cell2mat(raw(:, 1));
% VarName2 = cell2mat(raw(:, 2));
% VarName3 = cell2mat(raw(:, 3));
% VarName4 = cell2mat(raw(:, 4));
% VarName5 = cell2mat(raw(:, 5));
% VarName6 = cell2mat(raw(:, 6));
% VarName7 = cell2mat(raw(:, 7));
% VarName8 = cell2mat(raw(:, 8));
% VarName9 = cell2mat(raw(:, 9));
% VarName10 = cell2mat(raw(:, 10));
% VarName11 = cell2mat(raw(:, 11));
% VarName12 = cell2mat(raw(:, 12));
% VarName13 = cell2mat(raw(:, 13));
% VarName14 = cell2mat(raw(:, 14));
% VarName15 = cell2mat(raw(:, 15));
% VarName16 = cell2mat(raw(:, 16));
% VarName17 = cell2mat(raw(:, 17));
% VarName18 = cell2mat(raw(:, 18));
% VarName19 = cell2mat(raw(:, 19));
% VarName20 = cell2mat(raw(:, 20));
% VarName21 = cell2mat(raw(:, 21));
% VarName22 = cell2mat(raw(:, 22));
% VarName23 = cell2mat(raw(:, 23));
% VarName24 = cell2mat(raw(:, 24));
% VarName25 = cell2mat(raw(:, 25));
% VarName26 = cell2mat(raw(:, 26));
% VarName27 = cell2mat(raw(:, 27));
% VarName28 = cell2mat(raw(:, 28));
% VarName29 = cell2mat(raw(:, 29));
% VarName30 = cell2mat(raw(:, 30));
% VarName31 = cell2mat(raw(:, 31));
% VarName32 = cell2mat(raw(:, 32));
% VarName33 = cell2mat(raw(:, 33));
% VarName34 = cell2mat(raw(:, 34));
% VarName35 = cell2mat(raw(:, 35));
% VarName36 = cell2mat(raw(:, 36));
% VarName37 = cell2mat(raw(:, 37));
% VarName38 = cell2mat(raw(:, 38));
% VarName39 = cell2mat(raw(:, 39));
% VarName40 = cell2mat(raw(:, 40));
% VarName41 = cell2mat(raw(:, 41));
numPts = 20;
d = .3/numPts;
D = -.1:d:.2-d;
xi = -.1:d/10:.2-d;
% kernel = 1/sqrt(2*pi*sigma)*exp(-()^2/(2*sigma^2)
% D = 1:20;
j=1;
for i=1:length(r_hand)
    values(i,1:length(raw)-1) = cell2mat(raw(i,2:length(raw)));
    p(i,1:(length(raw)-1)/2) = values(i,1:(length(raw)-1)/2);
    n(i,1:(length(raw)-1)/2) = values(i,(length(raw)-1)/2+1:(length(raw)-1));
    
    % Remove the component equal to NaN
    pn = p(i,:) + n(i,:);
    idZero = find(pn==0 & p(i,:)==0);
%     P(i,1:(length(raw)-1)/2) = values(i,1:(length(raw)-1)/2)./values(i,(length(raw)-1)/2+1:(length(raw)-1));
    P(i,1:(length(raw)-1)/2)= p(i,:)./(n(i,:)+p(i,:));
    P(i,idZero) = 0;
    %===================================

%     P(i,1:(length(raw)-1)/2) = values(i,(length(raw)-1)/2+1:(length(raw)-1))./values(i,1:(length(raw)-1)/2);
    if (any(p(i,:)~=0))
        figureTitle = sprintf('Taxel %ith',r_hand(i));
        [f(i,:),x] = parzen_estimation(D,P(i,:),4*d,'r',figureTitle,0);
    end
end

%% Load taxel files
loadTaxelPositions_r_hand;
taxel_pos = taxel_positions_FoR_10/1000;    %convert to m from mm
% M = NR_TAXELS;
[M,N] = size(taxel_pos);

pos0_r_h = taxel_pos;

%% Transform
matT=  [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

if (~isempty(varargin))
    if (length(varargin)>=2)
        fig = varargin{1};
        matT = varargin{2};  
    elseif (length(varargin)>=1)
        fig = varargin{1};        
    end
else
    close all;
    fig = figure(1);
    
end
for i=1:M
    taxel_pos(i,4:6) = [0 0 +1];
    
    pos1(i,:) = matT(1:3,4)+matT(1:3,1:3)*taxel_pos(i,1:3)';
    pos0n(i,:)= taxel_pos(i,4:6)+taxel_pos(i,1:3);
    pos1n(i,:)= matT(1:3,4)+matT(1:3,1:3)*pos0n(i,:)';
    taxel_pos(i,1:3) = pos1(i,:)';
    taxel_pos(i,4:6) = pos1n(i,:)' - pos1(i,:)';
    
end
T = matT
posOrigin = matT(1:3,4)';

%% Plot whole PPS from 0->0.2cm


figure(fig.Number); hold on
title('PPS of left palm taxels from 0->0.2cm (in 1st wrist FoR - FoR_{10})');
colormap autumn %flag hot

for i=1:M
    if (nnz(taxel_pos(i,:)) > 1) % it's not an all-zero row
        plot3(taxel_pos(i,1),taxel_pos(i,2),taxel_pos(i,3),'xb');
%         if (mod(i-3,12)==0)
            
            
%         end
    end
end

lNorm = 0.05;
for i=1:length(r_hand)
    if (any(f(i,:)~=0))
%         hist_map3d([0 0 0],x,f(i,find(x>0)));
        for j=1:M
            if (r_hand(i)==j-1+TAXEL_ID_OFFSET_PALM_TO_HAND)
                h = hist_map3d([taxel_pos(j,1),taxel_pos(j,2),taxel_pos(j,3)],...
                    x(70:end),-f(i,70:end));
               
                vz = [0 0 .05];
                v2 = [taxel_pos(j,4),taxel_pos(j,5),taxel_pos(j,6)];
                n = cross(vz,v2);
                angle(j) = acos(dot(vz,v2)/(norm(vz)*norm(v2)));
                rotate(h,n,angle(j)/pi*180,taxel_pos(j,1:3));
                
                text(taxel_pos(j,1),taxel_pos(j,2),taxel_pos(j,3),int2str(j-1+TAXEL_ID_OFFSET_PALM_TO_HAND),'Color','r'); 
                                                                              
                h = quiver3(taxel_pos(j,1),taxel_pos(j,2),taxel_pos(j,3),lNorm*taxel_pos(j,4),lNorm*taxel_pos(j,5),lNorm*taxel_pos(j,6));
                set(h, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 4, 'ShowArrowHead', 'on');
                
            end
        end
    end
end

xlabel('Taxel position x (m)');
% set(gca,'XDir','reverse');
ylabel('Taxel position y (m)');
zlabel('Taxel position z (m)');
% set(gca,'ZDir','reverse');
% axis equal;
hold off; grid on;
% hAxes = axes;
clear taxel_pos taxel_positions_FoR_10 pos1 pos0n pos1n angle

%% Clear temporary variables
% clearvars filename delimiter startRow formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp me;
end