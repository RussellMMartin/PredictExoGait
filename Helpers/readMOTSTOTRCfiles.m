function[data,C,tableData] =  readMOTSTOTRCfiles(path,file)

% for i = 1 : height(handles.T)

if ~isempty(path)
    
startRow = 0;
filename = [path , file];

while ~exist('data','var')   
try 
data = dlmread(filename, '\t',startRow,0);
catch   
end
 startRow = startRow + 1;
 if startRow == 1000
    disp(['WARNING: in readMOTSTOTRCfiles.m, 1000 rows have been searched ' ...
        ' and dlmread still isnt working! Filename: ', filename]);
    data = dlmread(filename, '\t',startRow,0);

 end
end
% handles.T.data{i} = data;
[~,nCols] = size(data);


fid=fopen(filename,'r');
format = '%s';
Spec = repmat(format, 1, nCols);

if strcmp(file(end-2:end), 'trc')
C = textscan(fid,Spec,1,'delimiter','\t', 'headerlines',startRow-3);
for m = 3 :3: length(C)-2
    markerName = C{m};
    C{m} = strcat(markerName,'X');
    C{m+1} = strcat(markerName,'Y');
    C{m+2} = strcat(markerName,'Z');
end




elseif strcmp(file(end-2:end), 'txt') % for V3D files
    
C = textscan(fid,Spec,1,'delimiter','\t', 'headerlines',startRow-5);
conversion = [C{:}];
[~,~,c] = unique(conversion,'stable');
h = accumarray(c, 1);                              % Count Occurrences to check if same variables names exist

C{1} = 'Frames';
if max(h)> 3
    trial = 1;
for m = 2 :3: length(C)-2
    markerName = C{m};
    C{m} = strcat(markerName,'X', num2str(trial));
    C{m+1} = strcat(markerName,'Y',num2str(trial));
    C{m+2} = strcat(markerName,'Z',num2str(trial));
    trial = trial +1;
end
else 
    for m = 2 :3: length(C)-2
    markerName = C{m};
    C{m} = strcat(markerName,'X');
    C{m+1} = strcat(markerName,'Y');
    C{m+2} = strcat(markerName,'Z');
    end
end





else
C = textscan(fid,Spec,1,'delimiter','\t', 'headerlines',startRow-2);
end

fclose(fid);





for j = 1 : length(C)
     if strcmp(C{j},'Time')
        C{j} = {'time'};
    end
    
    if strcmp(C{j},'')
        C{j} = {['Unknow_', num2str(j)]};
    end
    
    if contains(C{j},'#')
        disp(j)
        C{j} = strrep(C{j},'#','');
    end
        
    
end
% handles.T.variableNames{i} =horzcat(C{:});
C =horzcat(C{:});


tableData = array2table(data,'VariableNames', C);

% set(handles.(['data', num2str(i)]),'String',handles.T.variableNames{i})

else 
    data= [];
    C= [];
    tableData = [];
end


% clearvars data
% end

