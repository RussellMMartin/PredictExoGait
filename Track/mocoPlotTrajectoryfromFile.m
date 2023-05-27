
%% mocoPlotTrajectory plots states, controls, and grfs after trackNW runs
function mocoPlotTrajectoryfromFile(saveLoc, trajA, trajB, nameA, nameB, grfA, grfB)
% Plot a MocoTrajectory. Optionally, specify a second trajectory and names
% for the trajectories.
% Pass empty character '' if you don't want to save.
% For a generic version of this function, see the utility
% osimMocoTrajectoryReport.m.
import org.opensim.modeling.*;
nFigs = 0;

%% Joint positions. 
commonName.include = {'value'};
commonName.exclude = {};
plotCategory(trajA, trajB, commonName, 'Joint Angles', {nameA, nameB})
nFigs = nFigs + 1;

%% Muscle activity 
commonName.include = {'activation'};
commonName.exclude = {};
plotCategory(trajA, trajB, commonName, 'Muscle Activity', {nameA, nameB})
nFigs = nFigs + 1;

%% Muscle force 
commonName.include = {'force'};
commonName.exclude = {'activation'};
plotCategory(trajA, trajB, commonName, 'Muscle Force', {nameA, nameB})
nFigs = nFigs + 1;


%% GRFs. 
commonName.include = {'ground'};
commonName.exclude = {};
plotCategory(grfA, grfB, commonName, 'GRF', {nameA, nameB})
nFigs = nFigs + 1;


%% Save plot. 
if ~strcmp(saveLoc,'') % if save location has been specified, save
    filename = [saveLoc,'statesAndControls.pdf'];
    if ~isfile(filename)
        for i=1:nFigs
                exportgraphics(figure(i), filename, 'Append', true);
        end
        disp(['saved ', filename])
    
    else
        disp(['WARNING ', filename, ' not saved b/c the specified save file already exists at this location'])
        now = char(datetime('now','Format','yyyy-MM-dd hh mm ss'));
        filename = [saveLoc,'statesControlsGRFs ', now,'.pdf'];
        disp(['instead saving as ', filename]);
        for i=1:nFigs
                exportgraphics(figure(i), filename, 'Append', true);
        end

    end
end

return


function plotCategory(seriesA, seriesB, commonName, topTitle, seriesNameAB)

    figure('units','normalized','outerposition',[0 0 1 1]);
    [xA, yA, namesA] = getDataFromFile(seriesA.loc, seriesA.file, commonName);
    seriesNameA = seriesNameAB{1};
    numPlots = numel(namesA);
    width = 1.5;
    
    
    plotB = false;
    if ~ischar(seriesB)
        plotB = true;
        [xB, yB, namesB] = getDataFromFile(seriesB.loc, seriesB.file, commonName);
        numPlots = max(numel(namesA), numel(namesB)); 
        seriesNameB = seriesNameAB{2};
    
        if numel(namesA) > numel(namesB)
            subplotTitles = namesA;
        else
            subplotTitles = namesB;
        end
    else
        subplotTitles = namesA;
        seriesNameB = '(none)';
    end
    
    numCols = 4;
    numRows = ceil(numPlots/numCols);
    
    for i=1:numPlots
        subplot(numRows, numCols, i)
        plotTitle = subplotTitles(i);
        % disp(['plotTitle=',plotTitle])
        
        for j = 1:numel(namesA)
            if contains(namesA(j), plotTitle) || contains(plotTitle, namesA(j))
                yA_plot = yA(:,j);
                plot(xA, yA_plot, '-r', 'linewidth', width);
                break
            end
        end
        if plotB && numel(namesB) > 0
            hold on;
            for j = 1:numel(namesB)
                if contains(namesB(j), plotTitle) || contains(plotTitle, namesB(j))
                    yB_plot = yB(:,j);
                    plot(xB, yB_plot, '--b', 'linewidth', width);
                    break
                end
            end
            hold off;
        end
        
        % title, axis labels, legend   
        xlabel('time (s)')
        if contains(subplotTitles(i), 'activation')
            ylabel('Activation(%)');
            plotTitle = regexprep(plotTitle, 'activation', '');
            plotTitle = regexprep(plotTitle, 'forceset', '');
            ylim([0,1])
        elseif contains(subplotTitles(i), 'force')
            ylabel('Force (N?)');
            plotTitle = regexprep(plotTitle, 'forceset', '');
            if contains(subplotTitles(i), 'ground')
                ylim([0,1000])
            else
                ylim([0,1])
            end
        elseif contains(subplotTitles(i), 'speed') || contains(subplotTitles(i), 'velocity')
            ylabel('Velocity (rad/s?)');
        elseif contains(subplotTitles(i), 'position') || contains(subplotTitles(i), 'angle') || contains(subplotTitles(i), 'value')
            ylabel('Angle (rad)');
            plotTitle = regexprep(plotTitle, 'value', '');
        elseif contains(subplotTitles(i), 'torque')
            ylabel('Torque(Nm?)');
            ylim([0,1])
        else
            ylabel('TODO YLABEL')
        end
        plotTitle = regexprep(plotTitle, '/', ' ');
        title(plotTitle, 'Interpreter', 'none');
    end
    sgtitle([topTitle,' (red=',seriesNameA,', blue=',seriesNameB,')'])

return

%%
function [x, y, names] = getDataFromFile(loc, file, commonName)
    [data, C, ~] = readMOTSTOTRCfiles(loc, file);
    x = data(:,1);
    y = data(:, 2:end);
    names = string(C);
    names = names(2:end);
    columnsToKeep = [];

    % if specifying a common name, get only columns containing it
    % e.g. commonName = 'ground' results in 'ground_r_vy' being returned
    if nargin > 2 
        for i=1:numel(names)
            if contains(names(i), commonName.include) && ~contains(names(i), commonName.exclude)
                columnsToKeep = [columnsToKeep, i];
            end
        end
    end
    names = names(columnsToKeep);
    y = y(:, columnsToKeep);
return
