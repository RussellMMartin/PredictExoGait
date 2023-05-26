
function mocoPlotTrajectory(saveLoc, trajA, trajB, nameA, nameB)
% Plot a MocoTrajectory. Optionally, specify a second trajectory and names
% for the trajectories.
% Pass empty character '' if you don't want to save.
% For a generic version of this function, see the utility
% osimMocoTrajectoryReport.m.
import org.opensim.modeling.*;

if ischar(trajA)
    trajA = MocoTrajectory(trajA);
end
if nargin > 2 && ischar(trajB)
    trajB = MocoTrajectory(trajB);
end

width = 0.5;
%% States.
figure('units','normalized','outerposition',[0 0 1 1]);
stateNames = trajA.getStateNames();
numStates = stateNames.size();
dim = sqrt(numStates);
if dim == ceil(dim)
    numRows = dim;
    numCols = dim;
else
    numCols = min(numStates, 5);
    numRows = floor(numStates / 5);
    if mod(numStates, 5) ~= 0
        numRows = numRows + 1;
    end
end
for i = 0:numStates-1
    subplot(numRows, numCols, i+1);
    plot(trajA.getTimeMat(), ...
         trajA.getStateMat(stateNames.get(i)), '-r', ...
         'linewidth', width);
    if nargin > 2
        hold on
        plot(trajB.getTimeMat(), ...
             trajB.getStateMat(stateNames.get(i)), '--b', ...
             'linewidth', width);
        hold off
    end
    
    stateName = char(stateNames.get(i));
    title(stateName(11:end), 'Interpreter', 'none')
    xlabel('time (s)')
    if contains(stateName, 'value')
        ylabel('position (rad)')
    elseif contains(stateName, 'speed')
        ylabel('speed (rad/s)')
    elseif contains(stateName, 'activation')
        ylabel('activation (-)')
        ylim([0, 1])
    end
    if i == 0 && nargin > 2
        if nargin == 5
            legend(nameA, nameB);
        else
            legend('A', 'B');
        end
    end
end
sgtitle('States')

%% Controls.
figure('units','normalized','outerposition',[0 0 1 1]);
controlNames = trajA.getControlNames();
numControls = controlNames.size();
dim = sqrt(numControls);
if dim == ceil(dim)
    numRows = dim;
    numCols = dim;
else
    numCols = min(numControls, 4);
    numRows = floor(numControls / 4);
    if mod(numControls, 4) ~= 0
        numRows = numRows + 1;
    end
end
for i = 0:numControls-1
    subplot(numRows, numCols, i+1);
    yA = trajA.getControlMat(controlNames.get(i));
    plot(trajA.getTimeMat(), yA, '-r', 'linewidth', width);
    if nargin > 2
        hold on
        yB = trajB.getControlMat(controlNames.get(i));
        plot(trajB.getTimeMat(), yB, '--b', 'linewidth', width);
        hold off
    end
    title(char(controlNames.get(i)), 'Interpreter', 'none')
    xlabel('time (s)')
    ylabel('value')
    if max(yA) <= 1 && min(yA) >= 0
        fixYLim = true;
        if nargin > 2 && (max(yB) > 1 || min(yB) < 0)
            fixYLim = false;
        end
        if fixYLim
            ylim([0, 1]);
        end
    end
    if i == 0 && nargin > 2
        if nargin == 5
            legend(nameA, nameB);
        else
            legend('A', 'B');
        end
    end
end
sgtitle('Controls')

if ~strcmp(saveLoc,'') % if save location has been specified, save
    filename = [saveLoc,'statesAndControls.pdf'];
    if ~isfile(filename)
        for i=1:2
                exportgraphics(figure(i), filename, 'Append', true);
        end
        disp(['saved ', filename])
    
    else
        disp(['WARNING ', filename, ' not saved b/c this file already exists'])
    end
end

return
