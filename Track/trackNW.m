% -------------------------------------------------------------------------- %
% OpenSim Moco: example2DWalking.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Brian Umberger                                                  %
%                                                                            %
% Licensed under the Apache License, Version 2.0 (the "License"); you may    %
% not use this file except in compliance with the License. You may obtain a  %
% copy of the License at http://www.apache.org/licenses/LICENSE-2.0          %
%                                                                            %
% Unless required by applicable law or agreed to in writing, software        %
% distributed under the License is distributed on an "AS IS" BASIS,          %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   %
% See the License for the specific language governing permissions and        %
% limitations under the License.                                             %
% -------------------------------------------------------------------------- %

% This is a Matlab implementation of an example optimal control
% problem (2-D walking) orginally created in C++ by Antoine Falisse
% (see: example2DWalking.cpp).
%
% This example features two different optimal control problems:
%  - The first problem is a tracking simulation of walking.
%  - The second problem is a predictive simulation of walking.
%
% The code is inspired from Falisse A, Serrancoli G, Dembia C, Gillis J,
% De Groote F: Algorithmic differentiation improves the computational
% efficiency of OpenSim-based trajectory optimization of human movement.
% PLOS One, 2019.
%
% Model
% -----
% The model described in the file '2D_gait.osim' included in this file is a
% modified version of the 'gait10dof18musc.osim' available within OpenSim. We
% replaced the moving knee flexion axis by a fixed flexion axis, replaced the
% Millard2012EquilibriumMuscles by DeGrooteFregly2016Muscles, and added
% SmoothSphereHalfSpaceForces (two contacts per foot) to model the
% contact interactions between the feet and the ground.
%
% Do not use this model for research. The path of the gastroc muscle contains
% an error--the path does not cross the knee joint.
%
% Data
% ----
% The coordinate data included in the 'referenceCoordinates.sto' comes from
% predictive simulations generated in Falisse et al. 2019.  As such,
% they deviate slightly from typical experimental gait data.


clear all; close all; % clc;
isMac = isunix; % figure out if running on Ava or Russell's computer

% Load the Moco libraries
import org.opensim.modeling.*;

% Add location of relevant reference files
% assuming this script is in Track folder, baseDir should be something like
% "C:\Users\russe\Documents\stanford\PredictExoGait"
baseDir = fileparts(cd); 
loc_ReferenceTrackingData = [baseDir,'/Experiment/'];
file_ReferenceTrackingData = 'NW1_muscleDrivenIK_ground.sto';
loc_InitialGuess = [baseDir,'\Track\Results\2023-06-01 4R\'];
file_InitialGuess = 'gaitTracking_solution_notFullStride.sto'; % low resids
loc_referenceGRF = [baseDir,'/Experiment/'];
file_referenceGRFxml = 'NW1_external_forces_trim_stride.xml';
file_referenceGRFmot = 'NW1_grf_trim_stride.mot';
loc_model = [baseDir,'/Models/'];
file_model = 'Ong_gait9dof18musc_addMarkers_scalePB.osim';
s.ReferenceTrackingDataGRFDataAndModel = {file_ReferenceTrackingData, file_referenceGRFxml, file_model};

% ---------------------------------------------------------------------------
% Set up a coordinate tracking problem where the goal is to minimize the
% difference between provided and simulated coordinate values and speeds (and
% ground reaction forces), as well as to minimize an effort cost (squared
% controls). The provided data represents half a gait cycle. Endpoint
% constraints enforce periodicity of the coordinate values (except for
% pelvis tx) and speeds, coordinate actuator controls, and muscle activations.


% Define the optimal control problem
% ==================================
track = MocoTrack();
track.setName('gaitTracking');

% Set the weights for the terms in the objective function. The values below were
% obtained by trial and error.
%
% Note: If s.GRFTrackingWeight is set to 0 then GRFs will not be tracked. Setting
% s.GRFTrackingWeight to 1 will cause the total tracking error (states + GRF) to
% have about the same magnitude as control effort in the final objective value.
s.controlEffortWeight = 0.01;
s.stateTrackingWeight = 10;
s.GRFTrackingWeight   = 0.05;

% Reference data for tracking problem
% tableProcessor = TableProcessor('MocoT_muscleDriven_OpensimIK_raisePelvis2.sto');
% tableProcessor = TableProcessor([loc_ReferenceTrackingData, file_ReferenceTrackingData]);
% tableProcessor.append(TabOpLowPassFilter(6));

modelProcessor = ModelProcessor([loc_model, file_model]);
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
s.reserveOptimalForce = 5;
s.reserveBound = 1;
modelProcessor.append(ModOpAddReserves(s.reserveOptimalForce, s.reserveBound));

% added for moco labeling fix
model = modelProcessor.process();
model.initSystem();
coordinates = TableProcessor([loc_ReferenceTrackingData, file_ReferenceTrackingData]);
coordinates.append(TabOpLowPassFilter(6));
coordinates.append(TabOpUseAbsoluteStateNames());
% coordinatesRadians = coordinates.processAndConvertToRadians(model);
tableProcessor = coordinates;

track.setModel(modelProcessor);
track.setStatesReference(tableProcessor);
track.set_states_global_tracking_weight(s.stateTrackingWeight);
track.set_allow_unused_references(true);
track.set_track_reference_position_derivatives(true);
track.set_apply_tracked_states_to_guess(true);
% track.set_guess_file([loc_InitialGuess, file_InitialGuess]);
track.set_initial_time(306.6);  % 306.6 (us) or 0 (ex) 
track.set_final_time(307.85);    % 307.5 (us) or 0.47 (ex) stride is 307.8
study = track.initialize();
problem = study.updProblem();


% initial guess - increase residuals

% Goals
% =====

% Symmetry
% --------
% This goal allows us to simulate only one step with left-right symmetry
% that we can then double to create a full gait cycle.
s.doSymmetry = false;

if s.doSymmetry
    symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
    problem.addGoal(symmetryGoal);
    model = modelProcessor.process();
    model.initSystem();
    
    % Symmetric coordinate values (except for pelvis_tx) and speeds. Here, we 
    % constrain final coordinate values of one leg to match the initial value of the 
    % other leg. Or, in the case of the pelvis_tx value, we constrain the final 
    % value to be the same as the initial value.
    for i = 1:model.getNumStateVariables()
        currentStateName = string(model.getStateVariableNames().getitem(i-1));
        if startsWith(currentStateName , '/jointset')
            if contains(currentStateName,'_r')
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                               regexprep(currentStateName,'_r','_l'));
                symmetryGoal.addStatePair(pair);
            end
            if contains(currentStateName,'_l')
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                               regexprep(currentStateName,'_l','_r'));
                symmetryGoal.addStatePair(pair);
            end
            if (~contains(currentStateName,'_r') && ...
                ~contains(currentStateName,'_l') && ...
                ~contains(currentStateName,'pelvis_tx/value') && ...
                ~contains(currentStateName,'/activation'))
                symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
            end
        end
    end
    
    % Symmetric muscle activations. Here, we constrain final muscle activation 
    % values of one leg to match the initial activation values of the other leg.
    for i = 1:model.getNumStateVariables()
        currentStateName = string(model.getStateVariableNames().getitem(i-1));
        if endsWith(currentStateName,'/activation')
            if contains(currentStateName,'_r')
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                             regexprep(currentStateName,'_r','_l'));
                symmetryGoal.addStatePair(pair);
            end
            if contains(currentStateName,'_l')
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                             regexprep(currentStateName,'_l','_r'));
                symmetryGoal.addStatePair(pair);
            end
        end
    end
    
    % The lumbar coordinate actuator control is symmetric.
    % symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));
end 

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default and change the weight
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));
effort.setWeight(s.controlEffortWeight);

% Optionally, add a contact tracking goal.
if s.GRFTrackingWeight ~= 0
    % Track the right and left vertical and fore-aft ground reaction forces.
    contactTracking = MocoContactTrackingGoal('contact', s.GRFTrackingWeight);
    contactTracking.setExternalLoadsFile([loc_referenceGRF, file_referenceGRFxml]);
    forceNamesRightFoot = StdVectorString();
    forceNamesRightFoot.add('/forceset/contactHeel_r');
    forceNamesRightFoot.add('/forceset/contactFront_r');
    contactTracking.addContactGroup(forceNamesRightFoot, 'ForcePlate2');
    forceNamesLeftFoot = StdVectorString();
    forceNamesLeftFoot.add('/forceset/contactHeel_l');
    forceNamesLeftFoot.add('/forceset/contactFront_l');
    contactTracking.addContactGroup(forceNamesLeftFoot, 'ForcePlate1');
    contactTracking.setProjection('plane');
    contactTracking.setProjectionVector(Vec3(0, 0, 1));
    problem.addGoal(contactTracking);
end


% Bounds
% ======
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [1, 2.5]); % or 1 to 1.1
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.9, 1.25]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-1, 1]);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-1, 1]);
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-70*pi/180, 0]); % ran with [-1, 0])
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-70*pi/180, 0]);
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-40*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-40*pi/180, 25*pi/180]); % was [-40*pi/180, 15*pi/180]
problem.setStateInfo('/jointset/lumbar/lumbar/value', [-0.0873, -0.0873]);

% Reserves
% ======
% model = modelProcessor.process();
% model.initSystem();
s.reserveWeightForControl = 15; 
forceSet = model.getForceSet();
for i = 0:forceSet.getSize()-1
   forcePath = forceSet.get(i).getAbsolutePathString();
   if contains(string(forcePath), 'reserve')
       effort.setWeightForControl(forcePath, s.reserveWeightForControl);
   end
end


% Configure and solve the problem
% =================
s.numMeshIntervals = 50;
s.optimConvergenceTolerance = 1e-3;
s.optimConstraintTolerance = 1e-1;
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(s.numMeshIntervals);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(s.optimConvergenceTolerance);
% scale objective to be between 0.1 and 10 (~iter 100) with tol 1e-1 or looser
% in future, use convergence analysis to determine tolerances
solver.set_optim_constraint_tolerance(s.optimConstraintTolerance); % 10e-3 or 10e-4
% solver.set_optim_max_iterations(500);
solver.setGuessFile([loc_InitialGuess, file_InitialGuess])
gaitTrackingSolution = study.solve();

gaitTrackingSolutionUnsealed = gaitTrackingSolution.unseal();

% get results file save location: ./Results/YYYY-MM-DD #
if isMac
    name = 'A';
else
    name = 'R';
end

for i=1:1000
    resultsLoc = ['./Results/', char(datetime('now','Format','yyyy-MM-dd')), ' ', num2str(i), name];
    if ~exist(resultsLoc)
        break; 
    end
end

mkdir(resultsLoc)
resultsLoc = append(resultsLoc, '/');

% plot
close all;
% mocoPlotTrajectory(filename, gaitTrackingSolution)

% Create a full stride from the periodic single step solution.
% For details, view the Doxygen documentation for createPeriodicTrajectory().
notFullStrideFilename = 'gaitTracking_solution_notFullStride.sto';
gaitTrackingSolution.write([resultsLoc, notFullStrideFilename]);
disp(['saved notFullStride to', resultsLoc, notFullStrideFilename]);

fullStrideFilename = 'gaitTracking_solution_fullStride.sto';
fullStride = opensimMoco.createPeriodicTrajectory(gaitTrackingSolution);
fullStride.write([resultsLoc,fullStrideFilename]);
disp(['saved fullStride to', resultsLoc, fullStrideFilename]);

% Uncomment next line to visualize the result
% study.visualize(fullStride);


% Extract ground reaction forces
% ==============================
contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('/forceset/contactHeel_r');
contact_r.add('/forceset/contactFront_r');
contact_l.add('/forceset/contactHeel_l');
contact_l.add('/forceset/contactFront_l');

externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
                                 fullStride,contact_r,contact_l);
file_GRFsOut = 'gaitTracking_solutionGRF_fullStride_trackNW.sto';
STOFileAdapter.write(externalForcesTableFlat, ...
                             [resultsLoc, file_GRFsOut]);
disp(['saved ext forces to gaitTracking_solutionGRF_fullStride_trackNW.sto at ', resultsLoc]);
save([resultsLoc, 'optSettings.mat'], '-struct', 's')

% plot
close all;
trajA.loc = resultsLoc;
trajA.file = notFullStrideFilename;
trajB.loc = loc_ReferenceTrackingData;
trajB.file = file_ReferenceTrackingData;
grfA.loc = resultsLoc;
grfA.file = file_GRFsOut;
grfB.loc = loc_referenceGRF;
grfB.file = file_referenceGRFmot;
nameA = 'tracking output';
nameB = 'reference';
addpath([baseDir,'/Helpers/'])
mocoPlotTrajectoryfromFile(resultsLoc, trajA, trajB, nameA, nameB, grfA, grfB, s, model);

return