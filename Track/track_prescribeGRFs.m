% -------------------------------------------------------------------------- %
% OpenSim Moco: exampleMocoTrack.m                                           %
% -------------------------------------------------------------------------- %
% Copyright (c) 2019 Stanford University and the Authors                     %
%                                                                            %
% Author(s): Nicholas Bianco                                                 %
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

% function [track, solution] = exampleMocoTrack()

% Solve the muscle-driven state tracking problem.
% [track, solution] = muscleDrivenStateTracking();

% Plot and save outputs
% control_names = ["hamstrings_r", "bifemsh_r", "glut_max_r", "iliopsoas_r", "rect_fem_r", ...
%     "vasti_r", "gastroc_r", "soleus_r", "tib_ant_r", "hamstrings_l", "bifemsh_l", ...
%     "glut_max_l", "iliopsoas_l", "rect_fem_l", "vasti_l", "gastroc_l", "soleus_l", ...
%     "tib_ant_l", "reserve_pelvis_tilt", "reserve_pelvis_tx", "reserve_pelvis_ty", ...
%     "reserve_hip_r", "reserve_knee_r", "reserve_ankle_r", "reserve_hip_l", ...
%     "reserve_knee_l", "reserve_ankle_l"];
% activations = solution.getControlsTrajectoryMat;
% 
% figure
% for m = 1:length(control_names)
%     subplot(3,9,m)
%     plot(activations(:,m), 'LineWidth', 2);
%     title(control_names(m), 'Interpreter','none')
%     ylim([0 0.5])
% end
% 
% sol.activations = solution.getControlsTrajectoryMat();
% sol.states = solution.getStatesTrajectoryMat();
% save('solution.mat', 'sol')


% function [track, solution] = muscleDrivenStateTracking()

import org.opensim.modeling.*;

% Create and name an instance of the MocoTrack tool. 
track = MocoTrack();
track.setName("muscle_driven_state_tracking");

% Construct a ModelProcessor and set it on the tool. The default
% muscles in the model are replaced with optimization-friendly
% DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
% parameters.
modelProcessor = ModelProcessor("../Models/Ong_gait9dof18musc_addMarkers_scalePB.osim");
modelProcessor.append(ModOpAddExternalLoads("../Experiment/NW1_external_forces_trim_stride.xml"));
modelProcessor.append(ModOpIgnoreTendonCompliance());
modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% Only valid for DeGrooteFregly2016Muscles.
modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));

%
modelProcessor.append(ModOpAddReserves(1000,1));

track.setModel(modelProcessor);

% Construct a TableProcessor of the coordinate data and pass it to the 
% tracking tool. TableProcessors can be used in the same way as
% ModelProcessors by appending TableOperators to modify the base table.
% A TableProcessor with no operators, as we have here, simply returns the
% base table.
track.setStatesReference(TableProcessor("../Experiment/NW1_ik_trim_stride.mot"));
track.set_states_global_tracking_weight(10);
% track.set_control_effort_weight(0.001);
% default global tracking weight = 1, set to 10 in 3D tutorial
% defualt control effort weight = 0.001

% This setting allows extra data columns contained in the states
% reference that don't correspond to model coordinates.
track.set_allow_unused_references(true);

% Since there is only coordinate position data in the states references, this
% setting is enabled to fill in the missing coordinate speed data using
% the derivative of splined position data.
track.set_track_reference_position_derivatives(true);

% Initial time, final time, and mesh interval.
track.set_initial_time(306.6);
track.set_final_time(307.85); % 307.5 for stance
track.set_mesh_interval(0.08);

% Instead of calling solve(), call initialize() to receive a pre-configured
% MocoStudy object based on the settings above. Use this to customize the
% problem beyond the MocoTrack interface.
study = track.initialize();

% Get a reference to the MocoControlGoal that is added to every MocoTrack
% problem by default.
problem = study.updProblem();
effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));

% Bounds
% ======
problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0.75, 1.25]);
problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [1, 1.1]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-1, 1]);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-1, 1]);
problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-1, 0]);
problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-70*pi/180, 0]);
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-40*pi/180, 25*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-40*pi/180, 15*pi/180]);
problem.setStateInfo('/jointset/lumbar/lumbar/value', [-0.0873, -0.0873]);


% Put a large weight on the pelvis CoordinateActuators, which act as the
% residual, or 'hand-of-god', forces which we would like to keep as small
% as possible.
model = modelProcessor.process();
model.initSystem();
forceSet = model.getForceSet();
for i = 0:forceSet.getSize()-1
   forcePath = forceSet.get(i).getAbsolutePathString();
   if contains(string(forcePath), 'reserve')
       effort.setWeightForControl(forcePath, 10);
   end
end

% for i = 0:forceSet.getSize()-1
%    forcePath = forceSet.get(i).getAbsolutePathString();
%    if ~contains(string(forcePath), 'Limit')
%     effort.setWeightForControl(forcePath, 0.001);
%    end
%    if contains(string(forcePath), 'reserve')
%        effort.setWeightForControl(forcePath, 10);
%    end
% end

% Solve and visualize.
solution = study.solve();
solution.write('solution_prescribedGRFs.sto')
study.visualize(solution);
% end

trajA.loc = '/Users/avalakmazaheri/Documents/GitHub/PredictExoGait/Track/';
trajA.file = 'solution_prescribedGRFs.sto';
trajB.loc = '../Experiment/';
trajB.file = 'NW1_muscleDrivenIK_ground.sto';

nameA = 'tracking output';
nameB = 'reference';
addpath('../Helpers/')
mocoPlotTrajectoryfromFile(trajA.loc, trajA, trajB, nameA, nameB, [], []);



