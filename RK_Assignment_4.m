%% 41013 Robotics Assigment 4 Script
% Ritesh KUMAR - 11656978
close all
clear all
clc
%% 1.4 Creating Fetch Robot Model
% Location (4x4 Transformation) of the Fetch Robot (7 DOF Arm Only)
robotBase = transl(0,0,0.8);

% Nmae of the Robot
robotName = 'Fetch_01';

% Creating Instance of the Fetch Object (Custom class)
% Refer to Fetch Class Documenation for details on D&H and other
% modelling parameters
robot = Fetch(robotName,robotBase);

% Clearing unrequired workspace Variables
clear 'robotBase' 'robotName';

%% Specifing the Initial Pose
% The Initial Pose has been select such that the Helical Stair Case is
% near the top left hand corner of the image plane

qInitial = robot.model.ikcon(transl(0.7,0.21,0.8)*trotx(pi));
robot.model.plot(qInitial);
axis([-1.2 1.2 -1.2 1.2 0 2]);
view(3);


%% Creating a 'Helical-Stairs' Object in the environment

% Location Transfromation of the helicalStairCase
base = transl(0.45,0.5,0);

% Creating the 3D Model using 'ObjectModels' Class (custom class)
stair = ObjectModels('Stairs',base,'helicalStaircaseModel.ply');
hold on;

% Visualising the 3D Model of Helical Stair Case
stair.PlotAndColour();
box off
view(3);
grid on
grid minor

%% Creating Feature Points
% Feature points are 3D are points in the world frame that describle the
% the location of the helical Stair case in the world.
% Note: These points are generated manually in this section of the the code
% The feature points will consist of 16 points.
% 
clc

% Allocate Array for Feature Points
featurePoints = zeros(16,3);
counter = 0;

% Helical Stair Case Base has radius of the 0.2 m.
stairRadius = 0.2;

% Create a Feature Points corresponding to particular angles on circle
% In this case, it creates a Feature point every 30 degress until one
% complete revolution
for i=0:360/12:359
    counter = counter + 1; 
    featurePoints(counter,1) = stairRadius*cos(deg2rad(i));
    featurePoints(counter,2) = stairRadius*sin(deg2rad(i));
end


% Create equally distributed linear feature points from the center of the
% helical start case to the base radius. Visually this help to identify the
% rotation of the helical start case and form a visual guide points. 
for i=0:stairRadius/4:stairRadius-0.0000001
    counter = counter + 1; 
    featurePoints(counter,1) = i;
end

% The height of the Helical Stair Case BASE
featurePoints(:,3) = 0; %[m]


% Transforming the Feature Points as per the base Helical Stair Case
if tr2rpy(base) == 0
    % No transformation needed
else
    rotation = tr2rpy(base);
    rotation(1) = -rotation(1);
    rotation(2) = -rotation(2);
    rotation(3) = -rotation(3);
    rotation = rpy2r(rotation);
    
    featurePoints= featurePoints*rotation;
end

if base(1:3,4) == 0
    % No transformation needed
else
    featurePoints(:,1) = featurePoints(:,1) + base(1,4);
    featurePoints(:,2) = featurePoints(:,2) + base(2,4);
    featurePoints(:,3) = featurePoints(:,3) + base(3,4);
end

% Remove unwanted variables from the workspace
clear 'counter' 'stairRadius' 'base' 'i' 'rotation';

%% Feature Points Visualisation - Overlay on Helical Stair Case

hold on;
plot3(featurePoints(:,1),featurePoints(:,2),featurePoints(:,3),'g*');
% axis([-0.6 0.6 -0.6 0.6 0 0.6]);
grid on
grid minor


%% Creating a Camera Model
% The camera mounted on the Fetch is a perspective Camera (CentralCamera)
% with focal length of 0.08 (8cm) , pixel size of 10e-5 pixels and 
% a resolution of 1024 x1024 with the centre point exactly 
% in the middle of image plane, which gets images at 25fps. 

focalLength = 0.08;
pixelSize = 10e-5;
resolution = [1024 1024];
principlePoint = [512 512];

% frame rate
fps = 25;

% Define the camera
cam = CentralCamera('focal', focalLength, 'pixel', pixelSize, ...
'resolution', resolution, 'centre', principlePoint, 'name', 'FetchCamera');

% 16 Referecen Points - The UV reference points that determine the finish
% poisition of the visual servo. These points have been generated such that
% when the visual servo finishes, the End Effector of the Robot will be
% appropimately 0.55m above the base of the helical stair case. 
% Loads are varible in the workspace called 'pRef';
load('pRef.mat');

clear 'focalLength' 'pixelSize' 'resolution' 'principlePoint';

%% Setting the Camera Positoin For Eye-in-hand Configuration

% The pose of the camera is given by the position of the End Effector
% Note that initial Position of the Effector has been defined above.
Tc0 = robot.model.fkine(qInitial);
cam.T = Tc0;
drawnow;

%% Plotting the Camera 
cam.plot_camera('scale',0.09,'Tcam',Tc0,'color','y','solid');
hold on

%% Create Camera View
% Plot the Desired Features Points in the Camera Image Plane
cam.plot(pRef, 'r*'); % create the camera view
cam.hold(true);

% Plot the Current Feature Points in the Camera Image Plane
uv = cam.plot(featurePoints'); % create the camera view

%% Visual Servoing

% Values
%gain of the controller
lambda = 0.65;

% This will the approximate depth of the helical Stair Case at the end of
% the visual servoing task
depth = 0.55;

% Starting Joint State -> Current Joint State
q0 = robot.model.getpos();
%% Visual Servoing Loop

% Iterations Limit
kstepsMax = 250;

% Loop iteration counter
ksteps = 0;

% Clear command window
clc
while true
    % Increment Iteration counter
    ksteps = ksteps + 1;
    
    % Hold Camera View Figure
    cam.hold(true);
    
    % compute the view of the camera
    uv = cam.plot(featurePoints');
    
    % compute image plane error as a column
    e = pRef-uv;   % feature error
    e = e(:);
    
    % compute the Jacobian
    J = cam.visjac_p(uv, depth);
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    
    %compute robot's Jacobian (end Effector Frame of Reference) and inverse
    J2 = robot.model.jacobn(q0);
    Jinv = pinv(J2);
    
    % get joint velocities
    qp = Jinv*v;
    
    % Check if Joint Velocities exceeds limits
    % Velocity limit of each joints is avaliable as Fetch class property
    for i = 1:robot.model.n
        
        if abs(qp(i)) > robot.veloLim(i)            
            % Cap joint Velocity if above limits
            qp(i) = sign(qp(i))*robot.veloLim(i);            
        end
        
    end
    
    %Update joints
    q = q0' + (1/fps)*qp;
    robot.model.animate(q');
    
    %Get camera location - Current End Effector Location
    Tc = robot.model.fkine(q');
    cam.T = Tc;
    drawnow; 
    pause(1/fps);
    
    % Break While loop is number of iterations completed exceed the
    % desiredn
    if (ksteps > kstepsMax)
        break;
    end
    
    %update current joint position
    q0 = q';
    
end %visual servoing loop finishes
 
 % Clearing Unrequired Workspace Variables
 clear 'featurePoints' 'pRef' 'fps' 'Tc0' 'depth' 'lambda' 'q0' 'i';
 clear 'e' 'J' 'J2' 'Jinv' 'ksteps' 'kstepsMax'
 clear 'q' 'qInitial' 'qp' 'Tc' 'uv' 'v';
 disp('Visual Servoing - Completed');
 
%% Picking Up the Helical Stair Case

% Ensure the 3D Object is at the Originally Set Location
stair.SetPose(stair.rest);
stair.UpdatePlot();

% End Effector Gripping Transoformation Relative to Stairs
gripTr = transl(0,0,0.48)*trotx(pi);

% Goal Top of Helical Stairs
goalTr = stair.GetCurrentPose*gripTr;

% Starting Joint State
qStart = robot.model.getpos;

% Goal Joint State - Top of Helical Stairs
qEnd = robot.model.ikcon(goalTr,qStart);

% Generate Trejectory using trapezoidal velocity profile method
qMatrix = robot.GenJointStateTraj(qStart,qEnd,20);

% Animating Robot to Move
for i=1:size(qMatrix,1)
    
    % Animate Robot to Joint State
    robot.model.animate(qMatrix(i,:));
    
    % Update Position of Camera
    cam.T = robot.model.fkine(robot.model.getpos);
    drawnow; 
 
end 

clc

clear 'qStart' 'qMatrix' 'qEnd' 'goalTr' 'i'

%% Task 2: Dynamic Troque
% Credit: Section of Code Contains Snipts of Code From 41013 Robotics Lab 9 
% Question 3 - Solution written by J.Woofley.

%% Scenario Selection
% ScenarioNumber = 1:
% slow enough so none of the joints’ maximum torques are exceeded during
% motion (given object mass of 2kg)
% ScenarioNumber = 2:
% fast enough so the arm is overloaded and thus fails to achieve the
% task (given the object mass is 5kg) (10)

% Scenario Type Tracker
scenarioNum = 2;
clc
switch(scenarioNum)
    case 1
        disp('Scenario Number: 1 Selected' );
        disp('Scenario Info: Slow Enough - No Joints Are Overloaded');       
        % Payload mass (kg)
        mass = 2;
        % Time to Complete the Trajectory (seconds)
        time = 3;      % t = 3 is successful, t = 2.46 is failure
        
    case 2
        disp('Scenario Number: 2 Selected' );
        disp('Scenario Info: Fast Enough - Joint(s) Are Overloaded');
        % Payload mass (kg)
        mass = 5;
        % Time to Complete the Trajectory (seconds)
        time = 2;   % t = 4 is successful,  t = 2 is failure
      
    otherwise
        disp('Invalid Scenario Number' );
end

disp(['PayLoad = ', num2str(mass)]);
disp(['Trajectory Time = ', num2str(time)]);

%% Defining Trajectory Start and Goal Position

% Start Pos - Top of Helical Stairs
startTr = stair.GetCurrentPose*gripTr;

% Starting Joint State
% qStart = robot.model.ikcon(startTr,robot.restState);
qStart = robot.model.getpos;

% The Drop of Location of the Helical Stair Case.
dropOfLocation = transl(-0.5,-0.5,1.5)*troty(2*pi/3)*trotx(-pi/4);

% Goal Location of the End Effector (top of the Helical Stairs)
goalTr = dropOfLocation*gripTr;

% Goal Joint State - Top of Helical Stairs at the DropOff Location
qEnd = robot.model.ikcon(goalTr,robot.restState);


%% Torque Minimisation 
clc
% Array of Random Jointstate to be used as initial guess for
% Inverse kinematics
randomJointStates = [];

% One of the opition is restState of the robot
randomJointStates(1,:) = robot.restState; 

% Array of Possible Jointstate to hold the object
possibleStartQ = [];

possibleStartQ(1,:) = qStart;

% Robot Joint Limits
jointLims = robot.model.qlim;

% Posssible Joints State Sample Size
counterMax = 50;

clc
% Counter for countering number of joint states
counter = 1;

while counter < counterMax
    
    % Increment Counter
    counter = counter+1;
    
    % Generate randon jointstate within each joint Limit
    for i=1:robot.model.n        
        randomJointStates(counter,i) = (jointLims(i,2) - jointLims(i,1)).*...
        rand(1,1) + jointLims(i,1);     
    end
    
    % Inverse kinematics
    q = robot.model.ikcon(startTr,randomJointStates(counter,:));
    
    % Get the actual pose
    T = robot.model.fkine(q);  
    
    % Norm of pose error
    err = norm(startTr-T);
    
    % Check If the error between Calculated and Desired is below thrashhold
    % 
    if err < 0.0001
        
        % If error less than thrashhold, then store jointstate as possible
        % starting jointstate
        possibleStartQ(counter,:) = q;
    else
        counter = counter -1;
    end
    
end

clear 'i' 'T' 'err' 'counter' 'counterMax' 'jointLims' 'randomJointStates'
clear 'q'

%% Calculating Static Torque For each Joints Configuration
clc
torqueMax = [];

for i = 1: size(possibleStartQ,1)
    
    % Get the gravity torque at this configuration
    g = robot.model.gravload(possibleStartQ(i,:))';
    
    % Get the Jacobian at this pose
    J = robot.model.jacob0(possibleStartQ(i,:));
        
    % Calculate the wrench (Mass is dependant on the Sceneraio
    w = [0 0 mass*9.81 0 0 0]'; 

    % Joint Torque
    tau = g + J'*w;

    % Store the absolute Maximum Torque Value
    torqueMax(i,1) = max(abs(tau));
       
end
clear 'i' 'g' 'J' 'w' 'tau'
%% Move Robot to Jointstate with Minimum Static Torque
% Minimum Torque Index
index = find(torqueMax == min(torqueMax));
robot.model.animate(possibleStartQ(index,:));

%% Plotting Troque Minimisation Results
try close('Torque Minimisation'); end
figure('Name','Torque Minimisation');
plot(torqueMax,'b','LineWidth',1);
ylabel('Max Static Torque(N/m)');
xlabel('Possible JointStates');
title('Torque Minimisation');
box off
grid on
grid minor

% Annotate the Plot
text( index, ...
    torqueMax(index) , ...
    [ '\leftarrow ',sprintf( 'Minimum')],...
    'Color', [1, 0, 0.0], 'FontSize', 12 );




%% Dynamics Torque

% Set control frequency at 50Hz
dt = 1/50;
% No. of steps along trajectory
steps = floor(time/dt);

% Generate Trejectory using trapezoidal velocity profile method
qMatrix = robot.GenJointStateTraj(possibleStartQ(index,:),qEnd,steps);
% qMatrix = robot.GenJointStateTraj(qStart,qEnd,steps);

% Setting payload mass in Fetch Arm model
robot.model.payload(mass,[0;0;0]);

% Array of joint velocities
qd = zeros(steps,robot.model.n);
% Array of joint accelerations
qdd = nan(steps,robot.model.n);
% Array of joint torques
tau = nan(steps,robot.model.n);

% Flag to Indicate Failture
overloaded = 0

display(steps)

%% 
for i = 1:steps-1
    
    % Calculate joint acceleration to get to next set of joint angles
    qdd(i,:) = (1/dt)^2 * (qMatrix(i+1,:) - qMatrix(i,:) - dt*qd(i,:));
    
    % Calculate inertia matrix at this pose
    M = robot.model.inertia(qMatrix(i,:));
    
    % Calculate coriolis matrix at this pose
    C = robot.model.coriolis(qMatrix(i,:),qd(i,:));
    
    % Calculate gravity vector at this pose
    g = robot.model.gravload(qMatrix(i,:));  
    
    % Calculate the joint torque needed
    tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';
    
    
    for j = 1:robot.model.n
        
        % Check if torque exceeds limits
        if abs(tau(i,j)) > robot.torqueLim(j)
            
            % Display Warning
            warning('Step No: %0.1f ,Joint: %0.1f *** Torque Limit Exceeded ***',i,j)
            overloaded = 1;
            % Cap joint torque if above limits
            tau(i,j) = sign(tau(i,j))*robot.torqueLim(j);
        end
        
    end
    
    % Re-calculate acceleration based on actual torque
    qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';
    
    % Update joint angles based on actual acceleration
    qMatrix(i+1,:) = qMatrix(i,:) + dt*qd(i,:) + dt^2*qdd(i,:); 
    
    % Update the velocity for the next pose
    qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      
end


t = 0:dt:(steps)*dt - 0.001;  

if overloaded
   warning('!!!*** Likely to Fail Due TORQUE Overload ***!!!' ) 
else
   disp('No Torque Overload Detected'); 
end

%% Visulalisation and plotting of results From Dynamic 

% Close figures if already Exiting

try close('Joint Angles'); end
try close('Joint Velocities'); end
try close('Joint Acceleration'); end
try close('Joint Torque'); end

% Plot joint angles
figure('Name','Joint Angles')
for j = 1:robot.model.n
    subplot(4,2,j)
    plot(t,qMatrix(:,j)','k','LineWidth',1);    
    if j == 1 || j == 2 || j == 4 || j == 6        
        refline(0,robot.model.qlim(j,1));
        refline(0,robot.model.qlim(j,2)); 
    end
    ylabel('Angle(rad)');
    %xlabel('Time (sec)');
    title(['Joint: ', num2str(j)]);
    box off
    grid on
    grid minor
end


% Plot joint velocities
figure('Name','Joint Velocities')

for j = 1:robot.model.n
    subplot(4,2,j)
    plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
%     refline(0,robot.veloLim(j)*30/pi);
%     refline(0,-robot.veloLim(j)*30/pi);
    ylabel('Vel(RPM)');
    xlabel('Time (sec)');
    title(['Joint: ', num2str(j)]);
    box off
    grid on
    grid minor
    
end

% Plot joint acceleration
figure('Name','Joint Acceleration')
for j = 1:robot.model.n
    subplot(4,2,j)
    plot(t,qdd(:,j),'k','LineWidth',1);
    ylabel('Acc (rad/s/s');
    xlabel('Time (sec)');
    title(['Joint: ', num2str(j)]);
%     refline(0,0)
    box off
    grid on
    grid minor
end

% Plot joint torques
figure('Name','Joint Torque')
for j = 1:robot.model.n
    subplot(4,2,j)
    plot(t,tau(:,j),'k','LineWidth',1);
    refline(0,robot.torqueLim(j));
    refline(0,-robot.torqueLim(j));
    ylabel('Torque (Nm)');
    xlabel('Time (sec)');
    title(['Joint: ', num2str(j)]);
    box off
    grid on
    grid minor
end

%% Test Completion of Task
clc
% Get the actual pose - last jointState in the trajectory
T = robot.model.fkine(qMatrix(end,:));

% Norm of pose error
errorFinal = norm(goalTr-T);

if errorFinal < 1e-03
    disp('Task Completed');
else
    disp('Task Not Completed');
end


%% Animate the Robot to Perform Trajectory with Object 
% Animating Robot to Move
for i=1:size(qMatrix,1)
    
    robot.model.animate(qMatrix(i,:));
    
    % Forward Kinematics
    endEffTr = robot.model.fkine(robot.model.getpos);
    
    % Update Position of Camera
    cam.T = endEffTr;
    drawnow;
    
    % Update the Location of the 3D Model
    stair.SetPose(endEffTr*inv(gripTr));
    stair.UpdatePlot();
     
end 