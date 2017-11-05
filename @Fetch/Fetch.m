classdef Fetch < handle  
    
    properties (Access = public)
        
        %> Robot model
        model;
                       
        %> Name of the Dobot Magician
        name;
        
        %> Base Location of the Dobot Magician
        base;
        
        %> Threshold value for manipulability/Damped Least Squares
        epsilon = 0.1;
        
        %> Weighting matrix for the velocity vector
        W = diag([1 1 1 0.1 0.1 0.1]);    
        
                 
    end
    
    properties (Access = public, Constant)
        
        %> rest Joint State
        restState = [0,0,0,0,0,0,0];
        
        %> Torque Limit [Nm]
        torqueLim = [33.82, 131.76, 76.94, 66.18, 29.35, 25.70, 7.36];
        
        %> Velocity Limit [rad/s]
        veloLim = [1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26];
        
    end
    
    
    methods (Access = private)
        %% CreateModel
        function CreateModel(self)
            
            % link name="shoulder_pan_link" (URDF Start line: 196)
            L(1) = Link(...
                'revolute',...
                'd', 0.06, ...
                'a', 0.117,...
                'alpha', -pi/2, ...
                'offset', 0, ...
                'qlim', deg2rad([-92 92]), ...
                'I', [0.0125 0.0388 0.0308 7.0814e-04 -0.0124 0.0012], ...
                'r', [0.0927 -0.0056 0.0564], ...
                'm', 2.5587);
            
            % Name: 'shoulder_lift_link'
            L(2) = Link(...
                'revolute',...
                'd', 0, ...
                'a', 0,...  % 0.2190
                'alpha', pi/2, ...
                'offset', -pi/2, ...    
                'qlim', deg2rad([-87 72]), ...
                'I', [0.0029 0.0657 0.0659 1.9163e-06 3.8113e-05 -0.0048], ...
                'r', [0.1432 0.0072 -1.0000e-04], ...
                'm', 2.6615);
            
            %Name: 'upperarm_roll_link'
            L(3) = Link(...
                'revolute',...
                'd', -(0.2190+0.133), ...
                'a', 0,...
                'alpha', -pi/2, ...
                'offset', 0, ...
                'qlim', deg2rad([-1080 1080]), ...
                'I', [0.0019 0.0361 0.0363 0 0 -4.8020e-04], ...
                'r', [0.1165 0.0014 0], ...
                'm', 2.3311);
            
            % Name: 'elbow_flex_link'
            L(4) = Link(...
                'revolute',...
                'd', 0, ...
                'a', 0,...
                'alpha', pi/2, ...
                'offset', 0, ...
                'qlim', deg2rad([-129 129]), ...
                'I', [0.0025 0.0430 0.0434 0 0 -0.0036], ...
                'r', [0.1279 0.0073 0], ...
                'm', 2.1299);
            
            %Name: 'forearm_roll_link'
            L(5) = Link(...
                'revolute',...
                'd', -(0.1970+0.1245), ...
                'a', 0,...
                'alpha', -pi/2, ...
                'offset', 0, ...
                'qlim', deg2rad([-1080 1080]), ...
                'I', [0.0028 0.0229 0.0246 0 0 0.0045], ...
                'r', [0.1097 -0.0266 0], ...
                'm', 1.6563);
            
            %Name: 'wrist_flex_link'
            L(6) = Link(...
                'revolute',...
                'd', 0, ...
                'a', 0,...
                'alpha', pi/2, ...
                'offset', 0, ...
                'qlim', deg2rad([-127 127]), ...
                'I', [0.0018 0.0176 0.0176 ...
                     1.5525e-07 1.5215e-05 -2.3693e-04], ...
                'r', [0.0882 9.0000e-04 -1.0000e-04], ...
                'm', 1.7250);
            
            %Name: 'wrist_roll_link'
            L(7) = Link(...
                'revolute',...
                'd', -(0.1385+0.1664), ...
                'a', 0,...
                'alpha', pi, ...
                'offset', 0, ...
                'qlim', deg2rad([-1080 1080]), ...                          
                'I', [1.0003e-04 1.1223e-04 1.1224e-04 ...
                1.0832e-08 2.5726e-07 -5.1452e-07], ...
                'r', [0.0095 4.0000e-04 -2.0000e-04], ...
                'm', 0.1354);
           
            
            self.model = SerialLink(L, 'name', self.name,...
                         'base',self.base, ...
                          'manufacturer', 'Fetch Robotics');              
        end

    end
      
    methods (Access = public)
        %% Constructor
        function self = Fetch(robotName,baseTr)
                                              
            if nargin < 2
                baseTr = transl(0,0,0);               
                if nargin < 1                    
                    disp('No Name Passed');
                    robotName = ['Fetch ',datestr(datetime('now'))];                    
                end                                
            end
            
            self.name = robotName;
            self.base = baseTr;
            
            % Call private method to create the SerialLink Model
            self.CreateModel();
            
            % Set smaller delay when animating
            self.model.delay = 0.001;    
                        
            disp('Fetch Created !!!');
            

        end
        
        
        

        
        %% GenJointStateTraj
        % ======================================================================
        %> @brief GenJointStateTrajclass method that generates a
        %> trajectory using trapezoidal velocity profile method between the
        %> two joint states.
        %>
        %> @param self instance of the Fetch class.
        %> @param q1 starting joint state.
        %> @param q2 end joint state.
        %> @param steps number of interpolation between two specified joint
        %> states
        %> @retval qMatrix [nx5] matrix of joint states
        % ======================================================================
        function qMatrix = GenJointStateTraj(self,q1,q2,steps)
            
            s = lspb(0,1,steps);
            qMatrix = nan(steps,self.model.n);
            for i=1:steps
               qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2; 
            end
        end
        
        
        
        
        %% GenCompTraj
        % ======================================================================
        %> @brief GenCompTraj class method that generates a
        %> trajectory using trapezoidal velocity profile method between the
        %> two transformations. Return matrix of xyz and rpy component
        %> based trajectory
        %>
        %> @param self instance of the Fetch class.
        %> @param startTr starting joint state.
        %> @param goalTr end joint state.
        %> @param steps number of interpolation between two specified
        %> transformations
        %> @retval [pos,rot] - pos = Array for x-y-z trajectory. 
        %> rot = Array for roll-pitch-yaw angles
        % ======================================================================
        function [pos,rot]= GenCompTraj(self,startTr,goalTr,steps)
                       
            % Array for x-y-z trajectory
            pos = zeros(3,steps);
            % Array for roll-pitch-yaw angles
            rot = zeros(3,steps);
            
            pos1 = startTr(1:3,4)';
            pos2 = goalTr(1:3,4)';
            
            rot1 = tr2rpy(startTr);
            rot2 = tr2rpy(goalTr);
            
            % Trajectory Generation using Trapezoidal velocity profile
            % method
            s = lspb(0,1,steps);
            for i=1:steps
               pos(:,i) = (1-s(i))*pos1 + s(i)*pos2; 
               rot(:,i) = (1-s(i))*rot1 + s(i)*rot2;
            end
            
        end
        
        %% Get Base
        % ======================================================================
        %> @brief GetBase class method returns [4x4] transformation of the
        %> robot's base
        %>
        %> @param self instance of the Fetch class.
        %> @retval base [4x4] transformation of the Robot base
        % ======================================================================
        function base = GetBase(self)
            
            base = self.base;
            
        end
        
        %% GetLinkLinePoints
        % ======================================================================
        %> @brief GetLinkLinePoints class method returns [4x4xn+1]
        %> trasnformation of each link of the robot for the given joint State
        %>
        %> @param self instance of the Fetch class.
        %> @param qMatrix Joint State of the Robot. Must be
        %> either a [1x7] or [7x1] matrix
        %> @retval output [4x4xn+1] transformations. n = number of links
        % ======================================================================
        function output = GetLinkLinePoints(self,qMatrix)
            
            trPoses = linkPoses(self.model,qMatrix);
            
            linkStartPoint = zeros(self.model.n,3);
            linkEndPoint  = zeros(self.model.n,3);
                        
            for i = 1:self.model.n                
                linkStartPoint(i,:) = trPoses(1:3,4,i);
                linkEndPoint(i,:) = trPoses(1:3,4,i+1);              
            end  
            output = {linkStartPoint, linkEndPoint};
        end

    end
    
end

