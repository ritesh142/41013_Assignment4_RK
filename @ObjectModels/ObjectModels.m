%> @file ObjectModels.m
%> @brief This file contains the implmentation of class named ObjectModels
% ======================================================================
%> @brief This class is for creating instances physical objects in
%> simulated environment. The physcial object is modeled as single
%> SerialLink object within the class. 

%> Note: The SerialLink object is not publicly accessable.
%> All the class methods and properties avaliable from SerialLink object
%> can not be accessed.
%> This class contains methods for generating and simulating  a 'realtic'
%> looking 3D model of the phyiscal object. A ply file must be supplied.
% ======================================================================

classdef ObjectModels < handle
    properties (Access = private)
        %> Robot model
        model;
        
        %> Location of the ply file for the 3D model of the object
        plyFile;
        
       %> Define the boundaries of the workspace
        workspace = [-2 2 -2 2 -2 2];
        
    end
    
    properties
        %> Name of the object
        name;
        
        %> 4x4 homogeneous matrix of the Object's rest position
       rest;
    end
    
    methods%% Class for Object simulation
        % ======================================================================
        %> @brief Class constructor
        %>
        %> The constructor does creates a object model using
        %> SerialLink class. It uses the input parameters to assign the
        %> name and base location and saves the locaiton of the spcified ply file. 
        %>
        %> @param objectName Name of the object being created. 
        %> @param baseTr 4X4 Homogeneous Transformation of the object
        %> location. Note baseTr will also be set as the 'Rest Position' of
        %> the object
        %> @param plyFileName Ply file location file path
        %> @return instance of the ObjectModel class.
        % ======================================================================
        function self = ObjectModels(objectName,baseTr,plyFileName)

        self.CreateObject(objectName,baseTr);
        self.name = objectName;
        self.plyFile = plyFileName;
        self.rest = baseTr;

        end


        %% CreateObject
        function CreateObject(self,objectName,baseTr)

            self.name = objectName;

            % DH Parameters - treated as single link robot

            L = Link('revolute','d',0,'a',0,'alpha',0,'offset',0, 'qlim', [-pi, pi]);

            self.model = SerialLink(L,'name',objectName,'base',baseTr);

        end

        %% PlotAndColour
        % Given a object index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        % ======================================================================
        %> @brief The PlotAndColour class method loads the ply files from
        %> the saved file path
        %> Then the class method creates a 3D plot of the object with 3D Model
        %>
        %> @param self instance of the ObjectModel class.
        % ======================================================================
        function PlotAndColour(self)
            
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = ...
                    plyread(self.plyFile,'tri'); %#ok<AGROW>
                
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            

            % Display Object
            self.model.plot3d(zeros(1,self.model.n),'workspace',...
                            self.workspace,'arrow','tile1color',[1 1 1]);
%             if isempty(findobj(get(gca,'Children'),'Type','Light'))
%                 camlight
%             end  
            self.model.delay = 0;

            %Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData =...
                            [plyData{linkIndex+1}.vertex.red ...
                          , plyData{linkIndex+1}.vertex.green ...
                          , plyData{linkIndex+1}.vertex.blue]/255;
                      
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end    


        %% SetPose
        % ======================================================================
        %> @brief The SetPose class method Moves (changes t he position) the object
        %> Note: This class method does not update the 3D plot if the object is
        %> plotted. 
        %>
        %> @param self instance of the ObjectModel class.
        %> @param poseTransformation 4x4 homogeneous matrix of the Object
        %> position and orientation.
        % ======================================================================
        function SetPose(self,poseTransformation)
            self.model.base = poseTransformation; 
        end


        %% GetCurrentPose
        % Function returns the base transformation of the object
        % Returns a 4x4 homogeneous matrix
        % ======================================================================
        %> @brief The GetCurrentPose class method return the current
        %> location of the object.
        %>
        %> @param self instance of the ObjectModel class.
        %>
        %> @retval currentPose 4x4 homogeneous matrix of the Object current
        %> location
        % ======================================================================
        function currentPose = GetCurrentPose(self)
           currentPose = self.model.base; 
        end
        
        %% GetRestPose
        % Function returns the rest transformation of the object
        % Returns a 4x4 homogeneous matrix
        % ======================================================================
        %> @brief The GetRestPose class method return the resting
        %(initial) location of the object.
        %>
        %> @param self instance of the ObjectModel class.
        %>
        %> @retval RestPose 4x4 homogeneous matrix of the Object rest
        %> location
        % ======================================================================
        function restPose = GetRestPose(self)
            restPose = self.rest;
        end
        


        %% UpdatePlot
        % This functions updates the plot with current location of the object
        % If object is plotted, the location of the object will not update
        % Even if the location of the object has changed, this method must
        % called to visulise the new position and orientiation of the
        % object
        % ======================================================================
        %> @brief The UpdatePlot class method updates the plot with
        %> current location of the object.
        %> If object is plotted, the location of the object will not update
        %> Even if the location of the object has changed, this method must
        %> called to visulise the new position and orientiation of the
        %> object
        %>
        %> @param self instance of the ObjectModel class.
        % ======================================================================
       
        function UpdatePlot(self)
            try 
                self.model.animate(zeros(1,self.model.n));
            catch
                disp('No Visual Avaliable to Update');
            end
        end
        
        
        %% GetFaceandVertice
        % ======================================================================
        %> @brief The GetFaceandVertice class method returns the number of
        %> faces and vertice data of the Object model from its ply file.
        %>
        %> @param self instance of the ObjectModel class.
        %>
        %> @retval [faceData, verticeData] 
        %> faceData is Nx3 matrix which reveals which 3 vertice form that
        %> face. N is the number of faces
        %> verticeData is Mx3 matrix which contains the XYZ location of
        %> each vertice relative to the base of the object. M is number of
        %> vertice
        % ======================================================================
        function [faceData, verticeData] = GetFaceandVertice(self)
            
            faceData = self.model.faces{1};
            verticeData= self.model.points{1};
                   
            if tr2rpy(self.model.base) == 0
                % No transformation needed 
            else              
                verticeData = verticeData* rpy2r(tr2rpy(self.model.base)); 
            end
            
            if self.model.base(1:3,4) == 0              
                % No transformation needed  
            else
                verticeData(:,1) = verticeData(:,1) + self.model.base(1,4);
                verticeData(:,2) = verticeData(:,2) + self.model.base(2,4);
                verticeData(:,3) = verticeData(:,3) + self.model.base(3,4);     
            end

        end

    end
end