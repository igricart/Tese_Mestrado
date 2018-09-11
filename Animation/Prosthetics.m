classdef Prosthetics < handle
    % This class implements methods for plotting and simulation of a
    % planar leg robot on SE(2).
    
    properties
        l_base_hip;
        l_hip_thigh;
        l_thigh_shin;
        l_shin_foot;
        q0 = [0 0 0 0]';
        qt = 0;
        spatial_res = 0.01;
        joint_radius = 0.05;
        link_color = 'b';
        joint_colors = 'g';
        axis_range = [-4 4 -4 4];
        currentFigure
        currentAxes
        ground_size
        ground_color = 'r';
        s_z
        
        base_hip_Line
        hip_thigh_Line
        thigh_shin_Line
        shin_foot_Line
        foot_H_Line
        foot_T_Line
        H_T_Line
        ground_Line
        
        base_Circle
        hip_Circle
        thigh_Circle
        shin_Circle
        foot_Circle
        H_Circle
        T_Circle
    end
    
    methods
        %% Constructor of class
        function obj = Prosthetics( l_base_hip, l_hip_thigh, l_thigh_shin, l_shin_foot, contact_point_h, contact_point_t, s_z, varargin )
            % Check variable length entries.
            if mod(length(varargin),2) == 0
                for i = 1:length(varargin)
                    if mod(i,2) ~= 0
                        % Switch for properties
                        switch varargin{i}
                            case 'SpatialRes'
                                obj.spatial_res = varargin{i+1};
                            case 'JointRadius'
                                obj.joint_radius = varargin{i+1};
                            case 'LinkColor'
                                obj.link_color = varargin{i+1};
                            case 'JointColor'
                                obj.joint_colors = varargin{i+1};
                            case 'AxisRange'
                                obj.axis_range = varargin{i+1};
                            case 'GroundColor'
                                obj.ground_color = varargin{i+1};
                            otherwise
                                disp('No property with this name.')
                        end
                    end
                end
            else
                disp('Specify property value.')
            end

            obj.currentFigure = figure('visible','off'); hold on;
            set(obj.currentFigure, 'Position', [100 100 500 500]);
            obj.currentAxes = gca;
            set(obj.currentAxes,'Ydir','reverse')
            obj.l_base_hip = l_base_hip;
            obj.l_hip_thigh = l_hip_thigh;
            obj.l_thigh_shin = l_thigh_shin;
            obj.l_shin_foot = l_shin_foot;
            obj.s_z = s_z;
            
            % Initialize graphical objects.
            base_line = 0:(obj.spatial_res):1;
            obj.base_hip_Line  = plot(norm(obj.l_base_hip)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.hip_thigh_Line  = plot(norm(obj.l_hip_thigh)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.thigh_shin_Line  = plot(norm(obj.l_thigh_shin)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.shin_foot_Line    = plot(norm(obj.l_shin_foot)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.foot_H_Line = plot(norm(contact_point_h)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.foot_T_Line = plot(norm(contact_point_t)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.H_T_Line = plot(norm(contact_point_h - contact_point_t)*base_line, zeros(length(base_line),1), obj.link_color);
            obj.ground_Line = plot(base_line, zeros(length(base_line),1), obj.ground_color);
            
            obj.base_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.hip_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.thigh_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.shin_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.foot_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.H_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            obj.T_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            clf; hold off; axis square;
        end
        
        %% Plot the Prosthetic Robot with specified coordinates
        function PlotProsthetics( obj, q, L, h, contact_point_h, contact_point_t, joint_type )
            clf
            % Compute direct kinematics
            if ~isvalid(obj.currentFigure)
               obj.currentFigure = figure(1);
            else
                set(obj.currentFigure,'Visible','on');
            end
            set(obj.currentFigure, 'Position', [100 100 600 600]);
            hold on; grid on;
            obj.currentAxes = gca; 
            axis(obj.axis_range); 
            axis square;
            set(obj.currentAxes,'Ydir','reverse');
%             set(obj.currentAxes,'Xdir','reverse');
            
            %% Computes the joints and point positions in the animation
            [ P_hip, ~ ] = prosthetics_forward_kinematics( q, L, h, 1, zeros(3,1), joint_type );
            [ P_thigh, ~ ] = prosthetics_forward_kinematics( q, L, h, 2, zeros(3,1), joint_type );
            [ P_shin, ~ ] = prosthetics_forward_kinematics( q, L, h, 3, zeros(3,1), joint_type );
            [ P_foot, ~ ] = prosthetics_forward_kinematics( q, L, h, 4, zeros(3,1), joint_type );
            [ P_H, ~ ] = prosthetics_forward_kinematics( q, L, h, 4, contact_point_h, joint_type );
            [ P_T, ~ ] = prosthetics_forward_kinematics( q, L, h, 4, contact_point_t, joint_type );
            
            % If some of the object handles no longer exists...
            any = (~isvalid(obj.base_hip_Line))||(~isvalid(obj.hip_thigh_Line))||(~isvalid(obj.thigh_shin_Line))||(~isvalid(obj.shin_foot_Line))||(~isvalid(obj.currentAxes));
            if any
                base_line = 0:(obj.spatial_res):1;
                obj.base_hip_Line = plot(norm(obj.l_base_hip)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.hip_thigh_Line = plot(norm(obj.l_hip_thigh)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.thigh_shin_Line = plot(norm(obj.l_thigh_shin)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.shin_foot_Line = plot(norm(obj.l_shin_foot)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.foot_H_Line = plot(norm(contact_point_h)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.foot_T_Line = plot(norm(contact_point_t)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.H_T_Line = plot(norm(contact_point_h - contact_point_t)*base_line, zeros(length(base_line),1), obj.link_color);
                obj.ground_Line = plot(base_line, zeros(length(base_line),1), obj.ground_color);
                
                obj.base_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.hip_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.thigh_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.shin_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.foot_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.H_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
                obj.T_Circle = rectangle('Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius],'Curvature',[1,1], 'FaceColor',obj.joint_colors);
            end
            
            [ X,Y ] = drawLine( zeros(2,1), [ P_hip(1) ; P_hip(3) ] );
            set( obj.base_hip_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_hip(1) ; P_hip(3) ], [ P_thigh(1) ; P_thigh(3) ] );
            set(  obj.hip_thigh_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_thigh(1) ; P_thigh(3) ], [ P_shin(1) ; P_shin(3) ] );
            set(  obj.thigh_shin_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_shin(1) ; P_shin(3) ], [ P_foot(1) ; P_foot(3) ] );
            set(  obj.shin_foot_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_foot(1) ; P_foot(3) ], [ P_H(1) ; P_H(3) ] );
            set(  obj.foot_H_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_foot(1) ; P_foot(3) ], [ P_T(1) ; P_T(3) ] );
            set(  obj.foot_T_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ P_H(1) ; P_H(3) ], [ P_T(1) ; P_T(3) ] );
            set(  obj.H_T_Line, 'XData', X, 'YData', Y );
            [ X,Y ] = drawLine( [ obj.axis_range(1)  ; obj.s_z ], [ obj.axis_range(2) ; obj.s_z ] );
            set(  obj.ground_Line, 'XData', X, 'YData', Y );
            
            set(obj.base_Circle,'Position',[0-obj.joint_radius,0-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.hip_Circle,'Position',[P_hip(1)-obj.joint_radius,P_hip(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.thigh_Circle,'Position',[P_thigh(1)-obj.joint_radius,P_thigh(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.shin_Circle,'Position',[P_shin(1)-obj.joint_radius,P_shin(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.foot_Circle,'Position',[P_foot(1)-obj.joint_radius,P_foot(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.H_Circle,'Position',[P_H(1)-obj.joint_radius,P_H(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            set(obj.T_Circle,'Position',[P_T(1)-obj.joint_radius,P_T(3)-obj.joint_radius,2*obj.joint_radius,2*obj.joint_radius]);
            hold off;
        end
    end
    
end

