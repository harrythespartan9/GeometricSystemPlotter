function output = sysf_hybridC_2c1r(input_mode,pathnames)

	% Default arguments
	if ~exist('input_mode','var')
		
		input_mode = 'initialize';
		
	end
		
	%%%%%%%
	
	switch input_mode

		case 'name'

			output = 'Hybrid Contact System: Two Contact One Rotor'; % Display name

		case 'dependency'

			output.dependency = fullfile(pathnames.sysplotterpath,...
                {'Geometry/NLinkChain/',...
                'Physics/LeggedSystems/'});
            
		case 'initialize'

            %%%%%%
            % Define system geometry
            
            g_template.linklengths = [0.0 1]/6;
            
            g1 = g_template;
            g1.baseframe = {'tail'};

            g2 = g_template;
            g2.baseframe = {'tail','end'};
            g2.attach.parent = 1;
%             g2.attach.location = {'tail','end',[cos(4*pi/3) -sin(4*pi/3) 0;sin(4*pi/3) cos(4*pi/3) 0; 0 0 1]};
            g2.attach.location = {'tail','end',[cos(pi) -sin(pi) 0;sin(pi) cos(pi) 0; 0 0 1]};   %4*pi/3         
            
            s.geometry.subchains = {g1, g2};
            
            s.geometry.type = 'branched chain';
            s.geometry.contact = 1; % if this is a hybrid contact system.
            s.geometry.baseframe = 'tail-tip'; 
            % put the baseframe at the origin wrt to the first link eye(3,3) or 'tail-tip'
            s.geometry.length = 1;

            % Contact map domain limits:
            con_lim = [-1, 1];
            dom_margin = 5; 
            dom_thresh = [con_lim(1)-dom_margin*diff(con_lim)/100,...
                con_lim(2)+dom_margin*diff(con_lim)/100];

            %%%%%%
            % Ankle limit amplitude for the legged robot:
            ank = 1; % with a 1 radian ankle limit
            
            %%%
            % Define properties for visualizing the system
            
            % Make a grid of values at which to visualize the system in
            % illustrate_shapespace.
            s.visual.grid_spacing = [-1  0  1];
            
            %%%
            %%%%%%
            % Define system physics
            s.physics.rateLim = [1, 1]; % Add the rate cost in the swing and contact directions

            % Here, we experiment with a single joint angle since all
            % theleg-link angles are linked.
            
            %Functional Local connection and dissipation metric

            s.A = @(alpha,contact) HybridContact_local_connection( ...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha,contact]);                        % Joint angles
            
            s.metric = @(alpha,contact) HybridContact_dissipation_metric(...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha,contact]);                        % Joint angles

                    
			%%%
			%Processing details

			%Range over which to evaluate connection
			s.grid_range = [-ank,ank,dom_thresh(1),dom_thresh(2)];

			%densities for various operations
			s.density.vector = [22 22]; %density to display vector field  [ 11 11]
			s.density.scalar = [22 22]; %density to display scalar functions  [ 11 11]
			s.density.eval = [ 62 62];   %density for function evaluations  [ 31 31]
            s.density.metric_eval = [22 22]; %density for metric evaluation  [ 11 11]
            s.density.finite_element=22;


			%shape space tic locations
			s.tic_locs.x = [-1 0 1]*ank;
			s.tic_locs.y = [con_lim(1) 0 con_lim(2)];

            % Set system type variable for gait optimization
            s.system_type = 'drag'; % need to think about the choices


			%%%%
			%Save the system properties
			output = s;


	end

end

