function output = sysf_trefoil_double_lowRe(input_mode,pathnames)

	% Default arguments
	if ~exist('input_mode','var')
		
		input_mode = 'initialize';
		
	end
		
	%%%%%%%
	
	switch input_mode

		case 'name'

			output = 'Viscous Swimmer: Trefoil double'; % Display name

		case 'dependency'

			output.dependency = fullfile(pathnames.sysplotterpath,...
                {'Geometry/NLinkChain/',...
                'Physics/LowReynoldsRFT/'});
            
		case 'initialize'

            %%%%%%
            % Define system geometry
            
            g_template.linklengths = [1 .5 .5]/6;
            g_template.modes = [1;1];
            
            g1 = g_template;
            g1.baseframe = {'tail'};
            
            
            g2 = g_template;
            g2.baseframe = {'tail','start'};
            g2.attach.parent = 1;
            g2.attach.location = {'tail','start',[cos(2*pi/3) -sin(2*pi/3) 0;sin(2*pi/3) cos(2*pi/3) 0; 0 0 1]};
 
            g3 = g_template;
            g3.baseframe = {'tail','start'};
            g3.attach.parent = 1;
            g3.attach.location = {'tail','start',[cos(4*pi/3) -sin(4*pi/3) 0;sin(4*pi/3) cos(4*pi/3) 0; 0 0 1]};

            
            
            s.geometry.subchains = {g1, g2, g3};
            
            
            s.geometry.type = 'branched chain';
            s.geometry.baseframe = 'tail-tip';
            s.geometry.length = 1;
            
            
            %%%
            % Define properties for visualizing the system
            
            % Make a grid of values at which to visualize the system in
            % illustrate_shapespace. The code below uses properties of cell
            % arrays to automatically match the dimensionality of the grid
            % with the number of shape basis functions in use
            s.visual.grid = cell(numel(s.geometry.subchains),1);
            [s.visual.grid{:}] = ndgrid([-1  0  1]);

            
            %%%
            %%%%%%
            % Define system physics
            s.physics.drag_ratio = 2;
            s.physics.drag_coefficient = 1;
           
 
            %Functional Local connection and dissipation metric

            s.A = @(alpha1,alpha2,alpha3) LowRE_local_connection( ...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha1,alpha2,alpha3]);                        % Joint angles
            
            s.metric = @(alpha1,alpha2,alpha3) LowRE_dissipation_metric(...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha1,alpha2,alpha3]);                        % Joint angles

                    
			%%%
			%Processing details

			%Range over which to evaluate connection
			s.grid_range = [-1,1,-1,1,-1,1]*1.5;

			%densities for various operations
			s.density.vector = [11 11 11]; %density to display vector field
			s.density.scalar = [51 51 51]; %density to display scalar functions
			s.density.eval = [31 31 31];   %density for function evaluations
            s.density.metric_eval = [11 11 11]; %density for metric evaluation
            s.density.finite_element=11;


			%shape space tic locations
			s.tic_locs.x = [-1 0 1]*1;
			s.tic_locs.y = [-1 0 1]*1;


			%%%%
			%Save the system properties
			output = s;


	end

end

