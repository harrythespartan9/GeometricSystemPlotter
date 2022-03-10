function output = sysf_hybridC_2c2r_multidisk(input_mode,pathnames)

	% Default arguments
	if ~exist('input_mode','var')
		
		input_mode = 'initialize';
		
	end
		
	%%%%%%%
	
	switch input_mode

		case 'name'

			output = 'Contact-switching Multi-disk System: Two Contact Two Rotors'; % Display name

		case 'dependency'

			output.dependency = fullfile(pathnames.sysplotterpath,...
                {'Geometry/NLinkChain/',...
                'Physics/LeggedSystems/'});
            
		case 'initialize'

            %%%%%%

            % Contact -- we shall always we dealing with {1,1} contact
            % state in this system
            s.geometry.type = 'multi_disk_11';

            %%%%%%
            % Ankle limit amplitude for the legged robot:
            ank = pi/4;
            
            %%%
            %%%%%%
            % Define system physics
            s.physics.rateLim = [1, 1]; % Add the rate cost in the two 
                                        % swing directions let them be 
                                        % equal for now.
            
            %Functional Local connection and dissipation metric

            s.A = @(alpha,contact) HybridContact_local_connection( ...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha1,alpha2]);                        % Joint angles
            
            s.metric = @(alpha,contact) HybridContact_dissipation_metric(...
                        s.geometry,...                           % Geometry of body
                        s.physics,...                            % Physics properties
                        [alpha1,alpha2]);                        % Joint angles

                    
			%%%
			%Processing details

			%Range over which to evaluate connection
			s.grid_range = [-ank,ank,-ank,ank];

			%densities for various operations
			s.density.vector = [ 11 11]; %density to display vector field  [ 11 11]
			s.density.scalar = [ 11 11]; %density to display scalar functions  [ 11 11]
			s.density.eval = [ 31 31];   %density for function evaluations  [ 31 31]
            s.density.metric_eval = [ 11 11]; %density for metric evaluation  [ 11 11]
            s.density.finite_element=11;
%             %densities for various operations
% 			s.density.vector = [44 44]; %density to display vector field  [ 11 11]
% 			s.density.scalar = [44 44]; %density to display scalar functions  [ 11 11]
% 			s.density.eval = [93 93];   %density for function evaluations  [ 31 31]
%             s.density.metric_eval = [44 44]; %density for metric evaluation  [ 11 11]
%             s.density.finite_element=44;


			%shape space tic locations
			s.tic_locs.x = [-1 0 1]*ank;
			s.tic_locs.y = s.tic_locs.x;

%             % Set system type variable for gait optimization
%             s.system_type = 'drag'; % need to think about the choices


			%%%%
			%Save the system properties
			output = s;


	end

end

