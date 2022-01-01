function output = shchf_2DlegAngle_diametricallyOpp_constVel(input_mode,pathnames)

	% Default argument
	if ~exist('input_mode','var')
		
		input_mode = 'initialize';
		
	end	
	
	switch input_mode
		
		case 'name'
			
			output = 'Const Leg Angular Vel: Triangular Alpha, Amp = pi/4';
			
		case 'dependency'
			
			output.dependency = {};
			
		case 'initialize'

			%%%%
			% Filename to save to

			%%
			%Path definitions

			%path definition
			p.phi_def = @strokedef;
			
			%marker locations
			p.phi_marker = [];
			
			%arrows to plot
			p.phi_arrows = 2;

			%time to run path
			p.time_def = [0 2*pi];

			p.cBVI_method = 'simple';

			%path resolution
			p.phi_res = 101; 
            % keep this odd since we will have a distinct mid stroke value


			%%%%
			%Output the path properties
			output = p;
	end
	
end

function [stroke] = strokedef(t)
    
    % Let's define our initial condition and amplitude,
    stroke0 = pi/4; strokeHW = -pi/4; % HW stands for half-way

    % Define the half stroke:
    stroke_half = linspace(stroke0, strokeHW, ceil(length(t)/2));

    % Now, we define the stroke assuming we have constant leg angle rate
    stroke(:) = [stroke_half, ...
        fliplr(stroke_half(1:end-1))]';

    % And, we now extend this to the other leg as follows,
    stroke = repmat(stroke, 1, 2); stroke(:,2) = pi + stroke(:,2);

end