function output = color_CUB

% Spot color for output
output.spot = [0,0,0]/255;

% Secondary color
output.secondary = [150,150,150]/255;
    
% Colormap for surfaces and other "intense color" plots
load('CUBColormap.mat','CUB');
output.colormap = CUB;
            
% Colormap for contour plots
load('CUB_contourColormap.mat','CUB_contour');
output.colormap_contour = CUB_contour;
    
end