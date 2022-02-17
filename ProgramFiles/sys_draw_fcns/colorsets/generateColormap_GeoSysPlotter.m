% Colormap for surfaces and other "intense color" plots
CUB_contour = flipud([140,81,10;
            191,129,45;
            223,194,125;
            246,232,195;
            247,247,247;
            204,204,204;
            150,150,150;
            99,99,99;
            37,37,37])*(1/255);

CUB_contour = interp1(linspace(0,100,size(CUB_contour,1)), CUB_contour, linspace(0,100,251));

% Colormap for contour plots
CUB = flipud([140,81,10;
            191,129,45;
            223,194,125;
            246,232,195;
            204,204,204;
            150,150,150;
            99,99,99;
            37,37,37])*(1/255);

CUB = interp1(linspace(0,100,size(CUB,1)), CUB, linspace(0,100,251));

% Save these as separate files to call in the 'color_**..' file:
save('CUBColormap.mat','CUB');
save('CUB_contourColormap.mat','CUB_contour');




