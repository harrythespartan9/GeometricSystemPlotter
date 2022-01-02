% Colormap for surfaces and other "intense color" plots
CUB_contour = [178,24,43;
                214,96,77;
                244,165,130;
                253,219,199;
                247,247,247;
                209,229,240;
                146,197,222;
                67,147,195;
                33,102,172]*(1/255);

CUB_contour = interp1(linspace(0,100,size(CUB_contour,1)), CUB_contour, linspace(0,100,251));

% Colormap for contour plots
CUB = [215,48,39;
        244,109,67;
        253,174,97;
        254,224,144;
        255,255,191;
        224,243,248;
        171,217,233;
        116,173,209;
        69,117,180]*(1/255);

CUB = interp1(linspace(0,100,size(CUB,1)), CUB, linspace(0,100,251));

% Save these as separate files to call in the 'color_**..' file:
save('CUBColormap.mat','CUB');
save('CUB_contourColormap.mat','CUB_contour');




