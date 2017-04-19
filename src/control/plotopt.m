%% GRAPHICAL OPTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Axis label properties
asize = 20;                 % Axis label size

% Tick label properties
tsize = 14;                % Tick label size
tname = 'Helvetica';       % Tick label name

% Line properties
linewidth = 1.5;              % Line width
markersize = 6;            % Marker size

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% CHANGE PROPERTIES
% Figure properties
set(gcf,'Color'         , 'w'          );    

% Tick label properties
set(gca,...
    'TickLabelInterpreter', 'tex'       , ...
    'FontSize'          , tsize       , ...
    'FontName'          , tname       );

% Axis label properties
set(get(gca,'XLabel'),'FontSize',asize) 
set(get(gca,'YLabel'),'FontSize',asize) 
set(get(gca,'ZLabel'),'FontSize',asize) 


% Line properties
%set(get(gca,'Children'), ...
%    'LineWidth'         , linewidth)
%    set(get(gca,'Children'), ...
%    'MarkerSize'        , markersize   )


% Other properties
grid on
box on
