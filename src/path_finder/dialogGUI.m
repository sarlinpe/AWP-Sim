% Title:        ME5402 Project 1-2: Trajectory Planning
% File:         dialogGUI.m
% Date:         2017-04-20
% Author:       Nicolai Domingo Nielsen (A0164015R)
%               Paul-Edouard Sarlin (A0153124U)
% Description:  A simple dialog box for interaction with the user.

function [ ret ] = dialogGUI( )

    choice = questdlg('What next ?','Finished','Plan again','Exit','Plan again');
    switch choice
        case 'Exit'
            ret = 0;
        otherwise
            ret = 1;
    end


end

