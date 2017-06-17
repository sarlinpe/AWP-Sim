% Title:        AWP Trajectory Planning and Control
% File:         dialogGUI.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
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

