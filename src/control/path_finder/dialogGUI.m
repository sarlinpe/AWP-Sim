function [ ret ] = dialogGUI( )

    choice = questdlg('What next ?','Finished','Plan again','Exit','Plan again');
    switch choice
        case 'Exit'
            ret = 0;
        otherwise
            ret = 1;
    end


end

