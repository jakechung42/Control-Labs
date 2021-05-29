classdef makeFilters
    %Class to create filters of different kinds
    properties
        R
        C
        wb
        fb = wb/(2*pi) %break frequency
    end
    methods (Static)
        function R = makeRC(fb, C)
            %function to calculate R for RC filter
            R = 1/(2*pi*fb*C);
        end
        function R = make2ndRC(fb, C)
            %function to calculate R for a second order RC filter
            R = 0.3742/(2*pi*fb*C);
        end
    end
end