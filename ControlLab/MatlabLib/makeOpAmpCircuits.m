classdef makeOpAmpCircuits
    %Class to create filters of different kinds
    properties
        R
        C
        A %gain
    end
    methods (Static)
        function R2 = makeInv(A, R1)
            %function to calculate R2 for Inverting opamp circuit
            if A > 0
                R2 = A*R1;
            else
                R2 = -A*R1;
            end
        end
        function R2 = makeNonInv(A, R1)
            %function to calculate R2 for non inverting opamp circuit
            if A < 1
                fprintf('Non-inverting opamp, A cannot be less than 1')
                return
            end
            R2 = R1*(A-1);
        end
        function R = makePureIntegrator(A, C)
            %function to calculate R value for a pure integrator circuit
            R = 1/(A*C);
        end
        function [R1, R2] = makeModIntegrator(A, fb, C)
            %function to calculate R values for a modified integrator
            %circuit
            R2 = 1/(2*pi*fb*C);
            R1 = R2/A;
        end
    end
end