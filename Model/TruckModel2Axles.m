%%model of two axles
classdef TruckModel2Axles <handle
    properties
        m = 0.0  % the mass of truck
        v = 0.0  % the speed of truck
        C1 = 0.0 %truck Axle 1 Cornering Stiffness
        C2 = 0.0 %truck Axle 1 Cornering Stiffness
        b1 = 0.0 %distance from Axle 1 to COM
        b2 = 0.0 %distance from Axle 1 to COM
        I1 = 0.0  %truck Inertia Moment
        num_states = 4 %number of states:lateral_error,lateral_error_rate,heading_error,heading_error_rate
    end
    methods
        function out = SetMatrixM(obj)
            mat1 = [1, 0, 0, 0];
            mat2 = [0, obj.m, 0, 0];
            mat3 = [0, 0, 1, 0];
            mat4 = [0, 0, 0, obj.I1];
            out = [mat1;mat2;mat3;mat4];
        end
        function out = SetMatrixA(obj)
            ea = (-obj.C1-obj.C2)/obj.v;
            eb = obj.C1+obj.C2;
            ec = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.v;

            eg = (-obj.b1*obj.C1+obj.b2*obj.C2)/obj.v;
            eh = obj.b1*obj.C1-obj.b2*obj.C2;
            ei = (-obj.C1*(obj.b1^2)-obj.C2*(obj.b2^2))/obj.v;
           
            mat1 = [0, 1, 0, 0];
            mat2 = [0, ea, eb, ec];
            mat3 = [0 ,0 , 0, 1];
            mat4 = [0, eg, eh, ei];
            out = [mat1;mat2;mat3;mat4];
        end
        function out = SetMatrixC(obj)
            out = [0; obj.C1; 0; obj.b1*obj.C1;];
        end
        function out = SetParameters(obj,m,Vx,b1,b2,C1,C2,I1)
            obj.m  = m;
            obj.v   = Vx;
            obj.b1 = b1;
            obj.b2 = b2;
            obj.C1 = C1;
            obj.C2 = C2;
            obj.I1  = I1;
        end
        function val = SetMatrixAd(obj,ts)
            val = (eye(obj.num_states) + 0.5*inv(obj.SetMatrixM)*obj.SetMatrixA*ts)*inv(eye(obj.num_states) - 0.5*inv(obj.SetMatrixM)*obj.SetMatrixA*ts);
        end
        function val = SetMatrixCd(obj,ts)
            val = inv(obj.SetMatrixM)*obj.SetMatrixC*ts;
        end
        
    end
        
end