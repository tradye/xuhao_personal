classdef class_test <handle
    properties
        a = 0
        b = 0
        c = 0
    end
    properties(Access=private)
        d = 0
    end
    methods
        function obj = class_test(a,b,c)
            obj.a = a;
            obj.b = b;
            obj.c = c;
            obj.d = a +b+c;
        end
        function out = compute(obj)
            obj.a = 9;
            out = obj.a + 2;
        end
        
        function out = add(obj,d,e,f)
            bb = obj.compute;
            obj.c = 4;
            out = obj.d + d +e+f;
        end
        
    end
    
    
    
end









