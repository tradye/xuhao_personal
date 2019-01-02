%% navi planner:speed planning matlab version
%    author:xuhao
%    data:2018.12

classdef NaviSpeedTsGraph <handle
    properties
        s_step_ = 0.0  % 
        start_v_ = 0.0  % 
        start_a_ = 0.0 %
        start_da_ = 0.0 %
        cap_saved_ratio_ = 0.0 %
        t_ = []
        s_max_ =[]
        s_p_ =[]
        v_max_ = []
        v_p_ = []
        a_max_ = []
        a_min_ = []
        a_p_max_ = []
        a_p_min_ = []
        da_min_ = []
        da_max_ = []
        da_p_max_ = []
        da_p_min_ = []
        dda_max_ = []
        dda_min_ = []
        dda_p_max_ = []
        dda_p_min_ = []
        constraints_ 
        st_
        output_
        
    end
    methods
        function obj=NaviSpeedTsGraph(s_step,start_v,start_a)
            obj.s_step_ = s_step;
            obj.start_v_ = start_v;
            obj.start_a_ = start_a;
            
        end
        function Reset(obj,s_step,s_max,t_step,t_max,compress_t,cap_saved_ratio,start_v,start_a,start_da)
            obj.s_step_ = s_step;
            obj.start_v_ = start_v;
            obj.start_a_ = start_a;
            obj.start_da_ = start_da;
            
            obj.cap_saved_ratio_ = min(cap_saved_ratio, 1.0);
%             s_point_num = uint32((s_max + obj.s_step_) / obj.s_step_);
            s_point_num = uint32((s_max ) / obj.s_step_);
            fprintf('num of struct constraints_ is: %d \n' , s_point_num);
%             obj.constraints_ = zeros(s_point_num,1);
            MakeConstrainStructArray(obj,s_point_num);%init struct array constraints_
            
            if(~compress_t)
                t_num = uint32(( t_max + t_step) / (t_step)) ;
                obj.t_ = zeros(t_num,1);
                for i = 1:1:length(obj.t_)
                    obj.t_(i) = i * t_step;
                end
            else
                t_num1 = 3 * uint32((t_max + t_step) / (25.0 * t_step));
                fprintf('t_num1: %d \n' , t_num1);
                obj.t_ = zeros(t_num1,1);
                obj.t_(1) = 0.0;
                for i = 2:1:(t_num1/3)
                    obj.t_(i) = obj.t_(i-1) + t_step;
                end
                for i = (t_num1/3 +1):1:(t_num1*2/3)
                    obj.t_(i) = obj.t_(i-1) +8* t_step;
                end
                for i = (t_num1*2/3 + 1):1:(t_num1)
                    obj.t_(i) = obj.t_(i-1) +16* t_step;
                end
            end
            fprintf('length t_ : %d \n' , length(obj.t_));
            obj.s_max_ = 1000*ones(length(obj.t_),1);
            obj.s_p_ = 1000*ones(length(obj.t_),1);
                    
                
        end% end Reset function
        function InitConstraintsTables(obj)
            s_point_num = length(obj.constraints_);
            obj.v_max_ = zeros(s_point_num,1);
            obj.v_p_ = zeros(s_point_num,1);
            obj.a_max_ = zeros(s_point_num,1);
            obj.a_min_ =zeros(s_point_num,1);
            obj.a_p_max_ =zeros(s_point_num,1);
            obj.a_p_min_ = zeros(s_point_num,1);
            obj.da_min_ = zeros(s_point_num,1);
            obj.da_max_ = zeros(s_point_num,1);
            obj.da_p_max_ = zeros(s_point_num,1);
            obj.da_p_min_ = zeros(s_point_num,1);
            obj.dda_max_ = zeros(s_point_num,1);
            obj.dda_min_ = zeros(s_point_num,1);
            obj.dda_p_max_ =zeros(s_point_num,1);
            obj.dda_p_min_ = zeros(s_point_num,1);
            
            for i = 1:1:s_point_num
%                 con = MakeSingleConstrainStruct();
                obj.v_max_(i) = obj.constraints_(i).v_max;
                obj.v_p_(i) = obj.constraints_(i).v_preffered;
                obj.a_min_(i) = -obj.constraints_(i).b_max;
                obj.a_max_(i) = obj.constraints_(i).a_max;
                obj.a_p_min_(i) = -obj.constraints_(i).b_preffered;
                obj.a_p_max_(i) = obj.constraints_(i).a_preffered;
                obj.da_min_(i) = -obj.constraints_(i).da_max;
                obj.da_max_(i) = obj.constraints_(i).da_max;
                obj.da_p_min_(i) = -obj.constraints_(i).da_preffered;
                obj.da_p_max_(i) = obj.constraints_(i).da_preffered;
                obj.dda_min_(i) = -obj.constraints_(i).dda_max;
                obj.dda_max_(i) = obj.constraints_(i).dda_max;
                obj.dda_p_min_(i) = -obj.constraints_(i).dda_preffered;
                obj.dda_p_max_(i) = obj.constraints_(i).dda_preffered;
            end
        end% end InitConstraintsTables function
        
        function InitStTable(obj)
%             s_point_num = length(obj.s_max_);%
            num = length(obj.s_max_);
%             MakeStPointsStructArray(obj , num);
            obj.st_ = MakeStPointsStructArray(obj , num);
            obj.st_(1).s_max = 0;
            obj.st_(1).s_min = 0;
            obj.st_(1).s_p_max = 0;
            obj.st_(1).s_p_min = 0;
            obj.st_(1).s = 0;
            obj.st_(1).v = obj.start_v_;
            obj.st_(1).a = obj.start_a_;
            obj.st_(1).da= obj.start_da_;
            obj.st_(1).dda = 0;
            obj.st_(1).adjust = true;
            
        end% end InitStTable function
        function PreprocessSConstraints(obj)
            for i = 1:1:length(obj.t_)
                obj.s_p_(i) = obj.s_p_(i) * (1 - obj.cap_saved_ratio_);
            end
        end%end PreprocessSConstraints function
        function out =  MakeSingleConstrainStruct(obj)
            field1 = 'v_max'; value1 = 0;
            field2 = 'v_preffered'; value2 = 0;
            field3 = 'a_max'; value3 = 0;
            field4 = 'a_preffered'; value4 = 0;
            field5 = 'b_max'; value5 = 0;
            field6 = 'b_preffered'; value6 = 0;
            field7 = 'da_max'; value7 = 0;
            field8 = 'da_preffered'; value8 = 0;
            field9 = 'dda_max'; value9 = 0;
            field10 = 'dda_preffered'; value10 = 0;
            out=struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
                field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);
        end
        
        
        function  MakeConstrainStructArray(obj,sizeofconstraints)
            num = sizeofconstraints;
            field1 = 'v_max'; value1 = zeros(num,1);
            field2 = 'v_preffered'; value2 = zeros(num,1);
            field3 = 'a_max'; value3 = zeros(num,1);
            field4 = 'a_preffered'; value4 = zeros(num,1);
            field5 = 'b_max'; value5 = zeros(num,1);
            field6 = 'b_preffered'; value6 = zeros(num,1);
            field7 = 'da_max'; value7 = zeros(num,1);
            field8 = 'da_preffered'; value8 = zeros(num,1);
            field9 = 'dda_max'; value9 = zeros(num,1);
            field10 = 'dda_preffered'; value10 = zeros(num,1);
%             member = { 'v_max' , 'v_preffered' ,  'a_max' , 'a_preffered',  'b_max' , 'b_preffered' , ...
%                                    'da_max' , 'da_preffered' ,  'dda_max' , 'dda_preffered' };
%             obj.constraints_=struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                 field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);

%             field1 = 'v_max'; value1 = 0.0;
%             field2 = 'v_preffered'; value2 = 0.0;
%             field3 = 'a_max'; value3 = 0.0;
%             field4 = 'a_preffered'; value4 = 0.0;
%             field5 = 'b_max'; value5 = 0.0;
%             field6 = 'b_preffered'; value6 = 0.0;
%             field7 = 'da_max'; value7 = 0.0;
%             field8 = 'da_preffered'; value8 = 0.0;
%             field9 = 'dda_max'; value9 = 0.0;
%             field10 = 'dda_preffered'; value10 = 0.0;
%             constrain_struct_=struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                 field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);
            for i = 1:1:num
                obj.constraints_(i). v_max= value1(i);
                obj.constraints_(i). v_preffered= value2(i);
                obj.constraints_(i). a_max= value3(i);
                obj.constraints_(i). a_preffered= value4(i);
                obj.constraints_(i). b_max= value5(i);
                obj.constraints_(i). b_preffered= value6(i);
                obj.constraints_(i). da_max= value7(i);
                obj.constraints_(i). da_preffered= value8(i);
                obj.constraints_(i). dda_max= value9(i);
                obj.constraints_(i). dda_preffered= value10(i);
            end
            
        end%end MakeConstrainStructArray function
        
        function st_struct = MakeStPointsStructArray(obj,sizeofstpoints)
%             num = length(obj.s_max_);
            num = sizeofstpoints;
            field1 = 's_max'; value1 = zeros(num,1);
            field2 = 's_min'; value2 = zeros(num,1);
            field3 = 's_p_max'; value3 = zeros(num,1);
            field4 = 's_p_min'; value4 = zeros(num,1);
            field5 = 's'; value5 = zeros(num,1);
            field6 = 'v'; value6 = zeros(num,1);
            field7 = 'a'; value7 = zeros(num,1);
            field8 = 'da'; value8 = zeros(num,1);
            field9 = 'dda'; value9 = zeros(num,1);
            field10 = 'adjust'; value10 = zeros(num,1);
%             obj.st_=struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,...
%                 field6,value6,field7,value7,field8,value8,field9,value9,field10,value10);
%             for i = 1:1:num
%                 obj.st_(i). s_max= value1(i);
%                 obj.st_(i). s_min= value2(i);
%                 obj.st_(i). s_p_max= value3(i);
%                 obj.st_(i). s_p_min= value4(i);
%                 obj.st_(i). s= value5(i);
%                 obj.st_(i). v= value6(i);
%                 obj.st_(i). a= value7(i);
%                 obj.st_(i). da= value8(i);
%                 obj.st_(i). dda= value9(i);
%                 obj.st_(i). adjust= value10(i);
%             end
            for i = 1:1:num
                st_struct(i). s_max= value1(i);
                st_struct(i). s_min= value2(i);
                st_struct(i). s_p_max= value3(i);
                st_struct(i). s_p_min= value4(i);
                st_struct(i). s= value5(i);
                st_struct(i). v= value6(i);
                st_struct(i). a= value7(i);
                st_struct(i). da= value8(i);
                st_struct(i). dda= value9(i);
                st_struct(i). adjust= value10(i);
            end
            
        end% end MakeStPointsStructArray function
        
        function speed_ts_struct = MakeNaviSpeedTsPointsStructArray(obj,size)
            num = size;
            field1 = 't'; value1 = zeros(num,1);
            field2 = 's'; value2 = zeros(num,1);
            field3 = 'v'; value3 = zeros(num,1);
            field4 = 'a'; value4 = zeros(num,1);
            field5 = 'da'; value5 = zeros(num,1);
            field6 = 'dda'; value6 = zeros(num,1);
            
             for i = 1:1:num
                speed_ts_struct(i).t= value1(i);
                speed_ts_struct(i).s= value2(i);
                speed_ts_struct(i).v= value3(i);
                speed_ts_struct(i).a= value4(i);
                speed_ts_struct(i).da= value5(i);
                speed_ts_struct(i).dda= value6(i);
            end
        end% end MakeNaviSpeedTsPointsStructArray function
        
        function out = OptimizeStTable(obj)
            mode = 0;
            start_i = 0;
            end_i = 0;
            mode3_start_i = 0;
            
            for i=2:1:length(obj.st_)
                cur_s_p = obj.s_p_(i);
                cur_s_max = obj.s_max_(i);
                cur_v_max = 1000.0;
                cur_a_max = 1000.0;
                cur_a_min = -1000.0;
                cur_da_max = 1000.0;
                cur_da_min = -1000.0;
                cur_dda_max =1000.0;
                cur_dda_min = -1000.0;
                
                [status1 , cur_v_max] = GetConstraint(obj,true,obj.v_max_,obj.st_(i - 1).s);
                [status2 , cur_a_max] = GetConstraint(obj,true,obj.a_max_,obj.st_(i - 1).s);
                [status3 , cur_a_min] = GetConstraint(obj,false,obj.a_min_,obj.st_(i - 1).s);
                [status4 , cur_da_max] = GetConstraint(obj,true,obj.da_max_,obj.st_(i - 1).s);
                [status5 , cur_da_min] = GetConstraint(obj,false,obj.da_min_,obj.st_(i - 1).s); 
                [status6 , cur_dda_max] = GetConstraint(obj,true,obj.dda_max_,obj.st_(i - 1).s);
                [status7 , cur_dda_min] = GetConstraint(obj,true,obj.dda_min_,obj.st_(i - 1).s);
                fprintf('cur_v_max: %f \n' , cur_v_max);
                fprintf('cur_a_max: %f \n' , cur_a_max);
                fprintf('cur_a_min: %f \n' , cur_a_min);
                fprintf('cur_da_max: %f \n' , cur_da_max);
                fprintf('cur_da_min: %f \n' , cur_da_min);
                fprintf('cur_dda_max: %f \n' , cur_dda_max);
                
                
                if (~status1 || ~status2 || ...
                    ~status3 || ~status4 || ...
                    ~status5 || ~status6 || ~status7)
%                 st_.resize(i);
                    break;
                end
                
%                 if (~GetConstraint(true,obj.v_max_,obj.st_(i - 1).s, cur_v_max) || ...
%                     ~GetConstraint(true,obj.a_max_,obj.st_(i - 1).s, cur_a_max) || ...
%                     ~GetConstraint(false,obj.a_min_,obj.st_(i - 1).s, cur_a_min) || ...
%                     ~GetConstraint(true,obj.da_max_,obj.st_(i - 1).s, cur_da_max) || ...
%                     ~GetConstraint(false,obj.da_min_,obj.st_(i - 1).s, cur_da_min) || ...
%                     ~GetConstraint(true,obj.dda_max_,obj.st_(i - 1).s, cur_dda_max) || ...
%                     ~GetConstraint(true,obj.dda_min_,obj.st_(i - 1).s, cur_dda_min))
% %                 st_.resize(i);
%                     break;
%                 end
                
                r = UpdateStPoint(mode, false, cur_s_p , cur_v_max , cur_a_max , cur_a_min,...
                                               cur_da_max , cur_da_min , cur_dda_max , cur_dda_min,i);
                                           
                if(~r && obj.st_(i - 1).adjust)
                    r = UpdateStPoint(mode, true, cur_s_p , cur_v_max , cur_a_max , cur_a_min,...
                                               cur_da_max , cur_da_min , cur_dda_max , cur_dda_min,i);
                end
                
                if(r && obj.st_(i).s > cur_s_max)
                    r = false;
                end
                
                if(r)
                    if(mode == 0)
                        i = i +1;
                    elseif(mode ==1)
                        if(i  == end_i)
                            if(end_i - 1 > 0)
                                mode =2;
                                i = start_i  ;
                            else
                                mode = 0;
                                i = i +1;
                            end
                        else
                            i = i +1;
                        end
                    elseif(mode == 2)
                        if(i == end_i)
                            start_i = start_i - 1;
                            i = start_i;
                        else
                            i = i + 1;
                        end
                    elseif(mode == 3)
                        if(i ==end_i)
                            if(mode3_start_i == 1)
                                mode = 0;
                                i = i +1;
                            else
                                mode = 2;
                                mode3_start_i = mode3_start_i - 1;
                                i = start_i;
                            end
                        else
                            i = i + 1;
                        end
                    else
                        if(mode == 0 )
                            mode = 1;
                            end_i = i;
                            i  = i - 1;
                            start_i = i;
                        elseif mode == 1
                            if (i > start_i + 1)
                                mode = 3;
                                i = i - 1;
                                mode3_start_i = i;
                            else
                                start_i = start_i - 1;
                                i = start_i;
                            end
                        elseif mode == 2
                            mode = 0;
                            obj.st_(start_i).up = false;
                            i = start_i;
                        elseif mode ==3
                            if(mode3_start_i ~= start_i + 1)
                                mode3_start_i = mode3_start_i - 1;
                                i = mode3_start_i;
                            else
                                mode = 1;
                                start_i = start_i - 1;
                                i = start_i;
                            end
                        end
                    end                     
                end             
            end
            out = true;
        end%end OptimizeStTable function
%         function out = GetConstraint(obj, prop ,table ,s ,con)
        function [out , con] = GetConstraint(obj, prop ,table ,s )
            si = s / obj.s_step_;
            i0 = uint32(floor(si));
            i1 = uint32(ceil(si));
            
%             if(i0 < 0 || i1 >= length(table))
            if(i0 < 1 || i1 >= length(table))
                out = false;
                con = table(2);
                return;
            end
            
            if(i0 ==i1)
                con = table(i0);
            else
                if prop
                    con = min(table(i0),table(i1));
                else
                    con = max(table(i0),table(i1));
                end
            end
            out = true;
            
        end%end GetConstraint function
        
        function out = UpdateStPoint(obj,mode,adjust,s_max,v_max,a_max,a_min,...
                                                        da_max,da_min,dda_max,dda_min,i)
            up = true;
            if(mode ==0)
                up = obj.st_(i).up;
            elseif(mode ==1)
                up = false;
            elseif (mode ==2)
                up = true;
            elseif(mode == 3)
                up = true;
                
            end
            
            adjust  = adjust && obj.st_(i -1).adjust;
            
            dt = obj.t_(i) - obj.t_(i - 1);
%             get_s_from_v = obj.st_(i - 1).s + v * dt;
%             get_s_from_a = obj.st_(i-1).s + (obj.st_(i - 1).v + a * dt) * dt;
%             get_s_from_da = obj.st_(i - 1).s + (obj.st_(i - 1).v +(obj.st_(i - 1).a + da *dt) * dt) * dt;
%             get_s_from_dda = obj.st_(i - 1).s + (obj.st_(i - 1).v +...
%                                                     (obj.st_(i - 1).a + (obj.st_(i - 1).da + dda * dt) *dt) * dt) * dt;
                                                
            s_dda_max =  obj.st_(i - 1).s + (obj.st_(i - 1).v +...
                                                    (obj.st_(i - 1).a + (obj.st_(i - 1).da + dda_max * dt) *dt) * dt) * dt; 
            s_dda_min =  obj.st_(i - 1).s + (obj.st_(i - 1).v +...
                                                    (obj.st_(i - 1).a + (obj.st_(i - 1).da + dda_min * dt) *dt) * dt) * dt;                                 
            s_da_max_tmp = obj.st_(i - 1).s + (obj.st_(i - 1).v +(obj.st_(i - 1).a + da_max *dt) * dt) * dt;
            s_da_max = min(s_da_max_tmp,s_dda_max);
            s_da_min_tmp = obj.st_(i - 1).s + (obj.st_(i - 1).v +(obj.st_(i - 1).a + da_min *dt) * dt) * dt;
            s_da_max = max(s_da_min_tmp,s_dda_min);
            
            if(adjust)
                s_a_max = obj.st_(i-1).s + (obj.st_(i - 1).v + max(a_max,obj.st_(i - 1).a) * dt) * dt;
                get_s_from_v_tmp =  obj.st_(i - 1).s + max(v_max,obj.st_(i - 1).a) * dt;
                hard_s_max = min(get_s_from_v_tmp,min(s_a_max,s_da_max));
                hard_s_min = max(max(obj.st_(i - 1).s , (obj.st_(i - 1).s + 0 * dt)),...
                                               max(obj.st_(i-1).s + (obj.st_(i - 1).v + min(a_min , obj.st_(i - 1).a) * dt) * dt , s_da_min));
                
                if(hard_s_max > 0)
                    if(hard_s_max < hard_s_min)
                        hard_s_max  = min(s_a_max , s_da_max);
                        if(hard_s_max < 0)
                            hard_s_max = 0.0;
                            hard_s_min = 0.0;
                        end
                    end
                else
                    hard_s_max = 0.0;
                    hard_s_min = 0.0;
                end
                
                if(hard_s_min > s_max)
                    obj.st_(i).s = hard_s_min;
                else
                    hard_s_max = min(s_max,hard_s_max);
                    soft_s_max = min((obj.st_(i-1).s + (obj.st_(i - 1).v + a_max * dt) * dt),s_da_max );
                    soft_s_min = max((obj.st_(i-1).s + (obj.st_(i - 1).v + a_min * dt) * dt),s_da_min );
                    s_v_p = obj.st_(i - 1).s + v_max * dt;
%                     s_p = 0.0;
                    if(up)
                        s_p = min(soft_s_max,s_v_p);
                    else
                        s_p = min(soft_s_min,s_v_p);
                    end
                    obj.st_(i).s = Clamp(s_p,hard_s_min,hard_s_max);
                end
                
            else
                hard_s_max = min(min(s_max , (obj.st_(i - 1).s + v_max * dt) ),...
                                               min( (obj.st_(i-1).s + (obj.st_(i - 1).v + a_max * dt) * dt) , s_da_max));
                hard_s_min = max(max(obj.st_(i - 1).s , (obj.st_(i - 1).s + 0.0 * dt) ),...
                                               max( (obj.st_(i-1).s + (obj.st_(i - 1).v + a_min * dt) * dt) , s_da_min));
                    
                if(hard_s_max < hard_s_min)
                    out =  false;
                    return;
                end
                if(up)
                    obj.st_(i).s = hard_s_max;
                else
                    obj.st_(i).s = hard_s_min;
                end
                   
            end
            obj.st_(i).v = (obj.st_(i).s - obj.st_(i - 1).s) / dt;
            obj.st_(i).a = (obj.st_(i).v - obj.st_(i - 1).v) / dt;
            obj.st_(i).da = (obj.st_(i).a - obj.st_(i - 1).a) / dt;
            obj.st_(i).dda = (obj.st_(i).da - obj.st_(i - 1).da) / dt;
            obj.st_(i).up = up;
            obj.st_(i).adjust = adjust;

            out = true;
                       
        end
        function out = Clamp(input,input_min,input_max)
            if input < input_min
                out = input_min;
            elseif(input > input_max)
                out = input_max;
            else
                out = input;
            end
        end
        function [val]= Solve(obj)
            InitConstraintsTables(obj);
            
            InitStTable(obj);
            
            PreprocessSConstraints(obj);
            
            if(~OptimizeStTable(obj))
                val = false;
                return;
            end
            num = length(obj.st_);
            obj.output_ = MakeNaviSpeedTsPointsStructArray(obj,num);
            
            for i  = 1:1:length(obj.output_)
                obj.output_(i).t = obj.t_(i);
                obj.output_(i).s = obj.st_(i).s;
                obj.output_(i).v = obj.st_(i).v;
                obj.output_(i).a = obj.st_(i).a;
                obj.output_(i).da = obj.st_(i).da;
                obj.output_(i).dda = obj.st_(i).dda;
            end
            
        val = true;    
        end%end Solve function
        
        function UpdateConstraints(obj,constraints)
            for i = 1:1:length(obj.constraints_)
                    CombineConstraints(constraints , obj.constraints_(i));
            end
        end%end UpdateConstraints function 
        
        function CombineConstraints(obj,constraints,dst)
            dst.v_max = min(constraints.v_max, dst.v_max);
            dst.v_preffered = min (constraints.v_preffered , dst.v_preffered);
            dst.a_max = min(constraints.a_max , dst.a_max);
            dst.a_preffered = min(constraints.a_preffered , dst.a_preffered);
            dst.b_max = min(constraints.b_max , dst.b_max);
            dst.b_preffered = min(constraints.b_preffered , dst.b_preffered);
            dst.da_max = min(constraints.da_max , dst.dda_max);
            dst.da_preffered = min(constraints.b_preffered , dst.da_preffered);
            dst.dda_max = min (constraints.dda_max , dst.dda_max);
            dst.dda_preffered = min(constraints.dda_preffered , dst.dda_preffered);
        end% end CombineConstraints function
        
        function CombineConstraintsRewrite(obj,constraints,index)
%             fprintf('constraints.v_max: %f \n',constraints.v_max);
            obj.constraints_(index).v_max = constraints.v_max;
            obj.constraints_(index).v_preffered = constraints.v_preffered ;
            obj.constraints_(index).a_max = constraints.a_max;
            obj.constraints_(index).a_preffered = constraints.a_preffered ;
            obj.constraints_(index).b_max = constraints.b_max ;
            obj.constraints_(index).b_preffered = constraints.b_preffered;
            obj.constraints_(index).da_max = constraints.da_max ;
            obj.constraints_(index).da_preffered = constraints.b_preffered ;
            obj.constraints_(index).dda_max = constraints.dda_max ;
            obj.constraints_(index).dda_preffered = constraints.dda_preffered ;
%             fprintf('obj.constraints_(%d ).v_max: %f \n',index,obj.constraints_(index).v_max);
        end% end CombineConstraintsRewrite function
        
        function UpdateRangeConstraints(obj,start_s, end_s ,constraints)
            i0 = uint32(floor(start_s / obj.s_step_));
            i1 = uint32(ceil(end_s / obj.s_step_));
            
            
            if i0 == i1
%                 CombineConstraints(obj,constraints , obj.constraints_(1));
                CombineConstraintsRewrite(obj,constraints , 1);
            else
                for i = 1:1:length(obj.constraints_)
%                     CombineConstraints(obj,constraints , obj.constraints_(i));
                    CombineConstraintsRewrite(obj,constraints , i);
                end
            end    
            
        end % end UpdateRangeConstraints
        
        
    end
        
end