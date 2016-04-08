classdef FunctionHandleConstraint < Constraint
  %FUNCTIONHANDLECONSTRAINT
  % A Constraint implementation where the constraint is given
  % by a function handle.

  properties(SetAccess = protected)
    eval_handle
  end

  methods
    function obj = FunctionHandleConstraint(lb,ub,xdim,eval_handle,grad_level)
      % @param lb    -- The lower bound of the constraint
      % @param ub    -- The upper bound of the constraint
      % @param xdim  -- An int scalar. x is double vector of xdim x 1
      % @param eval_handle -- A function handle which performs the
      % constraint evaluation
      % @param grad_level optional user_gradient level.  see Constraint constructor. @default -1

      if nargin<5, grad_level = -1; end

      obj = obj@Constraint(lb,ub,xdim,grad_level);
      obj.eval_handle = eval_handle;
    end

    function obj_new = setEvalHandle(obj,eval_handle_new)
      % obj_new = setEvalHandle(obj,eval_handle_new) returns a
      % FunctionHandleConstraint object the same as this one, but with a new
      % eval_handle.
      %
      % Note: We're constructing a new constraint object here because altering
      % the eval_handle property of an existing constraint object makes
      % concatenating that object in a cell array *very* slow. We don't know why
      % that is.

      obj_new = FunctionHandleConstraint(obj.lb,obj.ub,obj.xdim,eval_handle_new);
      obj_new = obj_new.setSparseStructure(obj.iCfun,obj.jCvar);
      obj_new = obj_new.setName(obj.name);
    end

    function whatIsDifferent(obj, fhc)
      fprintf('Analyzing difference between this FunctionHandleConstraint and another FunctionHandleConstraint:\n')

      fprintf('Comparing lb (%f vs. %f)...', obj.lb, fhc.lb)
      if (isequal(obj.lb, fhc.lb))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end

      fprintf('Comparing ub (%f vs. %f)...', obj.ub, fhc.ub)
      if (isequal(obj.ub, fhc.ub))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end

      fprintf('Comparing xdim (%d vs. %d)...', obj.xdim, fhc.xdim)
      if (isequal(obj.xdim, fhc.xdim))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end

      fprintf('Comparing num_cnstr (%d vs. %d)...', obj.num_cnstr, fhc.num_cnstr)
      if (isequal(obj.num_cnstr, fhc.num_cnstr))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end

      fprintf('Comparing name...')
      if (isequal(obj.name, fhc.name))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end

      fprintf('Comparing ceq_idx (%d vs. %d)...', obj.ceq_idx, fhc.ceq_idx)
      if ~exist(obj.ceq_idx) && ~exist(fhc.ceq_idx)
        fprintf('Neither exist...Match!\n')
      else
        if (isequal(obj.ceq_idx, fhc.ceq_idx))
          fprintf('Match!\n')
        else
          fprintf('No Match!\n')
        end
      end
      fprintf('Comparing cin_idx (%d vs. %d)...', obj.cin_idx, fhc.cin_idx)
      if (isequal(obj.cin_idx, fhc.cin_idx))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing iCfun...')
      if (isequal(obj.iCfun, fhc.iCfun))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing jCvar...')
      if (isequal(obj.jCvar, fhc.jCvar))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing nnz...')
      if (isequal(obj.nnz, fhc.nnz))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing grad_level...')
      if (isequal(obj.grad_level, fhc.grad_level))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing grad_method...')
      if (isequal(obj.grad_method, fhc.grad_method))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
      end
      fprintf('Comparing eval_handle...')
      if (isequal(func2str(obj.eval_handle), func2str(fhc.eval_handle)))
        fprintf('Match!\n')
      else
        fprintf('No Match!\n')
        fprintf('obj.eval_handle:\n')
        obj.eval_handle
        fprintf('fhc.eval_handle:\n')
        fhc.eval_handle
      end
    end
  end

  methods (Access = protected)
    function varargout = constraintEval(obj,varargin)
      varargout = cell(1,nargout);
      [varargout{:}] = obj.eval_handle(varargin{:});
    end
  end
end
