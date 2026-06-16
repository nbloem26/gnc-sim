function v = getcol(tbl, name)
%GETCOL  Fetch a named column from a run telemetry frame, portably.
%
%   V = GETCOL(TBL, NAME) returns the column NAME as a column vector. TBL is
%   what LOAD_RUN stores for a CSV: a MATLAB `table`, or (on Octave, which has
%   no `table`) a scalar struct of column vectors. This helper hides that
%   difference so the plotting scripts read the same on both platforms.
%
%   Errors if the column is absent. See also LOAD_RUN.

    if istable(tbl)
        if ~ismember(name, tbl.Properties.VariableNames)
            error('getcol:noColumn', 'column "%s" not found', name);
        end
        v = tbl.(name);
    elseif isstruct(tbl)
        if ~isfield(tbl, name)
            error('getcol:noColumn', 'column "%s" not found', name);
        end
        v = tbl.(name);
    else
        error('getcol:badInput', 'expected a table or struct of columns');
    end
    v = v(:);
end
