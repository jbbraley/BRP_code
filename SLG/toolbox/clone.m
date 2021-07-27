function new = clone(obj,name)
%% clone() -> create new instance of object and populate public properties
%
% * auto-fills public properties
% * supply a parent object handle to spawn object other than obj
%   * useful when creating inherited properties
%
% author: john devitis; john braley
% create date: 21-Nov-2016 11:01:41
    if nargin > 1
        new = feval(class(name));
    else
        new = feval(class(obj));
    end
    p = properties(obj);
    info = metaclass(obj);
    for ii = 1:length(p)
        if ~info.PropertyList(ii).Dependent
            new.(p{ii}) = obj.(p{ii});
        end
    end
end
