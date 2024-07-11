classdef Wolf
    %Data structure to store score and position of the leader wolfs
    
    properties
        Score {mustBeNumeric}
        Position
    end
    
    methods
        function obj = Wolf(score, position)
            obj.Score = score;
            obj.Position = position;
        end
    end
end

