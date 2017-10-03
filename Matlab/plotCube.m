function cube = plotCube(origin, lenSides, varargin)
% CUBE_PLOT plots a cube with a vertex at origin and dimensions lenSides

	% Define the vertexes of the unit cubic
	ver = [1 1 0;
		0 1 0;
		0 1 1;
		1 1 1;
		0 0 1;
		1 0 1;
		1 0 0;
		0 0 0];
	%  Define the faces of the unit cubic
	faces = [1 2 3 4;
		4 3 5 6;
		%6 7 8 5;
		1 2 8 7;
		6 7 1 4;
		2 3 5 8];
	vertices = ver.*lenSides + origin;
	cube = patch('Faces',faces, 'Vertices',vertices, varargin{:});
end