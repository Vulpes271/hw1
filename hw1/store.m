function store(name, data)
%Store data to file
%
%Inputs
% name: Name of the MAT file (without extension)
% data: Data to store
save(sprintf('%s.mat', name), 'data', '-v7');