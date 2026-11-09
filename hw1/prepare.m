function prepare
%Creates a homework.zip from all the files in the current directory
%
% Before running this function verify that all files are saved and that
% MLX files are saved in the evaluated state (Run and Save). 
%
% Folders, ZIP and ASV files, and files that start with a tilde or dot
% are excluded.

files = dir();

out = {};
for i = 1:length(files)
    file = files(i);
    if ~file.isdir() && file.name(1) ~= '.' && file.name(1) ~= '~' && (length(file.name) < 4 || (~strcmp(file.name(end-3:end), '.asv') && ~strcmp(file.name(end-3:end), '.zip')))
        out(end+1) = {file.name};
    end
end

zip('homework.zip', out);
