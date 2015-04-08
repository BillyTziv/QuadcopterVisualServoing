% This code reads a column of signal values and outputs a graph with the signal

fileID=fopen('signal.txt');

formatSpec = '%f';
sizeA = [1 Inf];

A = fscanf(fileID, formatSpec, sizeA);
fclose(fileID);

A = A';
A
figure;
plot(A);
