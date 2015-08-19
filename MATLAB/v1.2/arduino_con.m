clear all
clc
 
arduino=serial('COM3','BaudRate',9600);
 
fopen(arduino);
   
for i=1:500
    x(i)=i;
	y(i)=fscanf(arduino,'%d')
end
	
fclose(arduino);
disp('Making plot..')
plot(x,y);
