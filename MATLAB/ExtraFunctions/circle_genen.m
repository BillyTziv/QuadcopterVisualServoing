if (nargin <3),
 error('Please see help for INPUT DATA.');
elseif (nargin==3)
    style='b-';
end;
THETA=linspace(0,2*pi,100);
RHO=ones(1,100)*radius;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
Z = center(3)*ones(1,length(X));
H=plot3(X,Y,Z,style);
axis square;