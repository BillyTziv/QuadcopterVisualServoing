function visualize(xpos, ypos, zpos)
    [X,Y,Z] = sphere(80);

    mesh(X+xpos,Y+ypos,Z+zpos);
    axis([-15 15 -15 15 -15 15]);
end