% This function prints the initial and desired position of the projection
% points in the inertial coordinate system
function printInitialInertialPoints(s, s_des)
	figure;
	plot([s_des(1) s_des(3)], [s_des(2) s_des(4)], 'Color', 'b', 'LineWidth',2);
	hold on;
	plot([s_des(3) s_des(5)], [s_des(4) s_des(6)], 'Color', 'b', 'LineWidth',2);
	hold on;
	plot([s_des(5) s_des(7)], [s_des(6) s_des(8)], 'Color', 'b', 'LineWidth',2);
	hold on;
	plot([s_des(7) s_des(1)], [s_des(8) s_des(2)], 'Color', 'b', 'LineWidth',2);
	hold on;
	
	plot([s(1) s(3)],  [s(2) s(4)],  'Color',  'r',  'LineWidth', 2);
	hold on;
	plot([s(3 ) s(5 )],  [s(4 ) s(6 )],  'Color',  'r',  'LineWidth', 2);
	hold on;
	plot([s(5 ) s(7 )],  [s(6 ) s(8 )],  'Color',  'r',  'LineWidth', 2);
	hold on;
	plot([s(7 ) s(1 )], [s(8 ) s(2 )],  'Color',  'r',  'LineWidth', 2);
% 	hold on;
% 	
% 	plot([s_des_initial(1) s_des_initial(3)], [s_des_initial(2) s_des_initial(4)], 'Color', 'g', 'LineWidth',1);
% 	hold on;
% 	plot([s_des_initial(3) s_des_initial(5)], [s_des_initial(4) s_des_initial(6)], 'Color', 'g', 'LineWidth',1);
% 	hold on;
% 	plot([s_des_initial(5) s_des_initial(7)], [s_des_initial(6) s_des_initial(8)], 'Color', 'g', 'LineWidth',1);
% 	hold on;
% 	plot([s_des_initial(7) s_des_initial(1)], [s_des_initial(8) s_des_initial(2)], 'Color', 'g', 'LineWidth',1);
	hold off;
	axis equal;
	grid;
end