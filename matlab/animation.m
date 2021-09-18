function yyy = animation(mapname, solution);

VoxelizedMap(mapname);
hold on;



figure(1);
set(gcf,'Position',[100 100 1000 1000]);
axis equal;
xlim([0 10]);
ylim([0 10]);
zlim([0 2]);
xlabel('x');
ylabel('y');

writerObj = VideoWriter('yz.avi');
v.Quality = 95;
writeObj.FrameRate = 30;
open(writerObj);

p_dvt = load(solution);
plane = stlread('~/LMP/map/plane.stl');

n_dvt = size(p_dvt,1);

t_dvt = 0:0.025:(n_dvt-1) * 0.025;

fontsize = 15;

index_dvt = 1;

thecolor = get(gca,'ColorOrder');
curve_dvt = animatedline('Color','r','LineWidth', 2);
hold on;
head_dvt = patch(plane, 'FaceColor', [0.35 0.35 0.35], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.16);
hold off;

x = 10 .* head_dvt.XData;
y = 10 .*head_dvt.YData;
z = 10 .*head_dvt.ZData;

pause(10.0);


view([90 0]);

for index_t = 1:length(t_dvt)
    
    t = t_dvt(index_t);
    %xlim([p_dvt(index_t, 1) - 1, p_dvt(index_t, 1) + 1.6]);
    xlim([0, p_dvt(index_t, 1) + 0.1]);
    ylim([p_dvt(index_t, 2) - 0.5, p_dvt(index_t, 2) + 1.5]);
    %ylim([p_dvt(index_t, 2) - 0.1, 10]);
    %zlim([p_dvt(index_t, 3) - 0.7, p_dvt(index_t, 3) + 0.7]);
    %view([round(p_dvt(index_t,4))*53 60]);

    addpoints(curve_dvt, p_dvt(index_t,1), p_dvt(index_t,2), p_dvt(index_t,3));
    %set(gca, 'CameraPosition', [p_dvt(index_t,1)*10  p_dvt(index_t,2)*10 p_dvt(index_t,3)*10]);
    %set(gca, 'CameraPosition', [20*p_dvt(index_t,1) 20*( p_dvt(index_t,2)) (p_dvt(index_t,3))]);

    head_dvt.XData = p_dvt(index_t, 1) + cos(p_dvt(index_t, 4)) * x - sin(p_dvt(index_t, 4)) * y;
    head_dvt.YData = p_dvt(index_t, 2) + sin(p_dvt(index_t, 4)) * x + cos(p_dvt(index_t, 4)) * y;
    head_dvt.ZData = p_dvt(index_t, 3) + z;

    %set(gca, 'CameraTarget', [p_dvt(index_t,1)  p_dvt(index_t,2) p_dvt(index_t,3)]);
    %campos( [p_dvt(index_t,1)  -p_dvt(index_t,2) p_dvt(index_t,3)] );

    %if(mod(index_t,1000) == 0)
        %w = waitforbuttonpress;
    %end

    frame = getframe(gcf);
    writeVideo(writerObj, frame);

    drawnow limitrate;

end
xlim([0 10])
ylim([0 10])
zlim([0 2])

frame = getframe(gcf);
writeVideo(writerObj, frame);

close(writerObj);

