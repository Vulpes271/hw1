function summary = parkingResults(data, limited)
% Pregled dosezene natancnosti, poti in omejitev za porocilo.
map = Map();
colors = lines(numel(data));
figure('Position',[80,80,1150,560]);
tiledlayout(2,2,'TileSpacing','compact');
ax = nexttile(1,[2,1]);
hold(ax,'on');
for obstacle = map.obstacles
    p = obstacle{1};
    fill(p(1,:),p(2,:),[.8,.8,.8],'EdgeColor','none');
end
for j = 1:11
    p = map.parkingAreas{j};
    plot(p(1,:),p(2,:),':','Color',[.65,.65,.65]);
    text(map.parkingPoses(1,j),map.parkingPoses(2,j), ...
        sprintf('P%d',j),'FontSize',8);
end
axis equal
axis([0,map.width,0,map.height]);
xlabel('x [m]'); ylabel('y [m]'); title('Poti in koncne lege');
grid on
wheels = nexttile(2);
hold(wheels,'on');
xlabel('skupni cas [s]'); ylabel('v_L, v_R [m/s]');
title('Hitrosti pogonskih koles (polna: L, crtkana: R)');
yline(.3,':'); yline(-.3,':'); grid on
steering = nexttile(4);
hold(steering,'on');
xlabel('skupni cas [s]'); ylabel('\gamma [deg]');
title('Zasuk prednjega dela'); grid on
if limited, yline(30,':'); yline(-30,':'); end
rows = zeros(numel(data),9);
offset = 0;
for j = 1:numel(data)
    d = data(j);
    plot(ax,d.x,d.y,'Color',colors(j,:),'LineWidth',1.1);
    p = map.getOutline(d.qEnd);
    patch(ax,p(1,:),p(2,:),colors(j,:),'FaceAlpha',.25);
    plot(wheels,offset+d.t,d.vl,'Color',colors(j,:));
    plot(wheels,offset+d.t,d.vr,'--','Color',colors(j,:));
    plot(steering,offset+d.t,rad2deg(d.gamma),'Color',colors(j,:));
    offset = offset+d.tEnd;
    length = sum(hypot(diff(d.x),diff(d.y)));
    ed = 1000*norm(d.goalPose(1:2)-d.qEnd(1:2));
    ephi = rad2deg(abs(wrapToPi(d.goalPose(3)-d.qEnd(3))));
    speed = max(abs([d.vl,d.vr]));
    acceleration = max(abs(diff([0,d.vl;0,d.vr],1,2)),[],'all')/.01;
    rows(j,:) = [j,d.goalId,d.tEnd,length,ed,ephi, ...
        speed,acceleration,nnz(d.c)];
end
summary = array2table(rows,'VariableNames', ...
    {'robot','cilj','cas_s','pot_m','ed_mm','ephi_deg', ...
    'vmax_m_s','amax_m_s2','trki'});
set(findall(gcf,'Type','Axes'),'Toolbar',[]);
end
