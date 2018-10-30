function RRTu()
    % The RRT-u variant
    % By Urban Eriksson
 
    figure(1003)
    clf 
    axes('box','off','xtick',[],'ytick',[],'ztick',[],'xcolor',[1 1 1],'ycolor',[1 1 1]);
    hold on
    axis([0 1 0 1])

    daspect([1 1 1])
    title('RRT-u')

    xstart = 0.2;
    ystart = 0.2;
    xgoal = 0.8;
    ygoal = 0.8;
    goalradius = 0.03;
    maxacc = 1.0;
    maxvel = 0.25;
    dt_limit = 1;  % Fixed incremental travel time


    nodesx = xstart;
    nodesy = ystart;
    velocitiesx = 0;
    velocitiesy = 0;
    baktrk = -1;
    costs = 0;
    %linehandles = plot(xstart,ystart);
    accelerationsx = 0;
    accelerationsy = 0;
    dts = 0;

    plot(xstart,ystart,'.','markersize',24,'color',[1 0 1]) % start marker
    plot(xgoal,ygoal,'.g','markersize',24,'color',[0 0.8 0]) % goal marker
    plot([0.4 0.6 0.6 0.4 0.4],[0.4 0.4 0.6 0.6 0.4],'linewidth',2,'color','k') % obstacle

    delay = 0.4;

    max_iter = 1000;
    for j = 1:max_iter

        % Sample random configuration
        xrand = rand;
        yrand = rand;

        %  Find nearest
        N = length(nodesx);
        cost_cand = zeros(1,N);
        dt_min = zeros(1,N);
        ax_cand = zeros(1,N);
        ay_cand = zeros(1,N);
        for k = 1:length(nodesx)
            [dt_min(k), ax_cand(k) , ay_cand(k)]= minimize_deltat(nodesx(k), nodesy(k), xrand, yrand, velocitiesx(k), velocitiesy(k), maxacc, maxvel);
            cost_cand(k) = costs(k) + dt_min(k);
        end

        %[dt_min,ix] = min(dt_min);
        [dt_min,ix_min] = min(cost_cand);

        [x_path1,y_path1,vx1,vy1] = calculate_path_points(nodesx(ix_min), nodesy(ix_min), velocitiesx(ix_min), velocitiesy(ix_min), dt_min, ax_cand(ix_min), ay_cand(ix_min));
        
        if collision_free(x_path1, y_path1)

            h1 = plot(x_path1,y_path1,'k:');
            h2 = plot(xrand,yrand,'.k','markersize',10);
            pause(delay)
            delete(h1)
            delete(h2)
            
            dt_new = min(dt_limit,dt_min);
            [x_path2,y_path2,vx2,vy2] = calculate_path_points(nodesx(ix_min), nodesy(ix_min), velocitiesx(ix_min), velocitiesy(ix_min), dt_new, ax_cand(ix_min), ay_cand(ix_min));
            xnew = x_path2(end);
            ynew = y_path2(end);
            
            plot(x_path2,y_path2,'b');
            plot(xnew,ynew,'.k','markersize',10)
            
            nodesx = [nodesx xnew];
            nodesy = [nodesy ynew];
            velocitiesx = [velocitiesx vx2];
            velocitiesy = [velocitiesy vy2];
            baktrk = [baktrk ix_min];
            %linehandles = [linehandles h3];
            costs = [costs cost_cand(ix_min)];
            accelerationsx = [accelerationsx ax_cand(ix_min)];
            accelerationsy = [accelerationsy ay_cand(ix_min)];
            dts = [dts dt_new];
            
            if (sqrt((xnew-xgoal)^2 + (ynew-ygoal)^2) < goalradius)
                optimal_path_plot(nodesx,nodesy,velocitiesx,velocitiesy,dts,accelerationsx,accelerationsy,baktrk);
                break
            end
            
            pause(delay)
        end

        drawnow
        delay = delay * 0.9;

    end
end


function is_free = collision_free(xpath, ypath)
    is_free = ~any( abs(xpath - 0.5) < 0.1 & abs(ypath - 0.5) < 0.1);
end

function optimal_path_plot(nodesx,nodesy,nodesvx,nodesvy,dts,axs,ays,baktrk)
    ix = length(nodesx);
    while (baktrk(ix) ~= -1)

        ixbak = baktrk(ix);

        [x_path, y_path, vx_end, vy_end] = calculate_path_points(nodesx(ixbak),nodesy(ixbak),nodesvx(ixbak),nodesvy(ixbak),dts(ix),axs(ix),ays(ix));


        plot(x_path,y_path,'color',[0 0 0],'linewidth',2)
        plot(x_path,y_path,'color',[0 0.9 0],'linewidth',1.5)
        ix = baktrk(ix);
    end
end

function [dt_opt, ax_opt , ay_opt]= minimize_deltat(x0,y0,x1,y1,vx0,vy0,maxa, maxv)

    dx = x1 - x0;
    dy = y1 - y0;

    dt_array = zeros(1,12);
    ax_array = zeros(1,12);
    ay_array = zeros(1,12);
    approved = false(1,12);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % #1 vx is the max allowed velocity at dt
    vx1 = maxv;
    dt = 2 * dx / (vx1 +  vx0);
    ax = (vx1 - vx0) / dt;
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vy1 = vy0 + ay * dt;
    [approved(1),dt_array(1),ax_array(1),ay_array(1)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #2 vx is the min allowed velocity at dt
    vx1 = -maxv;
    dt = 2 * dx / (vx1 +  vx0);
    ax = (vx1 - vx0) / dt;
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vy1 = vy0 + ay * dt;
    [approved(2),dt_array(2),ax_array(2),ay_array(2)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #3 ax is the max allowed acceleration : positive solution
    ax = maxa;
    t2 = (vx0 / ax)^2 + 2 * dx / ax;
    dt = - vx0 / ax + sqrt( t2 );
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vx1 = vx0 + ax * dt;
    vy1 = vy0 + ay * dt;
    [approved(3),dt_array(3),ax_array(3),ay_array(3)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #4 ax is the max allowed acceleration : negativetive solution
    ax = maxa;
    t2 = (vx0 / ax)^2 + 2 * dx / ax;
    dt = - vx0 / ax - sqrt( t2 );
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vx1 = vx0 + ax * dt;
    vy1 = vy0 + ay * dt;
    [approved(4),dt_array(4),ax_array(4),ay_array(4)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #5
    ax = -maxa;
    t2 = (vx0 / ax)^2 + 2 * dx / ax;
    dt = - vx0 / ax + sqrt( t2 );
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vx1 = vx0 + ax * dt;
    vy1 = vy0 + ay * dt;
    [approved(5),dt_array(5),ax_array(5),ay_array(5)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #6
    ax = -maxa;
    t2 = (vx0 / ax)^2 + 2 * dx / ax;
    dt = - vx0 / ax - sqrt( t2 );
    ay = 2 / dt^2 * (dy - vy0 * dt);
    vx1 = vx0 + ax * dt;
    vy1 = vy0 + ay * dt;
    [approved(6),dt_array(6),ax_array(6),ay_array(6)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % #7 vy = given velocity at dt
    vy1 = maxv;
    dt = 2 * dy / (vy1 +  vy0);
    ay = (vy1 - vy0) / dt;
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vx1 = vx0 + ax * dt;
    [approved(7),dt_array(7),ax_array(7),ay_array(7)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #8 vx = given velocity at dt
    vy1 = -maxv;
    dt = 2 * dy / (vy1 +  vy0);
    ay = (vy1 - vy0) / dt;
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vx1 = vx0 + ax * dt;
    [approved(8),dt_array(8),ax_array(8),ay_array(8)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #9
    ay = maxa;
    t2 = (vy0 / ay)^2 + 2 * dy / ay;
    dt = - vy0 / ay + sqrt( t2 );
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vy1 = vy0 + ay * dt;
    vx1 = vx0 + ax * dt;
    [approved(9),dt_array(9),ax_array(9),ay_array(9)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #10
    ay = maxa;
    t2 = (vy0 / ay)^2 + 2 * dy / ay;
    dt = - vy0 / ay - sqrt( t2 );
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vy1 = vy0 + ay * dt;
    vx1 = vx0 + ax * dt;
    [approved(10),dt_array(10),ax_array(10),ay_array(10)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #11
    ay = -maxa;
    t2 = (vy0 / ay)^2 + 2 * dy / ay;
    dt = - vy0 / ay + sqrt( t2 );
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vy1 = vy0 + ay * dt;
    vx1 = vx0 + ax * dt;
    [approved(11),dt_array(11),ax_array(11),ay_array(11)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    % #12
    ay = -maxa;
    t2 = (vy0 / ay)^2 + 2 * dy / ay;
    dt = - vy0 / ay - sqrt( t2 );
    ax = 2 / dt^2 * (dx - vx0 * dt);
    vy1 = vy0 + ay * dt;
    vx1 = vx0 + ax * dt;
    [approved(12),dt_array(12),ax_array(12),ay_array(12)] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa);

    [dt_opt,ind] = min(dt_array);
    ax_opt = ax_array(ind);
    ay_opt = ay_array(ind);
end

function [approved,dt,ax,ay] = test_and_approve(dt,vx1,vy1,ax,ay,maxv,maxa)
    if ~isnan(dt) && ~isinf(dt) && isreal(dt) && dt > 0 && abs(vx1) <= maxv && abs(vy1) <= maxv  && abs(ax) <= maxa && abs(ay) <= maxa 
        approved = true;
    else
        approved = false;
        dt = 1e10;
    end
end

function [x_path, y_path, vx_end, vy_end] = calculate_path_points(x,y,vx,vy,dt,ax,ay)
    dt_array = (0:100)/100 * dt;
    x_path = ax * dt_array.^2 / 2 + vx * dt_array + x;
    y_path = ay * dt_array.^2 / 2 + vy * dt_array + y;
    vx_end = vx + ax * dt;
    vy_end = vy + ay * dt;
end
    
    
    