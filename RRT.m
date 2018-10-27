function RRT()
    % Rapidly exploring random tree (RRT) animation
    % By Urban Eriksson

    figure(1001)
    clf
    axes('box','off','xtick',[],'ytick',[],'ztick',[],'xcolor',[1 1 1],'ycolor',[1 1 1]);
    hold on
    axis([0 1 0 1])
    daspect([1 1 1])
    title('RRT')

    xstart = 0.2;
    ystart = 0.2;
    xgoal = 0.8;
    ygoal = 0.8;
    goalradius = 0.03;
    eta = 0.12; % Fixed incremental distance

    nodesx = xstart; % x for all nodes
    nodesy = ystart; % y for all nodes
    baktrk = -1; % back pointers

    plot(xstart,ystart,'.','markersize',24,'color',[1 0 1]) % start marker
    plot(xgoal,ygoal,'.g','markersize',24,'color',[0 0.8 0]) % goal marker
    plot([0.4 0.6 0.6 0.4 0.4],[0.4 0.4 0.6 0.6 0.4],'linewidth',2,'color','k') % obstacle

    delay = 0.4;
    
    max_iter = 1000;
    for j = 1:max_iter

        % Sample random configuration
        xrand = rand;
        yrand = rand;

        % Find nearest
        [d_nearest, i_nearest] = min(sqrt((xrand - nodesx).^2 + (yrand - nodesy).^2));
        if collision_free(nodesx(i_nearest), xrand, nodesy(i_nearest), yrand)
            d_new = min(eta, d_nearest); % distance can either be eta or the distance to the nearest node
            k = d_new / d_nearest; % fraction (0-1)
            xnew = (1-k)*nodesx(i_nearest) + k*xrand;
            ynew = (1-k)*nodesy(i_nearest) + k*yrand;
            h1 = plot([nodesx(i_nearest) xrand], [nodesy(i_nearest) yrand], 'k:');
            h2 = plot(xrand, yrand, '.k', 'markersize', 10);
            pause(delay)
            delete(h1)
            delete(h2)
            plot([nodesx(i_nearest) xnew],[nodesy(i_nearest) ynew],'b');
            plot(xnew,ynew,'.k','markersize',10)

            % Add new node to list
            nodesx = [nodesx xnew];
            nodesy = [nodesy ynew];
            baktrk = [baktrk i_nearest];

            % Check if goal condition has been met for 
            if (sqrt((xnew-xgoal)^2 + (ynew-ygoal)^2) < goalradius)
                optimal_path_plot(nodesx,nodesy,baktrk);
                break
            end
        else
            h1 = plot([nodesx(i_nearest) xrand], [nodesy(i_nearest) yrand], 'r:');
            h2 = plot(xrand, yrand, '.k', 'markersize', 10);
            pause(delay)
            delete(h1)
            delete(h2)
        end

        pause(delay)
        drawnow
        delay = delay * 0.9;

    end
end

function is_free = collision_free(x1, x2, y1, y2)
    is_free = true;
    for j = 1:100
        k = j/100;
        xj = (1-k)*x1 + k*x2;
        yj = (1-k)*y1 + k*y2;

        % Check if inside the square
        if abs(xj-0.5) < 0.1 && abs(yj-0.5) < 0.1
            is_free = false;
            break;
        end
    end
end

function optimal_path_plot(nodesx, nodesy, baktrk)
    ix = length(nodesx);
    while (baktrk(ix) ~= -1)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0 0],'linewidth',2)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0.9 0],'linewidth',1.5)
        ix = baktrk(ix);
    end
end

    