function RRT()
    % Rapidly exploring random tree (RRT) animation

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

    nodesx = xstart; % x of all nodes
    nodesy = ystart; % y of all nodes
    baktrk = -1; % backpointers

    plot(xstart,ystart,'.','markersize',24,'color',[1 0 1]) % start marker
    plot(xgoal,ygoal,'.g','markersize',24,'color',[0 0.8 0]) % goal marker
    plot([0.4 0.6 0.6 0.4 0.4],[0.4 0.4 0.6 0.6 0.4],'linewidth',2,'color','k') % obstacle

    goalr = 0.05; % goal radius
    eta = 0.12; % Fixed incremental distance

    delay = 0.4;

    for j = 2:1000

        % Random configuration
        xrand = rand;
        yrand = rand;

        % Find nearest
        [d_near, i_near] = min(sqrt((xrand - nodesx).^2 + (yrand - nodesy).^2));
        if collision_free(nodesx(i_near), nodesy(i_near), xrand, yrand)
            d_new = min(eta, d_near); % distance can either be eta or the distance to the nearest node
            k = d_new / d_near; % fraction (0-1)
            xnew = (1-k)*nodesx(i_near) + k*xrand;
            ynew = (1-k)*nodesy(i_near) + k*yrand;
            h1 = plot([nodesx(i_near) xrand], [nodesy(i_near) yrand], 'k:');
            h2 = plot(xrand, yrand, '.k', 'markersize', 10);
            pause(delay)
            delete(h1)
            delete(h2)

            plot([nodesx(i_near) xnew],[nodesy(i_near) ynew],'b');
            plot(xnew,ynew,'.k','markersize',10)

            % Add new node to list
            nodesx = [nodesx xnew];
            nodesy = [nodesy ynew];
            baktrk = [baktrk i_near];

            % Check if goal condition has been met for 
            if (sqrt((xnew-xgoal)^2 + (ynew-ygoal)^2) < goalr)
                optimal_path_plot(nodesx,nodesy,baktrk);
                break
            end
        else
            h1 = plot([nodesx(i_near) xrand], [nodesy(i_near) yrand], 'r:');
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

function optimal_path_plot(nodesx,nodesy,baktrk)
    ix = length(nodesx);
    while (baktrk(ix) ~= -1)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0 0],'linewidth',2)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0.9 0],'linewidth',1.5)
        ix = baktrk(ix);
    end
end

function is_free = collision_free(x1,y1,x2,y2)
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


    