function RRTstar()
    % Rapidly exploring random tree (RRT) animation
    % By Urban Eriksson
    
    run_with_pause = false;

    figure(1002)
    clf
    axes('box','off','xtick',[],'ytick',[],'ztick',[],'xcolor',[1 1 1],'ycolor',[1 1 1]);
    hold on
    axis([0 1 0 1])

    daspect([1 1 1])
    title('RRT*');

    xstart = 0.2;
    ystart = 0.2;
    xgoal = 0.8;
    ygoal = 0.8;
    goalradius = 0.03;
    eta=0.12;
    r_neighborhood = 0.15;

    nodesx = xstart; % x for all nodes
    nodesy = ystart; % y for all nodes
    baktrk = -1; % back pointers
    costs = 0;
    linehandles = plot(xstart,ystart);

    plot(xstart,ystart,'.','markersize',24,'color',[1 0 1]) % start marker
    plot(xgoal,ygoal,'.g','markersize',24,'color',[0 0.8 0]) % goal marker
    plot([0.4 0.6 0.6 0.4 0.4],[0.4 0.4 0.6 0.6 0.4],'linewidth',2,'color','k') % obstacle

    delay = 0.4;

    max_iter = 1000;
    i_current = 1;
    for j = 1:max_iter

        % Sample random configuration
        xrand = rand;
        yrand = rand;

        %  Find nearest
        [d_nearest, i_nearest] = min ( sqrt((xrand-nodesx).^2 + (yrand-nodesy).^2) );
        if collision_free(nodesx(i_nearest), xrand,nodesy(i_nearest), yrand)
            i_current = i_current + 1;
            d_new = min(eta, d_nearest);
            k = d_new / d_nearest;
            xnew = (1-k)*nodesx(i_nearest) + k*xrand;
            ynew = (1-k)*nodesy(i_nearest) + k*yrand;
            h1 = plot([nodesx(i_nearest) xrand],[nodesy(i_nearest) yrand],'k:');
            h2 = plot(xrand,yrand,'.k','markersize',10);
            pause(delay)
            delete(h1)
            delete(h2)
            prel_handle = plot([nodesx(i_nearest) xnew],[nodesy(i_nearest) ynew],'color',[0 0.8 0]);
            plot(xnew,ynew,'.k','markersize',10)

            % Rewire step
            hcircle = viscircles(gca,[xnew ynew],r_neighborhood,'color',[0.7 0.7 0.7],'linestyle','--');

            distance_to_new = sqrt( (nodesx - xnew).^2 + (nodesy - ynew).^2 );
            neighbors = find(distance_to_new <= r_neighborhood);

            % Sort costa from parents including the nearest parent
            [sorted_costs,i_sort] = sort(costs(neighbors) + distance_to_new(neighbors));

            found_collision_free_parent = false;
            i_loop = 1;
            % Nearest parent will always be collision free
            while ~found_collision_free_parent
                i_parent = i_sort(i_loop);
                if collision_free(nodesx(neighbors(i_parent)), xnew, nodesy(neighbors(i_parent)), ynew)
                    new_cost = sorted_costs(i_loop);
                    found_collision_free_parent = true;
                else
                    i_loop = i_loop + 1;
                end
            end

            if run_with_pause && neighbors(i_parent) ~= i_nearest
                pause
                delete(prel_handle)
                pause
                h_new = plot([nodesx(neighbors(i_parent)) xnew],[nodesy(neighbors(i_parent)) ynew],'b');
                pause
            else
                delete(prel_handle)
                pause(delay/2);
                h_new = plot([nodesx(neighbors(i_parent)) xnew],[nodesy(neighbors(i_parent)) ynew],'b');
            end

            nodesx = [nodesx xnew];
            nodesy = [nodesy ynew];
            baktrk = [baktrk neighbors(i_parent)];
            costs = [costs new_cost];
            linehandles = [linehandles h_new];

            neighbors_to_rewire = find(new_cost + distance_to_new(neighbors) < costs(neighbors));
            for ir = 1:length(neighbors_to_rewire)
                i_node = neighbors(neighbors_to_rewire(ir));
                if collision_free(nodesx(i_node), xnew, nodesy(i_node), ynew)
                    baktrk(i_node) = i_current;
                    costs(i_node) = new_cost + distance_to_new(i_node);
                    prel_handle =  plot([nodesx(i_node) xnew],[nodesy(i_node) ynew],'m');
                    if run_with_pause
                        pause
                        delete(linehandles(i_node));
                        pause
                        delete(prel_handle)
                        linehandles(i_node) = plot([nodesx(i_node) xnew],[nodesy(i_node) ynew],'b');
                        pause
                    else
                        delete(linehandles(i_node));
                        pause(delay)
                        delete(prel_handle)
                        linehandles(i_node) = plot([nodesx(i_node) xnew],[nodesy(i_node) ynew],'b');
                        pause(delay)
                    end
                end
            end

            delete(hcircle);

            if (sqrt((xnew-xgoal)^2 + (ynew-ygoal)^2) < goalradius)
                optimal_path_plot(nodesx, nodesy, baktrk);
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

function is_free = collision_free(x1,x2,y1,y2)
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


function optimal_path_plot(nodesx,nodesy,baktrk)
    ix = length(nodesx);
    while (baktrk(ix) ~= -1)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0 0],'linewidth',2)
        plot([nodesx(baktrk(ix)) nodesx(ix)],[nodesy(baktrk(ix)) nodesy(ix)],'color',[0 0.9 0],'linewidth',1.5)
        ix = baktrk(ix);
    end
end

        
                