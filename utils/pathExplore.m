function direction=pathExplore(knownPoints,beenThere)
    
    %knownPoints=[knownPoints knownPoints(:,1)];
    f=[0 0 0]';
    delta=0.01;
    %bound=boundary(knownPoints',1);
    q=2000;
    d0=1;
    for i=1:size(knownPoints,2);
        if ~(isnan(knownPoints(1,i)))
            f(1)=f(1)+potencial([0;0],knownPoints(:,i),q,d0);
            %i1=bound(ii);
            %i2=bound(ii+1);
            %f(1)=f(1)+potencial2([0;0],knownPoints(:,i1),knownPoints(:,i2));


            if isinf(f(1))
                f(1)=10^5;
            end

            f(2)=f(2)+potencial([delta;0],knownPoints(:,i),q,d0);
            f(3)=f(3)+potencial([0;delta],knownPoints(:,i),q,d0);
            %f(2)=f(2)+potencial2([delta;0],knownPoints(:,i1),knownPoints(:,i2));
            %f(3)=f(3)+potencial2([0;delta],knownPoints(:,i1),knownPoints(:,i2));
        end
    end
    q=500;
    d0=5;
    for i=1:size(beenThere,2);
        if ~(isnan(beenThere(1,i)))
            f(1)=f(1)+potencial([0;0],beenThere(:,i),q,d0);

            if isinf(f(1))
                f(1)=10^5;
            end

            f(2)=f(2)+potencial([delta;0],beenThere(:,i),q,d0);
            f(3)=f(3)+potencial([0;delta],beenThere(:,i),q,d0);
        end
    end
    
    
    %f;
    
    gradF=-([f(2);f(3)]-[f(1);f(1)])/delta;
    draw=0;
    if draw
        n=50;
        x = -n:2:n;
        y = x;
        [X,Y] = meshgrid(x,y);
        Z=zeros(size(X));
        for i=1:size(X,1)
            for j=1:size(X,2);
                for ii=1:numel(bound)-1
                    k1=bound(ii);
                    k2=bound(ii+1);
                    l=distance(knownPoints(:,k1)',knownPoints(:,k2)');
                    Z(i,j)=Z(i,j)+potencial2([X(i,j);Y(i,j)],knownPoints(:,k1),knownPoints(:,k2));
                    %q*l/(p2l([X(i,j);Y(i,j)],knownPoints(:,k1),knownPoints(:,k2)));
                    %Z(i,j)=Z(i,j)+1/distance([X(i,j) Y(i,j)],knownPoints(:,k)')^2;
                    if Z(i,j)>10
                        Z(i,j)=10;
                    end
                end
            end
        end

        figure(2)
        %contour(X,Y,Z)
        surf(X,Y,Z,'EdgeColor','none');
        %hold on
        drawnow
    end
    
    
    
    direction=atan2(gradF(2),gradF(1));
    

end

function U=potencial2(pt1,pt2,pt3)

    q=1000;
    l=distance(pt2',pt3');
    %U=q*l/(p2l([0;0],knownPoints(:,i1),knownPoints(:,i2)));
    if l<10
        U=q*l/distance(pt1',(pt2'+pt3')/2)^2;
    else
        U=q*l/distance(pt1',pt3')^2;
    end
    %(p2l([0;0],knownPoints(:,i1),knownPoints(:,i2)));
end

function U=potencial(pt1,pt2,q,d0)

    %q=1000;
    %d0=5;
    U=q/(distance(pt1',pt2')+d0)^2;
    
    %(p2l([0;0],knownPoints(:,i1),knownPoints(:,i2)));
end