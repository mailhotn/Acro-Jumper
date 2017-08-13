% %%%%%% % Renders Acro-Jumper % %%%%%% %
function status = Render(AJ, t, X, flag)
    % Get model positions
    P = AJ.GetPos(X,'P');
    Q = AJ.GetPos(X,'Q');
    R = AJ.GetPos(X,'R');

    if isempty(AJ.RenderObj)
        % Model hasn't been rendered yet
        
        % Render links
        AJ.RenderObj.nL1 = DrawLink(AJ, P(1), P(2), Q(1), Q(2), 0, []);
        AJ.RenderObj.nL2 = DrawLink(AJ, Q(1), Q(2), R(1), R(2), 0, []);

        axis equal
        axis([-1 1 0 1])
        
        % Finished rendering
        % Call function again to proceed with the code below
        Render(AJ, t, X, flag);
    else
            DrawLink(AJ, P(1), P(2), Q(1), Q(2), 0, AJ.RenderObj.nL1);
            DrawLink(AJ, Q(1), Q(2), R(1), R(2), 0, AJ.RenderObj.nL2);
    end
    status = 0;

    % %%%%%%%% Auxiliary nested functions %%%%%%%% %
    % %%%% Draw Circle %%%% %
    % Draws a circle of radius R in pos (x,y,z)
    function [ AJ ] = DrawCircle(AJ, x, y, z, R, color, ID) %#ok
        coordX=zeros(1,AJ.CircRes);
        coordY=zeros(1,AJ.CircRes);
        coordZ=zeros(1,AJ.CircRes);

        for r=1:AJ.CircRes
            coordX(1,r)=x+R*cos(r/AJ.CircRes*2*pi);
            coordY(1,r)=y+R*sin(r/AJ.CircRes*2*pi);
            coordZ(1,r)=z;
        end

        h=patch(coordX,coordY,coordZ,color);
        set(h,'EdgeColor',color.^4);
        set(h,'LineWidth',2*AJ.LineWidth);

        switch ID
            case 1
                AJ.RenderObj.Cm1=h;
            case 2
                AJ.RenderObj.Cm2=h;
            case 3
                AJ.RenderObj.Cmh=h;
            otherwise
                return;
        end                    
    end

    % %%%% Draw Link %%%% %
    % Draws a link of from (x0,y0) to (x1,y1)
    function [ res ] = DrawLink(AJ, x0, y0, x1, y1, z, Obj)
        if isempty(Obj)
            Length=sqrt((x1-x0)^2+(y1-y0)^2);
            Center=[(x0+x1)/2;
                    (y0+y1)/2];
            Orientation=atan2(y1-y0,x1-x0);

            res.Trans=hgtransform('Parent',gca);
            Txy=makehgtform('translate',[Center(1) Center(2) 0]);
            Rz=makehgtform('zrotate',Orientation-pi/2);

            coordX=zeros(1,2*AJ.LinkRes+1);
            coordY=zeros(1,2*AJ.LinkRes+1);
            coordZ=zeros(1,2*AJ.LinkRes+1);

            x=0;
            y=Length-AJ.link_width/2;
            for r=1:AJ.LinkRes
                coordX(1,r)=x+AJ.link_width/2*cos(r/AJ.LinkRes*pi);
                coordY(1,r)=y+AJ.link_width/2*sin(r/AJ.LinkRes*pi);
                coordZ(1,r)=0;
            end

            y=AJ.link_width/2;
            for r=AJ.LinkRes:2*AJ.LinkRes
                coordX(1,r+1)=x+AJ.link_width/2*cos(r/AJ.LinkRes*pi);
                coordY(1,r+1)=y+AJ.link_width/2*sin(r/AJ.LinkRes*pi);
                coordZ(1,r+1)=0;
            end

            res.Geom=patch(coordX,coordY,coordZ,AJ.link_color); 
            set(res.Geom,'EdgeColor',[0 0 0]);
            set(res.Geom,'LineWidth',2*AJ.LineWidth);

            set(res.Geom,'Parent',res.Trans);
            set(res.Trans,'Matrix',Txy*Rz);
        else
            Orientation=atan2(y1-y0,x1-x0);
            Length=sqrt((x1-x0)^2+(y1-y0)^2); %#ok

            Txy=makehgtform('translate',[x0 y0 z]);
            Rz=makehgtform('zrotate',Orientation-pi/2);
%             Sx=makehgtform('scale',[1,Length/(2*AJ.l),1]);
            set(Obj.Trans,'Matrix',Txy*Rz);
            res=1;
        end
    end

    % %%%% Draw Vector %%%% %
    % Draws a vector from x0 to x1
    function DrawVector(AJ,x0,x1,zIndex,Color) %#ok
        VecScale=AJ.link_width*0.75;
        Length=sqrt((x1(1)-x0(1))^2+(x1(2)-x0(2))^2);
        if Length<VecScale*4
            return;
        end

        Dir=(x1-x0)/Length;
        DirPerp=[-Dir(2); Dir(1)];

        Points=zeros(3,7);
        Points(1:2,1)=x0-DirPerp*VecScale/2;
        Points(1:2,2)=x0+DirPerp*VecScale/2;
        Points(1:2,3)=x1-Dir*VecScale*4+DirPerp*VecScale/2;
        Points(1:2,4)=x1-Dir*VecScale*4+DirPerp*VecScale*1.5;
        Points(1:2,5)=x1;
        Points(1:2,6)=x1-Dir*VecScale*4-DirPerp*VecScale*1.5;
        Points(1:2,7)=x1-Dir*VecScale*4-DirPerp*VecScale/2;
        Points(3,:)=zIndex*ones(1,7);

        patch(Points(1,:),Points(2,:),Points(3,:),Color);
    end
end