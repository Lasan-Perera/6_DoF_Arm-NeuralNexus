function armIK(robot, ik, weights, q0)
    clearVisual(robot.Base);
    for i = 1:6, clearVisual(robot.Bodies{i}); end

    Thome = getTransform(robot, q0, 'Body6');
    tipHome = Thome(1:3,4)';

    f = figure('Name','Arm IK Control','Position',[100 100 950 600]);
    ax = axes('Parent',f,'Position',[0.30 0.05 0.68 0.9]);
    show(robot, q0, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);

    S.robot=robot; S.ik=ik; S.weights=weights; S.Rhome=Thome(1:3,1:3); S.q=q0; S.ax=ax;
    lbl={'X','Y','Z'}; sld=gobjects(1,3); val=gobjects(1,3);
    for k=1:3
        uicontrol(f,'Style','text','Position',[20 500-70*k 20 22],'String',lbl{k});
        sld(k)=uicontrol(f,'Style','slider','Position',[45 500-70*k 180 22], ...
            'Min',tipHome(k)-0.25,'Max',tipHome(k)+0.25,'Value',tipHome(k));
        val(k)=uicontrol(f,'Style','text','Position',[45 478-70*k 180 20], ...
            'String',sprintf('%.3f m',tipHome(k)));
        sld(k).Callback=@(s,e) solveIK(f);
    end
    uicontrol(f,'Style','pushbutton','Position',[45 60 180 30], ...
        'String','▶ Play smooth move','Callback',@(s,e) playMove(f));
    S.qStart=q0;
    S.angleBox=uicontrol(f,'Style','text','Position',[20 110 210 130], ...
        'String','','FontName','Courier','FontSize',12, ...
        'HorizontalAlignment','left','BackgroundColor','w');
    S.sld=sld; S.val=val; f.UserData=S;
end

function solveIK(f)
    S=f.UserData;
    xyz=[S.sld(1).Value,S.sld(2).Value,S.sld(3).Value];
    for k=1:3, S.val(k).String=sprintf('%.3f m',xyz(k)); end
    T=trvec2tform(xyz); T(1:3,1:3)=S.Rhome;
    [q,info]=S.ik('Body6',T,S.weights,S.q);
    S.q=q; f.UserData=S; setappdata(0,'lastQ',q);
    showAngles(S.angleBox, q);
    show(S.robot,q,'Parent',S.ax,'PreservePlot',false,'FastUpdate',true);
    title(S.ax,info.Status);
end

function playMove(f)
    S=f.UserData;
    qA=S.qStart; qB=S.q; steps=500;
    for k=1:steps
        s=(k-1)/(steps-1);
        blend=10*s^3-15*s^4+6*s^5;
        q=qA+(qB-qA)*blend;
        show(S.robot,q,'Parent',S.ax,'PreservePlot',false,'FastUpdate',true);
        drawnow;
    end
    S.qStart=qB; f.UserData=S;
end

function showAngles(box, q)
    d = rad2deg(q);
    txt = "Joint angles (deg):";
    for i = 1:6
        txt = txt + sprintf("\n  J%d = %7.2f", i, d(i));
    end
    box.String = txt;
end