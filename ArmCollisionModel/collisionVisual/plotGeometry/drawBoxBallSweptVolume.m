function drawBoxBallSweptVolume(T, len, width, height, radius, offset)
    % 参数化球心在空间内扫过的轨迹
    t = linspace(0, 1, 5);

    [X, Y, Z] = meshgrid(t * len - len/2, t * width - width/2, t * height - height/2);
    centerX = X(:) + offset(1);
    centerY = Y(:) + offset(2);
    centerZ = Z(:) + offset(3);
    % 计算球心轨迹
%     centerX = t * len - len/2;
%     centerY = t * width - width/2;
%     centerZ = t * height - height/2;

    % 绘制球心轨迹

    % 计算 Ball Swept Volume
    [X, Y, Z] = sphere(10);
    X = X * radius;
    Y = Y * radius;
    Z = Z * radius;
    
    xTranslatedList = [];
    yTranslatedList = [];
    zTranslatedList = [];
    for i = 1:length(centerX)
        xTranslated = X + centerX(i);
        yTranslated = Y + centerY(i);
        zTranslated = Z + centerZ(i);
        xTranslatedList = [xTranslatedList; xTranslated];
        yTranslatedList = [yTranslatedList; yTranslated];
        zTranslatedList = [zTranslatedList; zTranslated];
        % 绘制球体在当前位置的截面
    end
    
    K = convhull(xTranslatedList, yTranslatedList, zTranslatedList);
    s = trisurf(K, xTranslatedList, yTranslatedList, zTranslatedList, 'FaceColor', 'cyan', 'EdgeColor', 'k', 'FaceAlpha', 0.3);
%     surf(xTranslatedList, yTranslatedList, zTranslatedList, 'FaceColor', 'none', 'EdgeColor', 'k', 'FaceAlpha', 0.2);
%     drawBox(len, width, height);


    quiver3(T(1,4),T(2,4),T(3,4),...
            T(1,3)/1,T(2,3)/1,T(3,3)/1,...
            'LineWidth',2,'Color','b');
    quiver3(T(1,4),T(2,4),T(3,4),...
            T(1,1)/1,T(2,1)/1,T(3,1)/1,...
            'LineWidth',2,'Color','r');
    quiver3(T(1,4),T(2,4),T(3,4),...
            T(1,2)/1,T(2,2)/1,T(3,2)/1,...
            'LineWidth',2,'Color','g');

    transformGraphicsObject(T, s);
    
end

