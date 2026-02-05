function transformGraphicsObject(T, s)
    % T: 4x4 SE(3) transformation matrix
    % s: Graphics object (e.g., trisurf)

    if size(T, 1) ~= 4 || size(T, 2) ~= 4
        error('Transformation matrix must be a 4x4 matrix');
    end

    if ~ishandle(s)
        error('Invalid graphics object');
    end

    % Extract rotation matrix and translation vector from SE(3) matrix
    rotationMatrix = T(1:3, 1:3);
    translationVector = T(1:3, 4);

    % Get the current vertices of the graphics object
    vertices = [s.XData(:), s.YData(:), s.ZData(:)];

    % Apply rotation
    rotatedVertices = (rotationMatrix * vertices')';

    % Apply translation
    translatedVertices = rotatedVertices + translationVector';

    % Update graphics object with new vertices
    s.XData = reshape(translatedVertices(:, 1), size(s.XData));
    s.YData = reshape(translatedVertices(:, 2), size(s.YData));
    s.ZData = reshape(translatedVertices(:, 3), size(s.ZData));
end
