%% Diagnostic: compare getUIInfoMationInterface vs forwardKinematics2 coordinate frames
% This script prints raw values from both SO interfaces to determine 
% the coordinate frame relationship.

fprintf('\n=== SO Library Coordinate Frame Diagnostic ===\n\n');

%% Load SO library
soPath = '/home/ara/文档/X86_test/lib/libHRCInterface.so';
stubPath = fullfile(pwd, 's50_tcp_stubs.so');
headerPath = fullfile(pwd, 's50_collision_matlab.h');

% RTLD_GLOBAL preload for stubs
if exist(fullfile(pwd,'dlopen_global.mexa64'),'file')
    dlopen_global(stubPath);
end

if ~libisloaded('libHRCInterface')
    loadlibrary(soPath, headerPath, 'alias', 'libHRCInterface');
end

%% Initialize
DH = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5];
baseGeo = [0,0,20, 0,0,330, 160];
lowerGeo = [0,0,340, 900,0,340, 140];
elbowGeo = [-10,0,60, 941.5,0,60, 120];
upperGeo = [0,0,-50, 0,0,100, 100];
wristGeo = [0,0,20, 140];

homeJ = [0, -90, 0, 0, 90, 0];
% v1.0.0 必须先调用 initilizeRobotType + setKinParams
calllib('libHRCInterface','initilizeRobotType', int32(1));
kinParams = [DH, 0, 0];  % 10-element: DH[8] + {0, 0}
calllib('libHRCInterface','setKinParams', kinParams);
calllib('libHRCInterface','initACAreaConstrainPackageInterface',...
    int32(1), DH, baseGeo, lowerGeo, elbowGeo, upperGeo, wristGeo, homeJ);

%% Test 2 configurations
configs = {
    [0, -90, 0, 0, 90, 0], 'HOME [0,-90,0,0,90,0]';
    [0,   0, 0, 0,  0, 0], 'ZERO [0,0,0,0,0,0]';
};

for ci = 1:size(configs,1)
    q_deg = configs{ci,1};
    label = configs{ci,2};
    
    fprintf('--- Config: %s ---\n', label);
    
    % Update
    calllib('libHRCInterface','updateACAreaConstrainPackageInterface', q_deg, zeros(1,6), zeros(1,6));
    
    % Get collision geometry
    collIdx = int32(zeros(1,7)); collType = int32(zeros(1,7));
    dataList = zeros(1,63); radiusList = zeros(1,7);
    [collIdx, collType, dataList, radiusList] = calllib('libHRCInterface',...
        'getUIInfoMationInterface', collIdx, collType, dataList, radiusList);
    dataM = reshape(dataList, 9, 7)';
    
    fprintf('  getUIInfoMation raw data (7 bodies):\n');
    bodyNames = {'Base','LowerArm','Elbow','UpperArm','Wrist','Body6','Body7'};
    for bi = 1:7
        if collType(bi)==0, continue; end
        typeStr = {'Ball','Capsule','Lozenge'};
        d = dataM(bi,:);
        fprintf('    Body %d (%s, type=%s, r=%.1f):\n', bi, bodyNames{bi}, typeStr{collType(bi)}, radiusList(bi));
        if collType(bi)==2
            fprintf('      p1=[%.1f, %.1f, %.1f]  p2=[%.1f, %.1f, %.1f]\n', d(1),d(2),d(3), d(4),d(5),d(6));
        else
            fprintf('      center=[%.1f, %.1f, %.1f]\n', d(1),d(2),d(3));
        end
    end
    
    % Get FK
    tcpS = libstruct('MC_COORD_REF');
    tcpS.X=0; tcpS.Y=0; tcpS.Z=0; tcpS.A=0; tcpS.B=0; tcpS.C=0;
    [~, ~, tcpS] = calllib('libHRCInterface', 'forwardKinematics2', q_deg, tcpS);
    fprintf('  forwardKinematics2:\n');
    fprintf('    tcp = [%.4f, %.4f, %.4f] (X,Y,Z)\n', tcpS.X, tcpS.Y, tcpS.Z);
    fprintf('    orient = [%.1f, %.1f, %.1f] (A,B,C)\n', tcpS.A, tcpS.B, tcpS.C);
    
    % Collision distance
    pairArr = int64([0,0]); distVal = 0.0;
    [~,pairArr,distVal] = calllib('libHRCInterface','checkCPSelfCollisionInterface',pairArr,distVal);
    fprintf('  selfCollDist = %.1f mm\n', distVal);
    
    fprintf('\n');
end

% Quick analysis
fprintf('=== Analysis ===\n');
fprintf('If getUIInfo values are ~100-1000: unit is likely MM\n');
fprintf('If getUIInfo values are ~0.1-1.0: unit is likely M\n');
fprintf('If FK values are ~0.1-1.0: FK unit is M (confirmed by C++)\n');
fprintf('Base offset in MATLAB world: [0, 0, 0.8] m\n');

calllib('libHRCInterface','releaseACAreaConstrainInterface');
unloadlibrary('libHRCInterface');
fprintf('\n=== Done ===\n');
