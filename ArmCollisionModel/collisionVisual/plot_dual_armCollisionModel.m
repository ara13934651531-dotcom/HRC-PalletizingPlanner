function outputStruct = plot_dual_armCollisionModel(jointPositions, params, params_tool)
    global T6T_l T6T_r;  % 左右臂工具坐标系全局变量
    global d1 d4 d6 a2;  % 左臂DH参数
    
    % 提取DH参数
    d1 = params.DH.d1; 
    d4 = params.DH.d4; 
    d6 = params.DH.d6; 
    a2 = params.DH.a2;
    

    % 设置左右臂工具在stand坐标系中的位姿
    T6T_l = rpy2Rotation(0, 0, 0.1, 0, 0, 0);  % 左臂工具变换
    T6T_r = rpy2Rotation(0, 0, -0.1, 0, 0, 0);  % 右臂工具变换
    
    % 计算左臂正运动学，得到各坐标系变换矩阵
    ql(1,6)=0;
    ql=jointPositions(1:6);
    [T00_l, T01_l, T02_l, T03_l, T04_l, T05_l, T06_l, T0T_l] = dual_arm_FKl(ql);
    Tf_tree_l = {T00_l, T01_l, T02_l, T03_l, T04_l, T05_l, T06_l, T0T_l};
  
    % 计算右臂正运动学，得到各坐标系变换矩阵
    qr(1,6)=0;
    qr=jointPositions(7:12);
    [T00_r, T01_r, T02_r, T03_r, T04_r, T05_r, T06_r, T0T_r] =dual_arm_FKr(qr);
    Tf_tree_r = {T00_r, T01_r, T02_r, T03_r, T04_r, T05_r, T06_r, T0T_r};
    
    % 绘制左右臂碰撞模型并返回结果
    outputStruct.l = dual_arm_left_plotSelfCollisonModel(Tf_tree_l, params, params_tool); hold on;
    outputStruct.r = dual_arm_right_plotSelfCollisonModel(Tf_tree_r, params, params_tool);hold on;
 
end

