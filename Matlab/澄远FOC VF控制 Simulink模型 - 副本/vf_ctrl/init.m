% 任务1 start
% 从'trajectory_ctrl.h'导入C定义的PosCtrlHandle_t数据类型，供simulink模型使用
% Simulink.importExternalCTypes('trajectory_ctrl.h');
% 任务1 end


% 任务2 start
% 根据选择的电机类型，修改数据字典中相关的预定义数据

% 获取当前模型的数据字典对象、设计数据分区对象
dictionaryObj = Simulink.data.dictionary.open('vf_ctrl_dd.sldd');
dDataSectObj = getSection(dictionaryObj,'Design Data');

% 获取相关数据条目对象，读取预定义的电机类型
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_TG5P60');
MOTOR_TYPE_TG5P60_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_TB2P');
MOTOR_TYPE_TB2P_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE_JSF630');
MOTOR_TYPE_JSF630_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TYPE');
MOTOR_TYPE_param = getValue(entryObj);

% 获取需要修改的数据条目对象
entryObj = getEntry(dDataSectObj, 'MOTOR_PNF');
MOTOR_PNF_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_PN');
MOTOR_PN_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_RS');
MOTOR_RS_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_LS');
MOTOR_LS_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_KE');
MOTOR_KE_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_J');
MOTOR_J_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_F');
MOTOR_F_param = getValue(entryObj);
entryObj = getEntry(dDataSectObj, 'MOTOR_TL');
MOTOR_TL_param = getValue(entryObj);

% 根据电机类型设置不同的电机参数值
if MOTOR_TYPE_param.Value == MOTOR_TYPE_TG5P60_param.Value
    % 为电机TG5P60设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 5;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(5);    % 极对数：整形
    MOTOR_RS_param.Value = 0.27;        % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00035;     % 相电感：H
    MOTOR_KE_param.Value = 6.264;       % 反电势常数：V/krpm
    MOTOR_J_param.Value = 0.000015;     % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 0.000011;     % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m

elseif MOTOR_TYPE_param.Value == MOTOR_TYPE_TB2P_param.Value
    % 为电机TB2P设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 2;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(2);    % 极对数：整形
    MOTOR_RS_param.Value = 0.575;       % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00105;     % 相电感：H
    MOTOR_KE_param.Value = 5.96708;     % 反电势常数：V/krpm
    MOTOR_J_param.Value = 7.5E-6;       % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 6.3E-6;       % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m

elseif MOTOR_TYPE_param.Value == MOTOR_TYPE_JSF630_param.Value
    % 为电机TG5P40设置参数值
    % 电机相关参数
    MOTOR_PNF_param.Value = 4;          % 极对数：浮点
    MOTOR_PN_param.Value = uint8(4);    % 极对数：整形
    MOTOR_RS_param.Value = 0.445;       % 相电阻：Ω
    MOTOR_LS_param.Value = 0.00031;     % 相电感：H
    MOTOR_KE_param.Value = 5.656;       % 反电势常数：V/krpm
    MOTOR_J_param.Value = 0.0000028;    % 转动惯量：kg.m^2
    MOTOR_F_param.Value = 0.000007;     % 阻尼系数：N.m.s
    MOTOR_TL_param.Value = 0.001;       % 模拟的空载转矩：N.m
    
else
    error('未知的电机类型');
end

% 获取相关数据条目对象，将上面配置的参数写入
entryObj = getEntry(dDataSectObj, 'MOTOR_PNF');
setValue(entryObj, MOTOR_PNF_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_PN');
setValue(entryObj, MOTOR_PN_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_RS');
setValue(entryObj, MOTOR_RS_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_LS');
setValue(entryObj, MOTOR_LS_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_KE');
setValue(entryObj, MOTOR_KE_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_J');
setValue(entryObj, MOTOR_J_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_F');
setValue(entryObj, MOTOR_F_param);
entryObj = getEntry(dDataSectObj, 'MOTOR_TL');
setValue(entryObj, MOTOR_TL_param);

% 保存数据字典的更改
saveChanges(dictionaryObj);

% 任务2 end

