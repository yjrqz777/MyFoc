defs = [];

% sfun_EmSvpwm
def = legacy_code('initialize');
def.SFunctionName = 'sfun_EmSvpwm';
def.OutputFcnSpec = 'void EmSvpwm(int16 u1, int16 u2, uint16 y1[1], uint16 y2[1], uint16 y3[1], uint16 y4[1])';
def.HeaderFiles   = {'em_svpwm.h'};
def.SourceFiles   = {'em_svpwm.c'};
defs = [defs; def];

legacy_code('sfcn_cmex_generate', defs);
legacy_code('compile', defs);
legacy_code('slblock_generate', defs, 'em_svpwm_model');
legacy_code('sfcn_tlc_generate', defs);


% mingw_path = 'D:\App\MinGW\mingw64';
% setenv('PATH', [mingw_path '\bin;' gete
% 
% 
% 
% nv('PATH')]);
% setenv('MW_MINGW64_LOC', mingw_path);