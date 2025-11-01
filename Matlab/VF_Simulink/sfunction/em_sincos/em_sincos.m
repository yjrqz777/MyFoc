defs = [];

% sfun_EmSin
def = legacy_code('initialize');
def.SFunctionName = 'sfun_EmSin';
def.OutputFcnSpec = 'int16 y1 = EmSin( int16 u1 )';
def.HeaderFiles   = {'em_sincos.h'};
def.SourceFiles   = {'em_sincos.c'};
defs = [defs; def];

% sfun_EmCos
def = legacy_code('initialize');
def.SFunctionName = 'sfun_EmCos';
def.OutputFcnSpec = 'int16 y1 = EmCos( int16 u1 )';
def.HeaderFiles   = {'em_sincos.h'};
def.SourceFiles   = {'em_sincos.c'};
defs = [defs; def];

legacy_code('sfcn_cmex_generate', defs);
legacy_code('compile', defs);
legacy_code('slblock_generate', defs, 'em_sincos_model');
legacy_code('sfcn_tlc_generate', defs);


% mingw_path = 'D:\App\MinGW\mingw64';

% 将编译器路径添加到系统路径
%  setenv('PATH', [mingw_path '\bin;' getenv('PATH')]);
% setenv('MW_MINGW64_LOC', mingw_path);